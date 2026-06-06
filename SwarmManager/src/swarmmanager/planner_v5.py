from ortools.sat.python import cp_model
from collections import defaultdict, deque
import random


# ==========================================
# UTIL
# ==========================================
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def neighbors(pos, grid, obstacles):
    x, y = pos
    for dx, dy in [(0,0),(1,0),(-1,0),(0,1),(0,-1)]:
        nx, ny = x + dx, y + dy
        if (
            0 <= nx < grid[0] and
            0 <= ny < grid[1] and
            (nx, ny) not in obstacles
        ):
            yield (nx, ny)


# ==========================================
# HELPER: cooperative behavior
# ==========================================
def has_stagnating_neighbor(i, current, stagnation):
    xi, yi = current[i]["start"]

    for j in range(len(current)):
        if i == j:
            continue

        xj, yj = current[j]["start"]

        if abs(xi - xj) <= 1 and abs(yi - yj) <= 1:
            if stagnation[j] >= 2:
                return True

    return False


# ==========================================
# CP-SAT (conflict resolver)
# ==========================================
def resolve_with_cpsat(starts, goals, grid, obstacles):
    model = cp_model.CpModel()
    n = len(starts)
    W, H = grid

    x, y = {}, {}

    for a in range(n):
        x[a] = model.NewIntVar(0, W - 1, "")
        y[a] = model.NewIntVar(0, H - 1, "")

    for a in range(n):
        sx, sy = starts[a]

        dx = model.NewIntVar(-1, 1, "")
        dy = model.NewIntVar(-1, 1, "")

        model.Add(x[a] == sx + dx)
        model.Add(y[a] == sy + dy)

        abs_dx = model.NewIntVar(0, 1, "")
        abs_dy = model.NewIntVar(0, 1, "")

        model.AddAbsEquality(abs_dx, dx)
        model.AddAbsEquality(abs_dy, dy)
        model.Add(abs_dx + abs_dy <= 1)

    # obstacles
    for a in range(n):
        for (ox, oy) in obstacles:
            model.AddForbiddenAssignments([x[a], y[a]], [(ox, oy)])

    # collisions (vertex + swap)
    for i in range(n):
        for j in range(i + 1, n):

            same = model.NewBoolVar("")
            model.Add(x[i] == x[j]).OnlyEnforceIf(same)
            model.Add(y[i] == y[j]).OnlyEnforceIf(same)
            model.Add(x[i] != x[j]).OnlyEnforceIf(same.Not())
            model.Add(y[i] != y[j]).OnlyEnforceIf(same.Not())
            model.Add(same == 0)

            si, sj = starts[i], starts[j]

            b1 = model.NewBoolVar("")
            b2 = model.NewBoolVar("")

            model.Add(x[i] == sj[0]).OnlyEnforceIf(b1)
            model.Add(y[i] == sj[1]).OnlyEnforceIf(b1)
            model.Add(x[j] == si[0]).OnlyEnforceIf(b2)
            model.Add(y[j] == si[1]).OnlyEnforceIf(b2)

            swap = model.NewBoolVar("")
            model.AddBoolAnd([b1, b2]).OnlyEnforceIf(swap)
            model.AddBoolOr([b1.Not(), b2.Not()]).OnlyEnforceIf(swap.Not())
            model.Add(swap == 0)

    # objective
    cost = []
    for a in range(n):
        gx, gy = goals[a]
        dx = model.NewIntVar(0, W, "")
        dy = model.NewIntVar(0, H, "")
        model.AddAbsEquality(dx, x[a] - gx)
        model.AddAbsEquality(dy, y[a] - gy)
        cost.append(dx + dy)

    model.Minimize(sum(cost))

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 0.05  # faster

    if solver.Solve(model) not in (cp_model.FEASIBLE, cp_model.OPTIMAL):
        return None

    return [(solver.Value(x[a]), solver.Value(y[a])) for a in range(n)]


# ==========================================
# MAIN SIMULATION
# ==========================================
def simulate_hybrid(agents, grid, obstacles, steps=2000):
    current = [dict(a) for a in agents]
    paths = [[a["start"]] for a in agents]

    previous = {}
    stagnation = {i: 0 for i in range(len(current))}
    stagnation_visits = defaultdict(lambda: defaultdict(int))

    # ✅ NEW: short-term memory (prevents loops)
    history = {i: deque(maxlen=6) for i in range(len(current))}

    for _ in range(steps):

        # ---- priority (farther first)
        order = sorted(
            range(len(current)),
            key=lambda i: manhattan(current[i]["start"], agents[i]["goal"]),
            reverse=True
        )

        proposals = {}

        # ===== SMART MOVE SELECTION =====
        for i in order:
            curr = current[i]["start"]
            goal = agents[i]["goal"]
            prev = previous.get(i)

            # ---- goal lock with cooperation
            if curr == goal and not has_stagnating_neighbor(i, current, stagnation):
                proposals[i] = curr
                continue

            best = curr
            best_dist = manhattan(curr, goal)

            strict = []
            relaxed = []

            for nxt in neighbors(curr, grid, obstacles):

                # no backtracking
                if prev is not None and nxt == prev:
                    continue

                d = manhattan(nxt, goal)
                penalty = stagnation_visits[i][nxt]

                # ✅ anti-cycle (history)
                if nxt in history[i]:
                    penalty += 100

                if d < best_dist:
                    strict.append((d, penalty, nxt))

                elif d == best_dist:
                    relaxed.append((d, penalty, nxt))

                elif stagnation[i] >= 2 and d == best_dist + 1:
                    relaxed.append((d, penalty, nxt))

            if strict:
                strict.sort(key=lambda x: (x[0], x[1]))
                best = strict[0][2]

            elif relaxed:
                relaxed.sort(key=lambda x: (x[0], x[1]))
                best = relaxed[0][2]

            # ✅ STRONG ESCAPE MODE (deep traps)
            if stagnation[i] >= 5:
                options = [
                    nxt for nxt in neighbors(curr, grid, obstacles)
                    if nxt != prev
                ]
                if options:
                    best = random.choice(options)

            proposals[i] = best

        # ---- conflict detection
        cell_map = {}
        for i, p in proposals.items():
            cell_map.setdefault(p, []).append(i)

        final_moves = proposals.copy()

        # ---- CP-SAT resolution
        for ids in cell_map.values():
            if len(ids) > 1:
                starts = [current[i]["start"] for i in ids]
                goals = [agents[i]["goal"] for i in ids]

                resolved = resolve_with_cpsat(starts, goals, grid, obstacles)

                if resolved:
                    for k, idx in enumerate(ids):
                        final_moves[idx] = resolved[k]
                else:
                    for idx in ids:
                        final_moves[idx] = current[idx]["start"]

        # ---- EDGE SWAP FIX (global)
        changed = True
        while changed:
            changed = False
            transitions = {}

            for i in final_moves:
                prev_pos = current[i]["start"]
                move = final_moves[i]

                if (move, prev_pos) in transitions:
                    final_moves[i] = prev_pos
                    changed = True
                else:
                    transitions[(prev_pos, move)] = i

        # ---- VERTEX FIX (global)
        occupied = {}
        for i in final_moves:
            pos = final_moves[i]
            curr = current[i]["start"]

            if pos in occupied:
                for nxt in neighbors(curr, grid, obstacles):
                    if nxt not in occupied:
                        final_moves[i] = nxt
                        break
                else:
                    final_moves[i] = curr

            occupied[final_moves[i]] = i

        # ---- APPLY
        for i in range(len(current)):
            prev_pos = current[i]["start"]
            move = final_moves[i]

            # progress tracking
            if manhattan(move, agents[i]["goal"]) < manhattan(prev_pos, agents[i]["goal"]):
                stagnation[i] = 0
                stagnation_visits[i].clear()
                history[i].clear()
            else:
                stagnation[i] += 1

            if stagnation[i] >= 2:
                stagnation_visits[i][move] += 1

            history[i].append(move)
            previous[i] = prev_pos

            current[i]["start"] = move
            paths[i].append(move)

        # ---- stop early
        if all(current[i]["start"] == agents[i]["goal"] for i in range(len(agents))):
            break

    # ---- pad paths
    max_len = max(len(p) for p in paths)
    for i in range(len(paths)):
        paths[i] += [paths[i][-1]] * (max_len - len(paths[i]))

    return paths


# ==========================================
# API
# ==========================================
class ORToolsPlannerV5:

    def __init__(self, grid_size, obstacles):
        self.grid = grid_size
        self.obstacles = set(obstacles)

    def plan(self, agents, steps=600):
        return simulate_hybrid(
            agents,
            self.grid,
            self.obstacles,
            steps
        )