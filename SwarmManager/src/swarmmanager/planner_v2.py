from ortools.sat.python import cp_model
import random


# ==========================================
# CP-SAT ONE STEP (guidance only)
# ==========================================
def plan_one_step(grid_size, obstacles, starts, goals):
    model = cp_model.CpModel()

    n = len(starts)
    W, H = grid_size

    x, y = {}, {}

    for a in range(n):
        x[a] = model.NewIntVar(0, W - 1, f"x_{a}")
        y[a] = model.NewIntVar(0, H - 1, f"y_{a}")

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
        for ox, oy in obstacles:
            model.AddForbiddenAssignments([x[a], y[a]], [(ox, oy)])

    # vertex collision (local only)
    for i in range(n):
        for j in range(i + 1, n):
            model.Add(x[i] != x[j] + W * (y[j] - y[i]))

    # objective: move toward goal
    costs = []
    for a in range(n):
        gx, gy = goals[a]

        dx = model.NewIntVar(0, W, "")
        dy = model.NewIntVar(0, H, "")

        model.AddAbsEquality(dx, x[a] - gx)
        model.AddAbsEquality(dy, y[a] - gy)

        costs.append(dx + dy)

    model.Minimize(sum(costs))

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 1

    if solver.Solve(model) not in (cp_model.FEASIBLE, cp_model.OPTIMAL):
        return False, []

    return True, [(solver.Value(x[a]), solver.Value(y[a])) for a in range(n)]


# ==========================================
# NEIGHBORS (fallback options)
# ==========================================
def neighbors(pos, grid, obstacles):
    x, y = pos
    for dx, dy in [(0,0),(1,0),(-1,0),(0,1),(0,-1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < grid[0] and 0 <= ny < grid[1]:
            if (nx, ny) not in obstacles:
                yield (nx, ny)


# ==========================================
# MAIN SIMULATION (SAFE)
# ==========================================
def simulate(agents, grid_size, obstacles, steps=600):
    current = [dict(a) for a in agents]
    paths = [[a["start"]] for a in agents]

    # reservation tables
    vertex_res = {}
    edge_res = {}

    for t in range(1, steps + 1):
        vertex_res.setdefault(t, set())
        vertex_res.setdefault(t-1, set())
        edge_res.setdefault(t, set())

        starts = [a["start"] for a in current]
        goals  = [a["goal"]  for a in agents]

        success, cp_moves = plan_one_step(
            grid_size, obstacles, starts, goals
        )

        proposals = {}

        for i, agent in enumerate(current):
            x, y = agent["start"]
            gx, gy = goals[i]

            if success:
                proposals[i] = cp_moves[i]
            else:
                # fallback
                best = (x, y)
                best_dist = abs(x-gx)+abs(y-gy)

                for nx, ny in neighbors((x,y), grid_size, obstacles):
                    d = abs(nx-gx)+abs(ny-gy)
                    if d < best_dist:
                        best = (nx, ny)
                        best_dist = d

                proposals[i] = best

        # resolve with reservation tables
        order = list(range(len(current)))
        random.shuffle(order)

        next_positions = {}

        for i in order:
            curr = current[i]["start"]

            candidates = [proposals[i]] + list(neighbors(curr, grid_size, obstacles))

            chosen = curr

            for nx, ny in candidates:

                # ✅ vertex collision
                if (nx, ny) in vertex_res[t]:
                    continue

                # ✅ edge swap (CRITICAL FIX)
                if ((nx, ny), curr) in edge_res[t]:
                    continue

                chosen = (nx, ny)
                break

            # record reservations
            vertex_res[t].add(chosen)
            edge_res[t].add((curr, chosen))

            next_positions[i] = chosen

        # apply
        for i in range(len(current)):
            current[i]["start"] = next_positions[i]
            paths[i].append(next_positions[i])

    # pad paths
    max_len = max(len(p) for p in paths)
    for i in range(len(paths)):
        paths[i] += [paths[i][-1]] * (max_len - len(paths[i]))

    return paths


# ==========================================
# API
# ==========================================
class ORToolsPlannerV2:

    def __init__(self, grid_size, obstacles):
        self.grid_size = grid_size
        self.obstacles = set(obstacles)

    def plan(self, agents, steps):
        return simulate(
            agents,
            self.grid_size,
            self.obstacles,
            steps
        )