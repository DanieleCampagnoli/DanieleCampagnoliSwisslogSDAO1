from ortools.sat.python import cp_model
import random


# =====================================================
# NEIGHBORS
# =====================================================
def neighbors(pos, grid, obstacles):
    x, y = pos
    for dx, dy in [(0,0),(1,0),(-1,0),(0,1),(0,-1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < grid[0] and 0 <= ny < grid[1] and (nx, ny) not in obstacles:
            yield (nx, ny)


# =====================================================
# CONFLICT DETECTION (O(N))
# =====================================================
def detect_conflicts(prev_positions, curr_positions):
    conflicts = []

    # vertex
    occupied = {}
    for a, pos in enumerate(curr_positions):
        if pos in occupied:
            conflicts.append((occupied[pos], a))
        else:
            occupied[pos] = a

    # edge
    transitions = {}
    for a in range(len(prev_positions)):
        move = (prev_positions[a], curr_positions[a])
        reverse = (move[1], move[0])

        if reverse in transitions:
            conflicts.append((transitions[reverse], a))
        else:
            transitions[move] = a

    return conflicts


# =====================================================
# UNION FIND (cluster conflicts)
# =====================================================
def build_conflict_clusters(conflicts, n_agents):
    parent = list(range(n_agents))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        parent[find(a)] = find(b)

    for a, b in conflicts:
        union(a, b)

    clusters = {}
    for i in range(n_agents):
        root = find(i)
        clusters.setdefault(root, []).append(i)

    return [c for c in clusters.values() if len(c) > 1]


# =====================================================
# CP-SAT ON SMALL CLUSTER
# =====================================================
def solve_cluster(cluster, starts, goals, grid, obstacles):
    model = cp_model.CpModel()
    n = len(cluster)

    x, y = {}, {}

    for i in range(n):
        x[i] = model.NewIntVar(0, grid[0]-1, "")
        y[i] = model.NewIntVar(0, grid[1]-1, "")

    for i in range(n):
        sx, sy = starts[i]

        dx = model.NewIntVar(-1,1,"")
        dy = model.NewIntVar(-1,1,"")

        model.Add(x[i] == sx + dx)
        model.Add(y[i] == sy + dy)

        abs_dx = model.NewIntVar(0,1,"")
        abs_dy = model.NewIntVar(0,1,"")

        model.AddAbsEquality(abs_dx, dx)
        model.AddAbsEquality(abs_dy, dy)
        model.Add(abs_dx + abs_dy <= 1)

        for ox, oy in obstacles:
            model.AddForbiddenAssignments([x[i], y[i]], [(ox, oy)])

    # avoid vertex conflict in cluster
    for i in range(n):
        for j in range(i+1, n):
            model.Add(x[i] != x[j] + grid[0]*(y[j]-y[i]))

    # objective
    obj = []
    for i in range(n):
        gx, gy = goals[i]

        dx = model.NewIntVar(0,grid[0],"")
        dy = model.NewIntVar(0,grid[1],"")

        model.AddAbsEquality(dx, x[i]-gx)
        model.AddAbsEquality(dy, y[i]-gy)

        obj.append(dx+dy)

    model.Minimize(sum(obj))

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 0.5

    if solver.Solve(model) not in (cp_model.FEASIBLE, cp_model.OPTIMAL):
        return None

    return [(solver.Value(x[i]), solver.Value(y[i])) for i in range(n)]


# =====================================================
# MAIN SIMULATION
# =====================================================
def simulate(agents, grid_size, obstacles, steps=300):
    n = len(agents)
    current = [dict(a) for a in agents]
    paths = [[a["start"]] for a in agents]

    vertex_res = {}
    edge_res = {}

    for t in range(1, steps+1):
        vertex_res.setdefault(t, set())
        edge_res.setdefault(t, set())

        prev = [a["start"] for a in current]

        # =================================================
        # STEP 1: GREEDY PROPOSALS
        # =================================================
        proposals = []

        for i, a in enumerate(current):
            x, y = a["start"]
            gx, gy = a["goal"]

            best = (x, y)
            best_dist = abs(x-gx)+abs(y-gy)

            for nx, ny in neighbors((x,y), grid_size, obstacles):
                d = abs(nx-gx)+abs(ny-gy)
                if d < best_dist:
                    best = (nx, ny)
                    best_dist = d

            proposals.append(best)

        # =================================================
        # STEP 2: DETECT CONFLICTS
        # =================================================
        conflicts = detect_conflicts(prev, proposals)

        # =================================================
        # STEP 3: RESOLVE WITH CP-SAT (only clusters)
        # =================================================
        if conflicts:
            clusters = build_conflict_clusters(conflicts, n)

            for cluster in clusters:
                starts = [prev[i] for i in cluster]
                goals  = [agents[i]["goal"] for i in cluster]

                sol = solve_cluster(cluster, starts, goals, grid_size, obstacles)

                if sol:
                    for idx, agent_id in enumerate(cluster):
                        proposals[agent_id] = sol[idx]

        # =================================================
        # STEP 4: RESERVATION SAFE EXECUTION
        # =================================================
        order = list(range(n))
        random.shuffle(order)

        next_pos = [None]*n

        for i in order:
            curr = current[i]["start"]

            for nx, ny in [proposals[i]] + list(neighbors(curr, grid_size, obstacles)):

                # vertex
                if (nx, ny) in vertex_res[t]:
                    continue

                # edge swap
                if ((nx, ny), curr) in edge_res[t]:
                    continue

                move = (nx, ny)
                break
            else:
                move = curr

            vertex_res[t].add(move)
            edge_res[t].add((curr, move))

            next_pos[i] = move

        # =================================================
        # STEP 5: APPLY
        # =================================================
        for i in range(n):
            current[i]["start"] = next_pos[i]
            paths[i].append(next_pos[i])

    # pad
    max_len = max(len(p) for p in paths)
    for i in range(n):
        paths[i] += [paths[i][-1]] * (max_len - len(paths[i]))

    return paths


# =====================================================
# API Greedy → Detect → Fix locally → Reserve → Execute
# in this version if a robot is stuck because of an obstacle
# it will stay there forever
# =====================================================
class ORToolsPlannerV3:

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