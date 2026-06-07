from ortools.sat.python import cp_model
from collections import defaultdict
import heapq
import logging
# logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# =====================================================
# LOCAL CP-SAT
# =====================================================
def plan_single(grid, obstacles, start, goal, horizon=12):
    model = cp_model.CpModel()
    W, H = grid

    x = [model.NewIntVar(0, W - 1, f"x_{t}") for t in range(horizon)]
    y = [model.NewIntVar(0, H - 1, f"y_{t}") for t in range(horizon)]

    model.Add(x[0] == start[0])
    model.Add(y[0] == start[1])

    for t in range(horizon - 1):
        dx = model.NewIntVar(-1, 1, "")
        dy = model.NewIntVar(-1, 1, "")

        model.Add(x[t+1] == x[t] + dx)
        model.Add(y[t+1] == y[t] + dy)

        ax = model.NewIntVar(0, 1, "")
        ay = model.NewIntVar(0, 1, "")

        model.AddAbsEquality(ax, dx)
        model.AddAbsEquality(ay, dy)
        model.Add(ax + ay <= 1)

    for t in range(horizon):
        for (ox, oy) in obstacles:
            model.AddForbiddenAssignments([x[t], y[t]], [(ox, oy)])

    gx, gy = goal
    dx = model.NewIntVar(0, W, "")
    dy = model.NewIntVar(0, H, "")

    model.AddAbsEquality(dx, x[-1] - gx)
    model.AddAbsEquality(dy, y[-1] - gy)

    model.Minimize(dx + dy)

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 0.05

    if solver.Solve(model) not in (cp_model.FEASIBLE, cp_model.OPTIMAL):
        return []

    return [(solver.Value(x[t]), solver.Value(y[t])) for t in range(horizon)]


# =====================================================
# REGIONS
# =====================================================
REGION_SIZE = 5
region_congestion = defaultdict(int)


def get_region(p):
    return (p[0] // REGION_SIZE, p[1] // REGION_SIZE)


def update_congestion(positions):
    region_congestion.clear()
    for p in positions:
        region_congestion[get_region(p)] += 1


def region_neighbors(r):
    x, y = r
    return [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]


def region_cost(r):
    return 1 + 2 * region_congestion[r]


def compute_region_path(start, goal):
    start_r = get_region(start)
    goal_r = get_region(goal)

    pq = [(0, start_r)]
    parent = {start_r: None}
    cost = {start_r: 0}

    while pq:
        _, r = heapq.heappop(pq)
        if r == goal_r:
            break

        for nr in region_neighbors(r):
            new_cost = cost[r] + region_cost(nr)
            if nr not in cost or new_cost < cost[nr]:
                cost[nr] = new_cost
                parent[nr] = r
                h = abs(nr[0]-goal_r[0]) + abs(nr[1]-goal_r[1])
                heapq.heappush(pq, (new_cost + h, nr))

    path = []
    cur = goal_r
    while cur is not None:
        path.append(cur)
        cur = parent.get(cur)

    return list(reversed(path))


def pick_cell_in_region(region, goal):
    rx, ry = region
    cells = [(x, y)
             for x in range(rx * REGION_SIZE, (rx + 1) * REGION_SIZE)
             for y in range(ry * REGION_SIZE, (ry + 1) * REGION_SIZE)]

    return min(cells, key=lambda c: abs(c[0]-goal[0]) + abs(c[1]-goal[1]))


# =====================================================
# MAIN PLANNER - Best One 
# =====================================================
class HierarchicalPlanner:

    def __init__(self, grid_size, obstacles):
        self.grid = grid_size
        self.obstacles = obstacles

    def plan(self, agents, steps=300):
        plan_successful=False   
        current = [dict(a) for a in agents]
        paths = [[a["start"]] for a in agents]

        active_plans = {i: [] for i in range(len(agents))}
        last_pos = {i: agents[i]["start"] for i in range(len(agents))}
        stuck = {i: 0 for i in range(len(agents))}

        priority = list(range(len(agents)))
        step_nr=0   
        for _ in range(steps):
            logger.info(f"{step_nr}-PLAN STATE######### ")
            logger.info(f"*****current*****")
            logger.info(f"{current}")
            step_nr+=1

            update_congestion([a["start"] for a in current])

            proposals = {}

            # =====================================================
            # PHASE 1: PROPOSE
            # =====================================================
            for i in priority:
                pos = current[i]["start"]
                goal = agents[i]["goal"]

                if pos == last_pos[i]:
                    stuck[i] += 1
                else:
                    stuck[i] = 0
                last_pos[i] = pos

                if stuck[i] >= 3 or len(active_plans[i]) < 2:
                    rp = compute_region_path(pos, goal)
                    target = goal if len(rp) == 1 else pick_cell_in_region(rp[1], goal)

                    plan = plan_single(self.grid, self.obstacles, pos, target)
                    if plan:
                        active_plans[i] = plan[1:]

                proposed = active_plans[i][0] if active_plans[i] else pos

                if abs(pos[0]-goal[0]) + abs(pos[1]-goal[1]) <= 1:
                    proposed = goal

                proposals[i] = proposed

                logger.debug(
                    f"[PROPOSE] Agent {i} pos={pos} -> proposed={proposed} goal={goal} stuck={stuck[i]}"
                )

            # =====================================================
            # PHASE 2: STRICT GLOBAL RESOLUTION
            # =====================================================
            reserved = set()
            reserved_edges = set()
            final_moves = {}

            # ✅ STEP 1 — lock waiting agents
            for i in priority:
                pos = current[i]["start"]
                if proposals[i] == pos:
                    final_moves[i] = pos
                    reserved.add(pos)

            # ✅ STEP 2 — resolve movers
            cell_to_agents = defaultdict(list)
            for i, p in proposals.items():
                if p != current[i]["start"]:
                    cell_to_agents[p].append(i)

            for cell, agents_here in cell_to_agents.items():
                if len(agents_here) > 1:
                    logger.debug(f"[CONFLICT] Cell {cell} requested by agents {agents_here}")

                # block entering occupied cell
                if cell in reserved:
                    for i in agents_here:
                        final_moves[i] = current[i]["start"]
                    continue

                winner = min(agents_here, key=lambda x: priority.index(x))

                for i in agents_here:
                    pos = current[i]["start"]
                    proposed = proposals[i]
                    reverse_edge = (proposed, pos)

                    if i == winner and reverse_edge not in reserved_edges:
                        move = proposed
                    else:
                        move = pos

                    final_moves[i] = move
                    reserved.add(move)
                    reserved_edges.add((pos, move))

            # =====================================================
            # PHASE 3: APPLY
            # =====================================================
            for i in range(len(agents)):
                move = final_moves.get(i, current[i]["start"])

                if active_plans[i] and move == proposals[i]:
                    active_plans[i].pop(0)

                if move == agents[i]["goal"]:
                    active_plans[i] = []

                current[i]["start"] = move
                paths[i].append(move)
                
                prev = paths[i][-2]
                curr = paths[i][-1]

                if prev == curr:
                    logger.debug(f"[WAIT] Agent {i} stayed at {curr}")
                else:
                    logger.debug(f"[MOVE] Agent {i} {prev} -> {curr}")

            # log number of moves,  If it becomes 0 → hard deadlock
            if len(paths[0]) > 1:
                moves = sum(
                1 for i in range(len(agents))
                    if paths[i][-1] != paths[i][-2]
                )
            logger.info(f"Moves this step: {moves}")
            # stuck agents  
            stuck_agents = [i for i in range(len(agents)) if stuck[i] >= 3]
            logger.info(f"Stuck agents count: {len(stuck_agents)}")
            
            for i in range(len(agents)):
                if stuck[i] >= 3:
                    logger.warning(
                        f"[STUCK] Agent {i} at {current[i]['start']} goal={agents[i]['goal']} stuck={stuck[i]}"
                    )

            # fairness rotation
            priority = priority[1:] + priority[:1]

            if all(current[i]["start"] == agents[i]["goal"]
                   for i in range(len(agents))):
                plan_successful=True
                break
        return plan_successful,paths
