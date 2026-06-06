from ortools.sat.python import cp_model


from ortools.sat.python import cp_model


def plan_paths(grid_size, obstacles, starts, goals, max_time=None):
    model = cp_model.CpModel()

    num_agents = len(starts)
    width, height = grid_size

    if max_time is None:
        max_time = width + height

    x = {}
    y = {}
    move = {}
    reached = {}
    arrival_time = {}

    # ================= VARIABLES =================
    for a in range(num_agents):
        for t in range(max_time):
            x[a, t] = model.NewIntVar(0, width - 1, f"x_{a}_{t}")
            y[a, t] = model.NewIntVar(0, height - 1, f"y_{a}_{t}")
            reached[a, t] = model.NewBoolVar(f"reached_{a}_{t}")

        for t in range(max_time - 1):
            move[a, t] = model.NewBoolVar(f"move_{a}_{t}")

        arrival_time[a] = model.NewIntVar(0, max_time - 1, f"arrival_{a}")

    # ================= START =================
    for a in range(num_agents):
        model.Add(x[a, 0] == starts[a][0])
        model.Add(y[a, 0] == starts[a][1])

    # ================= GOAL (flexible) =================
    for a in range(num_agents):
        gx, gy = goals[a]

        for t in range(max_time):
            model.Add(x[a, t] == gx).OnlyEnforceIf(reached[a, t])
            model.Add(y[a, t] == gy).OnlyEnforceIf(reached[a, t])

        # must reach goal at least once
        model.Add(sum(reached[a, t] for t in range(max_time)) >= 1)

        # once reached, always reached
        for t in range(max_time - 1):
            model.Add(reached[a, t] <= reached[a, t+1])

        # arrival time = first time reached
        model.Add(
            arrival_time[a] == sum(t * reached[a, t] for t in range(max_time))
        )

    # ================= MOVEMENT =================
    for a in range(num_agents):
        for t in range(max_time - 1):
            dx = model.NewIntVar(-1, 1, f"dx_{a}_{t}")
            dy = model.NewIntVar(-1, 1, f"dy_{a}_{t}")

            model.Add(x[a, t+1] == x[a, t] + dx)
            model.Add(y[a, t+1] == y[a, t] + dy)

            abs_dx = model.NewIntVar(0, 1, f"abs_dx_{a}_{t}")
            abs_dy = model.NewIntVar(0, 1, f"abs_dy_{a}_{t}")

            model.AddAbsEquality(abs_dx, dx)
            model.AddAbsEquality(abs_dy, dy)

            # no diagonal, no jumps
            model.Add(abs_dx + abs_dy <= 1)

            # movement detection
            model.Add(abs_dx + abs_dy == 1).OnlyEnforceIf(move[a, t])
            model.Add(abs_dx + abs_dy == 0).OnlyEnforceIf(move[a, t].Not())

            # 💡 important: after reaching goal -> no movement
            model.Add(move[a, t] == 0).OnlyEnforceIf(reached[a, t])

    # ================= OBSTACLES =================
    if obstacles:
        for a in range(num_agents):
            for t in range(max_time):
                model.AddForbiddenAssignments(
                    [x[a, t], y[a, t]],
                    obstacles
                )

    # ================= VERTEX COLLISIONS =================
    for t in range(max_time):
        for i in range(num_agents):
            for j in range(i + 1, num_agents):

                same_x = model.NewBoolVar(f"same_x_{i}_{j}_{t}")
                same_y = model.NewBoolVar(f"same_y_{i}_{j}_{t}")

                model.Add(x[i, t] == x[j, t]).OnlyEnforceIf(same_x)
                model.Add(x[i, t] != x[j, t]).OnlyEnforceIf(same_x.Not())

                model.Add(y[i, t] == y[j, t]).OnlyEnforceIf(same_y)
                model.Add(y[i, t] != y[j, t]).OnlyEnforceIf(same_y.Not())

                same_cell = model.NewBoolVar(f"same_cell_{i}_{j}_{t}")
                model.AddBoolAnd([same_x, same_y]).OnlyEnforceIf(same_cell)
                model.AddBoolOr(
                    [same_x.Not(), same_y.Not()]
                ).OnlyEnforceIf(same_cell.Not())

                model.Add(same_cell == 0)

    # ================= EDGE COLLISIONS =================
    for t in range(max_time - 1):
        for i in range(num_agents):
            for j in range(i + 1, num_agents):

                a_to_b_x = model.NewBoolVar(f"a_to_b_x_{i}_{j}_{t}")
                a_to_b_y = model.NewBoolVar(f"a_to_b_y_{i}_{j}_{t}")
                b_to_a_x = model.NewBoolVar(f"b_to_a_x_{i}_{j}_{t}")
                b_to_a_y = model.NewBoolVar(f"b_to_a_y_{i}_{j}_{t}")

                model.Add(x[i, t] == x[j, t+1]).OnlyEnforceIf(a_to_b_x)
                model.Add(x[i, t] != x[j, t+1]).OnlyEnforceIf(a_to_b_x.Not())

                model.Add(y[i, t] == y[j, t+1]).OnlyEnforceIf(a_to_b_y)
                model.Add(y[i, t] != y[j, t+1]).OnlyEnforceIf(a_to_b_y.Not())

                model.Add(x[j, t] == x[i, t+1]).OnlyEnforceIf(b_to_a_x)
                model.Add(x[j, t] != x[i, t+1]).OnlyEnforceIf(b_to_a_x.Not())

                model.Add(y[j, t] == y[i, t+1]).OnlyEnforceIf(b_to_a_y)
                model.Add(y[j, t] != y[i, t+1]).OnlyEnforceIf(b_to_a_y.Not())

                swap = model.NewBoolVar(f"swap_{i}_{j}_{t}")

                model.AddBoolAnd(
                    [a_to_b_x, a_to_b_y, b_to_a_x, b_to_a_y]
                ).OnlyEnforceIf(swap)

                model.AddBoolOr(
                    [a_to_b_x.Not(), a_to_b_y.Not(),
                     b_to_a_x.Not(), b_to_a_y.Not()]
                ).OnlyEnforceIf(swap.Not())

                model.Add(swap == 0)

    # ================= OBJECTIVE =================
    model.Minimize(
        1000 * sum(arrival_time[a] for a in range(num_agents)) +
        sum((t + 1) * move[a, t]
            for a in range(num_agents)
            for t in range(max_time - 1))
    )

    # ================= SOLVE =================
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 300
    solver.parameters.num_search_workers = 8

    result = solver.Solve(model)

    if result not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return False, []

    # ================= OUTPUT =================
    paths = []
    for a in range(num_agents):
        path = []
        for t in range(max_time):
            path.append((solver.Value(x[a, t]), solver.Value(y[a, t])))
        paths.append(path)

    return True, paths


# ================= API CLASS =================
class ORToolsPlanner:

    def __init__(self, grid_size, obstacles):
        self.grid_size = grid_size
        self.obstacles = obstacles

    def plan(self, agents):
        starts = [a["start"] for a in agents]
        goals  = [a["goal"]  for a in agents]

        return plan_paths(
            self.grid_size,
            self.obstacles,
            starts,
            goals
        )