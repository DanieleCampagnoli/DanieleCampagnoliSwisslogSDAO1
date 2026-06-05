from ortools.sat.python import cp_model


def plan_paths(grid_size, obstacles, starts, goals, max_time=None):
    model = cp_model.CpModel()

    num_agents = len(starts)
    width, height = grid_size

    # ✅ better horizon
    if max_time is None:
        max_time = width + height

    x = {}
    y = {}

    # ================= VARIABLES =================
    for a in range(num_agents):
        for t in range(max_time):
            x[a, t] = model.NewIntVar(0, width - 1, f"x_{a}_{t}")
            y[a, t] = model.NewIntVar(0, height - 1, f"y_{a}_{t}")

    # ================= START + GOAL =================
    for a in range(num_agents):
        model.Add(x[a, 0] == starts[a][0])
        model.Add(y[a, 0] == starts[a][1])

        # force goal at last step
        model.Add(x[a, max_time - 1] == goals[a][0])
        model.Add(y[a, max_time - 1] == goals[a][1])

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

            # ✅ no diagonal moves, no jumps
            model.Add(abs_dx + abs_dy <= 1)

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
                model.AddBoolOr([same_x.Not(), same_y.Not()]).OnlyEnforceIf(same_cell.Not())

                # ✅ forbid same cell
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
                    [a_to_b_x.Not(), a_to_b_y.Not(), b_to_a_x.Not(), b_to_a_y.Not()]
                ).OnlyEnforceIf(swap.Not())

                # ✅ forbid swap
                model.Add(swap == 0)

    # ================= SOLVE =================
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 5

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