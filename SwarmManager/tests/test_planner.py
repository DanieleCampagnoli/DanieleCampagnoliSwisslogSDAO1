
from swarmmanager.planner import ORToolsPlanner
from swarmmanager.planner_v2 import ORToolsPlannerV2
from swarmmanager.planner_v3 import ORToolsPlannerV3
from swarmmanager.planner_v4 import ORToolsPlannerV4
from swarmmanager.planner_v6 import HierarchicalPlanner
from swarmmanager.planner_v7 import LaCAMPlanner

def test_planner():
    
    planner = ORToolsPlanner(
        grid_size=(10,10),
        obstacles=[(2,2), (3,3)]
    )

    agents = [
        {"start": (0,0), "goal": (9,9)},
        {"start": (9,0), "goal": (0,9)}
    ]

    success, paths = planner.plan(agents)
    print(success)
    print(paths)

def test_grid_division_v2():
    
    planner = ORToolsPlannerV2(
        grid_size=(20,20),
        obstacles=[(10, 10), (10, 11), (10, 12)]
    )
    
    # -----------------------------
    # Agents
    # -----------------------------
    agents = [

    {"start": (1, 1), "goal": (15, 15)},
    {"start": (3, 1), "goal": (15, 14)},
    {"start": (1, 3), "goal": (14, 15)},
    {"start": (3, 3), "goal": (14, 14)},
    {"start": (5, 5), "goal": (18, 18)},

    ]
    
    # -----------------------------
    # Run planner
    # -----------------------------
    paths= planner.plan_division(
            agents=agents,
            steps=600)
    
        # -----------------------------
        # 1. Check all agents reached goal
        # -----------------------------
    for i, agent in enumerate(agents):
            final_pos = paths[i][-1]
            fx, fy = final_pos
            gx, gy = agent["goal"]
            assert abs(fx - gx) + abs(fy - gy) <= 1, \
                f"Agent {i} did not reach goal"


        # -----------------------------
        # 2. Check no vertex collisions
        # -----------------------------
    max_len = max(len(p) for p in paths)

    for t in range(max_len):
            occupied = {}

            for i, path in enumerate(paths):
                if t >= len(path):
                    continue

                pos = path[t]

                
                assert pos not in occupied, \
                   f"Vertex collision at time {t} between agents {i} and {occupied.get(pos)}"

                occupied[pos] = i

        # -----------------------------
        # 3. Check no edge swaps
        # -----------------------------
    for t in range(max_len - 1):
            transitions = {}

            for i, path in enumerate(paths):
                if t + 1 >= len(path):
                    continue

                move = (path[t], path[t + 1])

                # Check reverse move exists
                reverse = (move[1], move[0])
                assert reverse not in  transitions, f"Edge swap at time {t}: agent {i}" 
                transitions[move] = i




def test_grid_division_v3():
    
    planner = ORToolsPlannerV3(
        grid_size=(20,20),
        obstacles=[(10, 10), (10, 11), (10, 12)]
    )
    
    # -----------------------------
    # Agents
    # -----------------------------
    agents = [

    {"start": (1, 1), "goal": (15, 15)},
    {"start": (3, 1), "goal": (15, 14)},
    {"start": (1, 3), "goal": (14, 15)},
    {"start": (3, 3), "goal": (14, 14)},
    {"start": (5, 5), "goal": (18, 18)},

    ]
    
    # -----------------------------
    # Run planner
    # -----------------------------
    paths= planner.plan_division(
            agents=agents,
            steps=600)
    
        # -----------------------------
        # 1. Check all agents reached goal
        # -----------------------------
    for i, agent in enumerate(agents):
            final_pos = paths[i][-1]
            fx, fy = final_pos
            gx, gy = agent["goal"]
            assert abs(fx - gx) + abs(fy - gy) <= 1, \
                f"Agent {i} did not reach goal"


        # -----------------------------
        # 2. Check no vertex collisions
        # -----------------------------
    max_len = max(len(p) for p in paths)

    for t in range(max_len):
            occupied = {}

            for i, path in enumerate(paths):
                if t >= len(path):
                    continue

                pos = path[t]

                
                assert pos not in occupied, \
                   f"Vertex collision at time {t} between agents {i} and {occupied.get(pos)}"

                occupied[pos] = i

        # -----------------------------
        # 3. Check no edge swaps
        # -----------------------------
    for t in range(max_len - 1):
            transitions = {}

            for i, path in enumerate(paths):
                if t + 1 >= len(path):
                    continue

                move = (path[t], path[t + 1])

                # Check reverse move exists
                reverse = (move[1], move[0])
                assert reverse not in  transitions, f"Edge swap at time {t}: agent {i}" 
                transitions[move] = i

def test_grid_division_v4():
    
    planner = ORToolsPlannerV4(
        grid_size=(20,20),
        obstacles=[(10, 10), (10, 11), (10, 12)]
    )
    
    # -----------------------------
    # Agents
    # -----------------------------
    agents = [

    {"start": (1, 1), "goal": (15, 15)},
    {"start": (3, 1), "goal": (15, 14)},
    {"start": (1, 3), "goal": (14, 15)},
    {"start": (3, 3), "goal": (14, 14)},
    {"start": (5, 5), "goal": (18, 18)},

    ]
    
    # -----------------------------
    # Run planner
    # -----------------------------
    paths= planner.plan(
            agents=agents,
            steps=600)
    
        # -----------------------------
        # 1. Check all agents reached goal
        # -----------------------------
    for i, agent in enumerate(agents):
            final_pos = paths[i][-1]
            fx, fy = final_pos
            gx, gy = agent["goal"]
            assert abs(fx - gx) + abs(fy - gy) <= 1, \
                f"Agent {i} did not reach goal"


        # -----------------------------
        # 2. Check no vertex collisions
        # -----------------------------
    max_len = max(len(p) for p in paths)

    for t in range(max_len):
            occupied = {}

            for i, path in enumerate(paths):
                if t >= len(path):
                    continue

                pos = path[t]

                
                assert pos not in occupied, \
                   f"Vertex collision at time {t} between agents {i} and {occupied.get(pos)}"

                occupied[pos] = i

        # -----------------------------
        # 3. Check no edge swaps
        # -----------------------------
    for t in range(max_len - 1):
            transitions = {}

            for i, path in enumerate(paths):
                if t + 1 >= len(path):
                    continue

                move = (path[t], path[t + 1])

                # Check reverse move exists
                reverse = (move[1], move[0])
                assert reverse not in  transitions, f"Edge swap at time {t}: agent {i}" 
                transitions[move] = i

def test_grid_division_v6():
    
    planner = HierarchicalPlanner(
        grid_size=(20,20),
        obstacles=[(10, 10), (10, 11), (10, 12)]
    )
    
    # -----------------------------
    # Agents
    # -----------------------------
    agents = [

    {"start": (1, 1), "goal": (15, 15)},
    {"start": (3, 1), "goal": (15, 14)},
    {"start": (1, 3), "goal": (14, 15)},
    {"start": (3, 3), "goal": (14, 14)},
    {"start": (5, 5), "goal": (18, 18)},

    ]
    
    # -----------------------------
    # Run planner
    # -----------------------------
    paths= planner.plan(
            agents=agents,
            steps=600)
    print(f"calculated paths:{paths}")
        # -----------------------------
        # 1. Check all agents reached goal
        # -----------------------------
    for i, agent in enumerate(agents):
            final_pos = paths[i][-1]
            fx, fy = final_pos
            gx, gy = agent["goal"]
            assert abs(fx - gx) + abs(fy - gy) <= 1, \
                f"Agent {i} did not reach goal"


        # -----------------------------
        # 2. Check no vertex collisions
        # -----------------------------
    max_len = max(len(p) for p in paths)

    for t in range(max_len):
            occupied = {}

            for i, path in enumerate(paths):
                if t >= len(path):
                    continue

                pos = path[t]

                
                assert pos not in occupied, \
                   f"Vertex collision at time {t} between agents {i} and {occupied.get(pos)}"

                occupied[pos] = i

        # -----------------------------
        # 3. Check no edge swaps
        # -----------------------------
    for t in range(max_len - 1):
            transitions = {}

            for i, path in enumerate(paths):
                if t + 1 >= len(path):
                    continue

                move = (path[t], path[t + 1])

                # Check reverse move exists
                reverse = (move[1], move[0])
                assert reverse not in  transitions, f"Edge swap at time {t}: agent {i}" 
                transitions[move] = i


def test_grid_division_v7():
    
    planner = LaCAMPlanner(
        grid_size=(20,20),
        obstacles=[(10, 10), (10, 11), (10, 12)]
    )
    
    # -----------------------------
    # Agents
    # -----------------------------
    agents = [

    {"start": (1, 1), "goal": (15, 15)},
    {"start": (3, 1), "goal": (15, 14)},
    {"start": (1, 3), "goal": (14, 15)},
    {"start": (3, 3), "goal": (14, 14)},
    {"start": (5, 5), "goal": (18, 18)},

    ]
    
    # -----------------------------
    # Run planner
    # -----------------------------
    success,paths= planner.plan(
            agents=agents,max_time=500)
    print(f"success {success} calculated paths:{paths}")
        # -----------------------------
        # 1. Check all agents reached goal
        # -----------------------------
    for i, agent in enumerate(agents):
            final_pos = paths[i][-1]
            fx, fy = final_pos
            gx, gy = agent["goal"]
            assert abs(fx - gx) + abs(fy - gy) <= 1, \
                f"Agent {i} did not reach goal"


        # -----------------------------
        # 2. Check no vertex collisions
        # -----------------------------
    max_len = max(len(p) for p in paths)

    for t in range(max_len):
            occupied = {}

            for i, path in enumerate(paths):
                if t >= len(path):
                    continue

                pos = path[t]

                
                assert pos not in occupied, \
                   f"Vertex collision at time {t} between agents {i} and {occupied.get(pos)}"

                occupied[pos] = i

        # -----------------------------
        # 3. Check no edge swaps
        # -----------------------------
    for t in range(max_len - 1):
            transitions = {}

            for i, path in enumerate(paths):
                if t + 1 >= len(path):
                    continue

                move = (path[t], path[t + 1])

                # Check reverse move exists
                reverse = (move[1], move[0])
                assert reverse not in  transitions, f"Edge swap at time {t}: agent {i}" 
                transitions[move] = i