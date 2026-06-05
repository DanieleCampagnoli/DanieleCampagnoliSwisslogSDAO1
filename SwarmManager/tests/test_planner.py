
from swarmmanager.planner import ORToolsPlanner

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
