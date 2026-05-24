from swarmmanager.bo import SwarmManagerWarehouse

def basic_assertion_dataset_a(warehouse: SwarmManagerWarehouse):
    assert warehouse.maxX == 10
    assert warehouse.maxY == 10

    assert len(warehouse.obstacles) == 5
    assert len(warehouse.robots) == 3

    # check first robot
    r = warehouse.robots[0]
    assert (r.startingPoint.x, r.startingPoint.y) == (4, 4)
    assert (r.destinationPoint.x, r.destinationPoint.y) == (8, 8)