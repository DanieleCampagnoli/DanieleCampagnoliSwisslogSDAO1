# tests/test_converter.py

from swarmmanager.dataset_converter import DatasetConverter
from swarmmanager.constants import DATASET_A_IN_FILE

def test_dataset_loading():
    warehouse = DatasetConverter.from_file(DATASET_A_IN_FILE)

    assert warehouse.maxX == 10
    assert warehouse.maxY == 10

    assert len(warehouse.obstacles) == 5
    assert len(warehouse.robots) == 3

    # check first robot
    r = warehouse.robots[0]
    assert (r.startingPoint.x, r.startingPoint.y) == (4, 4)
    assert (r.destinationPoint.x, r.destinationPoint.y) == (8, 8)