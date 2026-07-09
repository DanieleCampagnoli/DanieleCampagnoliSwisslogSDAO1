from swarmmanager.dataset_converter import DatasetConverter
from swarmmanager.constants import DATASET_A_IN_FILE
from swarmmanager.constants import DATASET_B_IN_FILE
from swarmmanager.constants import DATASET_C_IN_FILE
from swarmmanager.bo import init_db
from swarmmanager.bo import clean_db
from swarmmanager.bo import SwarmManagerWarehouse
from swarmmanager.bo import SwarmManagerRobotMovementLog
from swarmmanager.bo import SwarmManagerCoordinate
from swarmmanager.visualization import SwarmManagerVisualization
from swarmmanager.planner import ORToolsPlanner
from swarmmanager.planner_v4 import ORToolsPlannerV4

def main():
    Session = init_db("sqlite:///db/swarm.db")
    session = Session()
    warehouse = session.query(SwarmManagerWarehouse)\
    .filter(SwarmManagerWarehouse.id == 1)\
    .first()
    DatasetConverter.to_file(session, warehouse, "dataset_B.out")

if __name__ == "__main__":
    main()