from swarmmanager.dataset_converter import DatasetConverter
from swarmmanager.constants import DATASET_A_IN_FILE
from swarmmanager.bo import init_db
from swarmmanager.bo import clean_db
from swarmmanager.bo import SwarmManagerWarehouse
from swarmmanager.bo import SwarmManagerRobotMovementLog
from swarmmanager.visualization import SwarmManagerVisualization




def main():
    clean_db("db/swarm.db")
    warehouse = DatasetConverter.from_file(DATASET_A_IN_FILE)
    Session = init_db("sqlite:///db/swarm.db")
    session = Session()
    # Save everything
    session.add(warehouse)
    session.commit() 
    # save a log for every robot with initial position
    for  robot in warehouse.robots:
         log = SwarmManagerRobotMovementLog(1,robot,robot.startingPoint)
         session.add(log)
         session.commit() 
    #inline lambda function to query the db
    mlQuery= lambda iteration_number:session.query(SwarmManagerRobotMovementLog).filter(SwarmManagerRobotMovementLog.iterationNumber == iteration_number).all()
    # create and start the visu
    visu=SwarmManagerVisualization(100,warehouse.maxX,warehouse.maxY,mlQuery)
    visu.start_visu(warehouse)

if __name__ == "__main__":
    main()