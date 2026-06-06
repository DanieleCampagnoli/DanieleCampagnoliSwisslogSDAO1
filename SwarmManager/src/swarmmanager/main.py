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



def main():
    clean_db("db/swarm.db")
    warehouse = DatasetConverter.from_file(DATASET_B_IN_FILE)
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
    # plan the movements
    run_planner(warehouse,session)      
    #inline lambda function to query the db
    mlQuery= lambda iteration_number:session.query(SwarmManagerRobotMovementLog).filter(SwarmManagerRobotMovementLog.iterationNumber == iteration_number).all()
    # create and start the visu
    visu=SwarmManagerVisualization(50,warehouse.maxX,warehouse.maxY,mlQuery)
    visu.start_visu(warehouse)

def run_planner(warehouse:SwarmManagerWarehouse,session):
    
    obstacles=[]
    for obs in warehouse.obstacles:
        obstacles.append((int(obs.coordinate.x),int(obs.coordinate.y)))
    
    agents= []
    sorted_list_of_robots = sorted(warehouse.robots, key=lambda x: x.robotId)
    for robot in sorted_list_of_robots:
         agents.append({"start": (int(robot.startingPoint.x),int(robot.startingPoint.y)), "goal": (int(robot.destinationPoint.x),int(robot.destinationPoint.y))})

    planner = ORToolsPlanner(
        grid_size=(warehouse.maxX,warehouse.maxY),
        obstacles=obstacles
    )

    success, paths = planner.plan(agents)
    print(f"planning result is successful? {success}")
    robotInd=0  
    for path in paths:
       robot= sorted_list_of_robots[robotInd]
       movementInd=2
       for movement in path:
         #log the movement
         nextPosition= SwarmManagerCoordinate(movement[0], movement[1])
         log = SwarmManagerRobotMovementLog(movementInd,robot,nextPosition)
         session.add(log)
         session.commit()
         #move the robot 
         robot.currentPosition=nextPosition  
         session.add(robot)  
         session.commit()  
         movementInd+=1 
       robotInd+=1
    print(f"saved  {len(paths)} paths on the movement log")

if __name__ == "__main__":
    main()