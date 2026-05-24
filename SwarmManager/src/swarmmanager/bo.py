""" coordinate of the simualtion 2D (x and y) """
class SwarmManagerCoordinate:
    def __init__(self,x,y):
        self.x=x
        self.y=y
    
    def __repr__(self):
        return f"({self.x},{self.y})"
    
""" the class represents an obstacle in the way """
class SwarmManagerObstacle:
    def __init__(self,obsId:int,coordinate: SwarmManagerCoordinate ):
        self.obsId=obsId
        self.coordinate=coordinate
    def __repr__(self):
        return f"SwarmManagerObstacle(id={self.obsId}, coordinate={self.coordinate})"
                
"""the class represents a robot """
class SwarmManagerRobot:
    def __init__(self,robotId:int, startingPoint: SwarmManagerCoordinate, destinationPoint: SwarmManagerCoordinate, currentPosition: SwarmManagerCoordinate):
        self.robotId=robotId
        self.startingPoint=startingPoint
        self.destinationPoint=destinationPoint
        self.currentPosition=currentPosition
    def __repr__(self):
        return f"SwarmManagerRobot(robotId={self.robotId}, startingPoint={self.startingPoint}, destinationPoint={self.destinationPoint}, currentPosition={self.currentPosition})"
        
""" the class holding general data of the warehous (grid-dimensions and so on) """
class SwarmManagerWharehouse:
    def __init__(self,maxX,maxY, obstacles:list , robots:list):
        self.maxX=maxX
        self.maxY=maxY
        self.obstacles=obstacles
        self.robots=robots        
    def __repr__(self):
        return f"SwarmManagerWharehouse(maxX={self.maxX}, maxY={self.maxY}, obstacles={self.obstacles},robots={self.robots})"    
    
""" calculation state """
class SwarmManagerCalculationState:
    def __init__(self,currentIterationNumber:int,maxIterationNumber:int):
         self.currentIterationNumber=currentIterationNumber
         self.maxIterationNumber=maxIterationNumber
    def __repr__(self):  
        return f"SwarmManagerCalculationState(currentIterationNumber={self.currentIterationNumber},maxIterationNumber={self.maxIterationNumber})"

"""past movements of the robot"""
class SwarmManagerRobotMovementLog:
    def __init__(self,iterationNunber:int,robot:SwarmManagerRobot,nextPosition: SwarmManagerCoordinate):
        self.robotId=robot.robotId
        self.startingPoint=robot.startingPoint
        self.destinationPoint=robot.destinationPoint
        self.currentPosition=robot.currentPosition
        self.nextPosition=nextPosition
        self.iterationNunber=iterationNunber
    def __repr__(self):
        return f"SwarmManagerRobotMovementLog(iterationNunber={self.iterationNunber}, robotId={self.robotId}, currentPosition={self.currentPosition}, nextPosition={self.nextPosition})"
