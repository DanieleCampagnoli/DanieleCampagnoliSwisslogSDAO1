""" coordinate of the simualtion 2D (x and y) """
class SwarmManagerCoordinate:
    def __init__(self,x,y):
        self.x=x
        self.y=y
    
    def __repr__(self):
        return f"({self.x},{self.y})"
    
""" the class represents an obstacle in the way """
class SwarmManagerObstacle:
    def __init__(self,coordinate: SwarmManagerCoordinate ):
        self.coordinate=coordinate
    def __repr__(self):
        return f"SwarmManagerObstacle({self.coordinate})"
                
"""the class represents a robot """
class SwarmManagerRobot:
    def __init__(self, startingPoint: SwarmManagerCoordinate, destinationPoint: SwarmManagerCoordinate, currentPosition: SwarmManagerCoordinate):
        self.startingPoint=startingPoint
        self.destinationPoint=destinationPoint
        self.currentPosition=currentPosition
    def __repr__(self):
        return f"SwarmManagerRobot(startingPoint={self.startingPoint}, destinationPoint={self.destinationPoint}, currentPosition={self.currentPosition})"
        
""" the class holding general data of the warehous (grid-dimensions and so on) """
class SwarmManagerWharehouse:
    def __init__(self,maxX,maxY, obstacles:list , robots:list):
        self.maxX=maxX
        self.maxY=maxY
        self.obstacles=obstacles
        self.robots=robots        
    def __repr__(self):
        return f"SwarmManagerWharehouse(maxX={self.maxX}, maxY={self.maxY}, obstacles={self.obstacles},robots={self.robots})"    