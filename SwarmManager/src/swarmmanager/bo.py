from sqlalchemy import Column, Integer, Float, ForeignKey, create_engine
from sqlalchemy.orm import declarative_base, relationship, sessionmaker
import os
# ---------------------------
# re-use business objects for the ORM framework.
# "Active Record / ORM Model reuse"
# ---------------------------

Base = declarative_base()

# ---------------------------
# Coordinate
# ---------------------------
class SwarmManagerCoordinate(Base):
    __tablename__ = "coordinates"

    id = Column(Integer, primary_key=True)
    x = Column(Float)
    y = Column(Float)

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"({self.x},{self.y})"


# ---------------------------
# Obstacle
# ---------------------------
class SwarmManagerObstacle(Base):
    __tablename__ = "obstacles"

    id = Column(Integer, primary_key=True)
    obsId = Column(Integer)

    coordinate_id = Column(Integer, ForeignKey("coordinates.id"))
    coordinate = relationship("SwarmManagerCoordinate")

    warehouse_id = Column(Integer, ForeignKey("warehouse.id"))
    warehouse = relationship("SwarmManagerWarehouse", back_populates="obstacles")

    def __init__(self, obsId, coordinate):
        self.obsId = obsId
        self.coordinate = coordinate

    def __repr__(self):
        return f"Obstacle(id={self.obsId}, coordinate={self.coordinate})"


# ---------------------------
# Robot
# ---------------------------
class SwarmManagerRobot(Base):
    __tablename__ = "robots"

    robotId = Column(Integer, primary_key=True)

    start_id = Column(Integer, ForeignKey("coordinates.id"))
    dest_id = Column(Integer, ForeignKey("coordinates.id"))
    current_id = Column(Integer, ForeignKey("coordinates.id"))

    startingPoint = relationship("SwarmManagerCoordinate", foreign_keys=[start_id])
    destinationPoint = relationship("SwarmManagerCoordinate", foreign_keys=[dest_id])
    currentPosition = relationship("SwarmManagerCoordinate", foreign_keys=[current_id])
    
    warehouse_id = Column(Integer, ForeignKey("warehouse.id"))
    warehouse = relationship("SwarmManagerWarehouse", back_populates="robots")

    def __init__(self, robotId, startingPoint, destinationPoint, currentPosition):
        self.robotId = robotId
        self.startingPoint = startingPoint
        self.destinationPoint = destinationPoint
        self.currentPosition = currentPosition

    def __repr__(self):
        return (
            f"Robot(robotId={self.robotId}, "
            f"start={self.startingPoint}, "
            f"dest={self.destinationPoint}, "
            f"current={self.currentPosition})"
        )


# ---------------------------
# Warehouse and simulation state
# ---------------------------
class SwarmManagerWarehouse(Base):
    __tablename__ = "warehouse"

    id = Column(Integer, primary_key=True)
    maxX = Column(Integer)
    maxY = Column(Integer)
    currentIterationNumber = Column(Integer)
    maxIterationNumber = Column(Integer)

    # Relationships
    obstacles = relationship("SwarmManagerObstacle", back_populates="warehouse")
    robots = relationship("SwarmManagerRobot", back_populates="warehouse")

    def __init__(self, maxX, maxY,obstacles=None, robots=None):
        self.maxX = maxX
        self.maxY = maxY
        self.obstacles = obstacles or []
        self.robots = robots or []


    def __repr__(self):
        return f"Warehouse(maxX={self.maxX}, maxY={self.maxY})"



# ---------------------------
# Robot Movement Log
# ---------------------------
class SwarmManagerRobotMovementLog(Base):
    __tablename__ = "movement_log"

    id = Column(Integer, primary_key=True)
    iterationNumber = Column(Integer)
    robotId = Column(Integer)

    current_x = Column(Float)
    current_y = Column(Float)
    next_x = Column(Float)
    next_y = Column(Float)

    def __init__(self, iterationNumber, robot, nextPosition):
        self.iterationNumber = iterationNumber
        self.robotId = robot.robotId

        self.current_x = robot.currentPosition.x
        self.current_y = robot.currentPosition.y

        self.next_x = nextPosition.x
        self.next_y = nextPosition.y

    def __repr__(self):
        return (
            f"Movement(iter={self.iterationNumber}, "
            f"robotId={self.robotId}, "
            f"current=({self.current_x},{self.current_y}), "
            f"next=({self.next_x},{self.next_y}))"
        )


# ---------------------------
# DB INITIALIZATION
# ---------------------------
def init_db(db_url="sqlite:///db/swarm.db"):
    engine = create_engine(db_url, echo=False)
    Base.metadata.create_all(engine)
    return sessionmaker(bind=engine)

def clean_db( db_path = "db/swarm.db"):
    if os.path.exists(db_path):
        os.remove(db_path)


# ---------------------------
# EXAMPLE USAGE
# ---------------------------
if __name__ == "__main__":
    Session = init_db()
    session = Session()

    # Create coordinates
    c1 = SwarmManagerCoordinate(1, 1)
    c2 = SwarmManagerCoordinate(5, 5)

    # Create robot
    robot = SwarmManagerRobot(
        robotId=1,
        startingPoint=c1,
        destinationPoint=c2,
        currentPosition=c1
    )

    # Movement example
    next_pos = SwarmManagerCoordinate(2, 1)
    movement = SwarmManagerRobotMovementLog(1, robot, next_pos)

    # Save everything
    session.add(robot)
    session.add(movement)
    session.commit()

    print("✅ Data stored in swarm.db")