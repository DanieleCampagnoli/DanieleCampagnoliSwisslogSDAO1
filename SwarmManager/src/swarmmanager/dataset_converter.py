from swarmmanager.bo import (
    SwarmManagerCoordinate,
    SwarmManagerObstacle,
    SwarmManagerRobot,
    SwarmManagerWarehouse
)


class DatasetConverter:

    @staticmethod
    def from_file(file_path: str) -> SwarmManagerWarehouse:
        """
        Parse dataset_A.in file and return a fully populated SwarmManagerWarehouse object
        """

        # --- read and clean lines (ignore empty + comments) ---
        with open(file_path, "r") as f:
            lines = [
                line.strip()
                for line in f
                if line.strip() and not line.strip().startswith("#")
            ]

        idx = 0

        # --- 1. Warehouse dimensions ---
        maxX, maxY = map(int, lines[idx].split())
        idx += 1

        # --- 2. Obstacles ---
        obstacle_count = int(lines[idx])
        idx += 1

        obstacles = []
        for _ in range(obstacle_count):
            x, y = map(int, lines[idx].split())

            coord = SwarmManagerCoordinate(x, y)
            obstacle = SwarmManagerObstacle(idx,coord)

            obstacles.append(obstacle)
            idx += 1

        # --- 3. Robots ---
        robot_count = int(lines[idx])
        idx += 1

        robots = []
        for _ in range(robot_count):
            x_start, y_start, x_dest, y_dest = map(int, lines[idx].split())

            start = SwarmManagerCoordinate(x_start, y_start)
            dest = SwarmManagerCoordinate(x_dest, y_dest)

            # ✅ initial position = start
            robot = SwarmManagerRobot(
                robotId=idx,
                startingPoint=start,
                destinationPoint=dest,
                currentPosition=start
            )

            robots.append(robot)
            idx += 1

        # --- 4. Build final warehouse object ---
        warehouse = SwarmManagerWarehouse(
            maxX=maxX,
            maxY=maxY,
            obstacles=obstacles,
            robots=robots
        )
        
        return warehouse
