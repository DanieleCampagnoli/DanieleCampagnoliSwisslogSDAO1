from swarmmanager.bo import (
    SwarmManagerCoordinate,
    SwarmManagerObstacle,
    SwarmManagerRobot,
    SwarmManagerWarehouse,
    SwarmManagerRobotMovementLog
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


    @staticmethod
    def _move_to_char(cx, cy, nx, ny):
        dx = nx - cx
        dy = ny - cy

        if dx == 1 and dy == 0:
            return "R"
        elif dx == -1 and dy == 0:
            return "L"
        elif dx == 0 and dy == 1:
            return "D"
        elif dx == 0 and dy == -1:
            return "U"
        elif dx == 0 and dy == 0:
            return "W"
        else:
            raise ValueError(f"Invalid move from ({cx},{cy}) to ({nx},{ny})")

    @staticmethod
    def to_file(session, warehouse: SwarmManagerWarehouse, output_path: str):
        """
        Extract movements from DB and write .out file
        """

        # ✅ 1. get all movements ordered by time
        logs = (
            session.query(SwarmManagerRobotMovementLog)
            .order_by(
                SwarmManagerRobotMovementLog.iterationNumber,
                SwarmManagerRobotMovementLog.robotId
            )
            .all()
        )

        # ✅ 2. group by robot
        robot_moves = {robot.robotId: [] for robot in warehouse.robots}

        for log in logs:
            move = DatasetConverter._move_to_char(
                log.current_x,
                log.current_y,
                log.next_x,
                log.next_y
            )
            robot_moves[log.robotId].append(move)

        # ✅ 3. normalize lengths (pad with W)
        max_len = max(len(moves) for moves in robot_moves.values())

        for rid in robot_moves:
            while len(robot_moves[rid]) < max_len:
                robot_moves[rid].append("W")

        # ✅ 4. write file (robot order MUST match input order)
        ordered_ids = [robot.robotId for robot in warehouse.robots]

        with open(output_path, "w") as f:
            for rid in ordered_ids:
                line = " ".join(robot_moves[rid])
                f.write(line + "\n")

