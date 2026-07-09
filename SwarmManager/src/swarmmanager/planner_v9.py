import subprocess
import tempfile
import os
import time


class PIBTPlanner:

    def __init__(self, grid_size, obstacles):

        self.W, self.H = grid_size

        self.grid = [["."] * self.W for _ in range(self.H)]

        for x, y in obstacles:
            self.grid[y][x] = "@"

        # ✅ auto resolve executable
        base_dir = os.path.dirname(os.path.abspath(__file__))
        root = os.path.abspath(os.path.join(base_dir, "..", ".."))
        lacam_dir = os.path.join(root, "pibt2")
        self.lacam_dir=lacam_dir 
        for name in ["mapf.exe"]:
                path = os.path.join(lacam_dir, name)
                if os.path.exists(path):
                    self.exe_path = path
                    break
        if self.exe_path is None:
                raise FileNotFoundError("pibt2 executable not found")

        self.root_dir = os.path.dirname(self.exe_path)

    # -------------------------------------------------

    def plan(self, agents):

        pairs = [
            (tuple(a["start"]), tuple(a["goal"]))
            for a in agents
        ]

        return self._run_once(pairs)

    # -------------------------------------------------

    def _run_once(self, agents):

        tmpdir = os.path.join(
            self.root_dir,
            "pibt_debug",
            str(time.time_ns())
        )

        os.makedirs(tmpdir, exist_ok=True)

        map_file = os.path.join(tmpdir, "map.map")
        instance_file = os.path.join(tmpdir, "instance.txt")
        result_file = os.path.join(tmpdir, "result.txt")

        self._write_map(map_file)
        self._write_instance(
            instance_file,
            map_file,
            agents
        )

        cmd = [
            self.exe_path,
            "-i",
            instance_file,
            "-s",
            "PIBT",
            "-o",
            result_file,
            "-v",
        ]

        assert os.path.exists(map_file), map_file
        assert os.path.exists(instance_file), instance_file

        print(open(instance_file).read().splitlines()[0])

        result = subprocess.run(
            cmd,
            cwd=self.root_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        if result.returncode != 0:
            print("PIBT failed:")
            print(result.stdout)
            print(result.stderr)
            return False, None

        paths = self._parse_output(result_file)

        if paths is None:
            return False, None

        return True, paths

    # -------------------------------------------------

    def _write_map(self, path):

        with open(path, "w") as f:

            f.write("type octile\n")
            f.write(f"height {self.H}\n")
            f.write(f"width {self.W}\n")
            f.write("map\n")

            for row in self.grid:
                f.write("".join(row) + "\n")

    # -------------------------------------------------

    def _write_instance(
        self,
        path,
        map_file,
        agents,
    ):

        map_file = os.path.abspath(map_file)
        map_file = map_file.replace("\\", "/")

        with open(path, "w") as f:

            f.write("map_file=" + map_file.replace("\\", "/") + "\n\n")

            f.write(f"agents={len(agents)}\n\n")

            f.write("seed=1\n\n")

            f.write("random_problem=0\n\n")

            f.write("max_timestep=5000\n\n")

            f.write("max_comp_time=120000\n\n")

            for (sx, sy), (gx, gy) in agents:
                f.write(
                    f"{sx},{sy},{gx},{gy}\n"
                )

    # -------------------------------------------------

    def _parse_output(self, path):

        if not os.path.exists(path):
            return None

        with open(path, "r") as f:
            text = f.read()
        #at the moment it's not able to solve the problem completely 
        if "solved=1" not in text:
            return None

        lines = text.splitlines()

        solution_lines = []

        recording = False

        for line in lines:

            line = line.strip()

            if line.startswith("solution="):
                recording = True

            if recording:
                solution_lines.append(line)

        if not solution_lines:
            return None

        timesteps = []

        #
        # Example PIBT output:
        #
        # 0: (1,1), (2,2), ...
        # 1: (1,2), (2,3), ...
        #

        for line in solution_lines:

            if ":" not in line:
                continue

            if line.startswith("solution="):
                line = line[len("solution="):]

            idx = line.find(":")

            if idx < 0:
                continue

            state = line[idx + 1:]

            positions = []

            for token in state.split("),"):

                token = (
                    token
                    .replace("(", "")
                    .replace(")", "")
                    .strip()
                )

                if not token:
                    continue

                try:
                    x, y = map(int, token.split(","))
                    positions.append((x, y))
                except:
                    pass

            if positions:
                timesteps.append(positions)

        if not timesteps:
            return None

        agent_count = len(timesteps[0])

        paths = [[] for _ in range(agent_count)]

        for step in timesteps:
            for i in range(agent_count):
                paths[i].append(step[i])

        return paths