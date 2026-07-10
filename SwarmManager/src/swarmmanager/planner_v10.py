import os
import time
import subprocess


class LaCAM3Planner:

    def __init__(self, grid_size, obstacles):

        self.W, self.H = grid_size

        self.grid = [["."] * self.W for _ in range(self.H)]

        for x, y in obstacles:
            self.grid[y][x] = "@"

        base_dir = os.path.dirname(os.path.abspath(__file__))
        root = os.path.abspath(
            os.path.join(base_dir, "..", "..")
        )

        lacam_dir = os.path.join(root, "lacam3")

        self.exe_path = None

        candidates = [
            os.path.join(lacam_dir, "build", "lacam3.exe"),
            os.path.join(lacam_dir, "lacam3.exe"),
        ]

        for p in candidates:
            if os.path.exists(p):
                self.exe_path = p
                break

        if self.exe_path is None:
            raise FileNotFoundError(
                "LaCAM3 executable not found"
            )

        self.root_dir = os.path.dirname(
            self.exe_path
        )

    # --------------------------------------------------

    def plan(self, agents):

        pairs = [
            (
                tuple(a["start"]),
                tuple(a["goal"]),
            )
            for a in agents
        ]

        return self._run_once(pairs)

    # --------------------------------------------------

    def _run_once(self, agents):

        tmpdir = os.path.join(
            self.root_dir,
            "lacam_debug",
            str(time.time_ns())
        )

        os.makedirs(
            tmpdir,
            exist_ok=True
        )

        map_file = os.path.join(
            tmpdir,
            "map.map"
        )

        scen_file = os.path.join(
            tmpdir,
            "instance.scen"
        )
        #
        # LaCAM3 writes result.txt
        # in the temp directory
        #
        result_file = os.path.join(
            tmpdir,
            "result.txt"
        )

        self._write_map(map_file)

        self._write_scenario(
            scen_file,
            map_file,
            agents,
        )

        cmd = [
            self.exe_path,
            "-m",
            map_file,
            "-i",
            scen_file,
            "-N",
            str(len(agents)),
            "-o",
            result_file,
            "-t", 
            "300", 
            "-s", 
            "0", 
            "-v", 
            "3",   
            "--refiner-num",
            "16", 
            "--recursive-rate", 
            "0.4", 
            "--recursive-time-limit", 
            "2", 
            "--scatter-margin", 
            "0"
        ]


        result = subprocess.run(
            cmd,
            cwd=self.root_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        if result.returncode != 0:

            print("LaCAM3 failed")
            print(result.stdout)
            print(result.stderr)

            return False, None



        paths = self._parse_output(
            result_file
        )

        if paths is None:
            return False, None

        return True, paths

    # --------------------------------------------------

    def _write_map(self, path):

        with open(path, "w") as f:

            f.write("type octile\n")
            f.write(
                f"height {self.H}\n"
            )
            f.write(
                f"width {self.W}\n"
            )
            f.write("map\n")

            for row in self.grid:
                f.write(
                    "".join(row) + "\n"
                )

    # --------------------------------------------------

    def _write_scenario(
        self,
        path,
        map_file,
        agents,
    ):

        map_name = os.path.basename(
            map_file
        )

        with open(path, "w") as f:

            f.write("version 1\n")

            for (
                (sx, sy),
                (gx, gy)
            ) in agents:

                #
                # MovingAI scenario format
                #
                f.write(
                    "0\t"
                    f"{map_name}\t"
                    f"{self.W}\t"
                    f"{self.H}\t"
                    f"{sx}\t"
                    f"{sy}\t"
                    f"{gx}\t"
                    f"{gy}\t"
                    "0\n"
                )

    # --------------------------------------------------

    def _parse_output(self, path):

        if not os.path.exists(path):
            return None

        with open(path, "r") as f:
            text = f.read()

        if "solved=1" not in text:
            return None

        timesteps = []

        recording = False

        for line in text.splitlines():

            line = line.strip()

            if line.startswith(
                "solution="
            ):
                recording = True
                continue

            if not recording:
                continue

            if ":" not in line:
                continue

            try:
                _,state = line.split(
                    ":",
                    1
                )
            except ValueError:
                continue

            positions = []

            for token in state.split(
                "),"
            ):

                token = (
                    token
                    .replace(
                        "(",
                        ""
                    )
                    .replace(
                        ")",
                        ""
                    )
                    .strip()
                )

                if not token:
                    continue

                try:

                    x, y = map(
                        int,
                        token.split(",")
                    )

                    positions.append(
                        (x, y)
                    )

                except Exception:
                    pass

            if positions:
                timesteps.append(
                    positions
                )

        if not timesteps:
            return None

        agent_count = len(
            timesteps[0]
        )

        paths = [
            []
            for _ in range(
                agent_count
            )
        ]

        for step in timesteps:

            for i in range(
                agent_count
            ):
                paths[i].append(
                    step[i]
                )

        return paths