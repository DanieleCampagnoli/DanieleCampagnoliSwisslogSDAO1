import subprocess
import tempfile
import os
import time

class LaCAMPlanner:

    def __init__(self, grid_size, obstacles, exe_path=None):

        self.W, self.H = grid_size

        # ✅ build grid ('.' free, '@' obstacle)
        self.grid = [["."] * self.W for _ in range(self.H)]

        for (x, y) in obstacles:
            self.grid[y][x] = "@"

        # ✅ auto resolve executable
        if exe_path is None:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            root = os.path.abspath(os.path.join(base_dir, "..", ".."))
            lacam_dir = os.path.join(root, "lacam")
            self.lacam_dir=lacam_dir 
            for name in ["lacam.exe", "main.exe"]:
                path = os.path.join(lacam_dir, name)
                if os.path.exists(path):
                    self.exe_path = path
                    break
            else:
                raise FileNotFoundError("LaCAM executable not found")
        else:
            self.exe_path = exe_path

    # -------------------------------------------------

    def plan(self, agents, max_time=None):
        """
        agents: [{"start": (x,y), "goal": (x,y)}, ...]
        returns: (success, paths)
        """

        # ✅ convert to internal format
        agent_pairs = [
            (tuple(a["start"]), tuple(a["goal"]))
            for a in agents
        ]

        grid = self._grid_to_strings()

        # ✅ single-shot solve
        solution = self._run_once(grid, agent_pairs)

        if not solution:
            return False, None

        return True, solution

    # -------------------------------------------------

    def _run_once(self, grid, agents):

        tmpdir=f"{self.lacam_dir}\lacam_debug\{time.time_ns()}"
        os.mkdir(tmpdir)
        map_file = os.path.join(tmpdir, "map.map")
        scen_file = os.path.join(tmpdir, "scen.scen")
        output_file = os.path.join(tmpdir, "out.txt")

        self._write_map(map_file, grid)
        self._write_scen(scen_file, agents, os.path.basename(map_file))

        cmd = [
                self.exe_path,
                "-m", map_file,
                "-i", scen_file,
                "-N", str(len(agents)),
                "-o", output_file,
                "-t", "600"
            ]

        # ✅ run solver (safe)
        result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            # ✅ handle failure gracefully
        if result.returncode != 0:
                print("LaCAM failed:")
                print(result.stdout)
                print(result.stderr)
                return None

        return self._parse_output(output_file)

    # -------------------------------------------------

    def _grid_to_strings(self):
        return ["".join(row) for row in self.grid]

    # -------------------------------------------------

    def _write_map(self, path, grid):
        height = len(grid)
        width = len(grid[0])

        with open(path, "w") as f:
            f.write("type octile\n")
            f.write(f"height {height}\n")
            f.write(f"width {width}\n")
            f.write("map\n")
            for row in grid:
                f.write(row + "\n")

    # -------------------------------------------------

    def _write_scen(self, path, agents, map_name):
        with open(path, "w") as f:
            f.write("version 1\n")

            for i, ((sx, sy), (gx, gy)) in enumerate(agents):
                f.write(f"{i}\t{map_name}\t0\t0\t{sx}\t{sy}\t{gx}\t{gy}\t1\n")

    # -------------------------------------------------

    def _parse_output(self, path):

        if not os.path.exists(path):
            return None

        with open(path, "r") as f:
            lines = f.readlines()

        timesteps = []

        solution_start = False

        for line in lines:
            if line.startswith("solution="):
                solution_start = True
                continue

            if solution_start:
                if not line.strip():
                    break

                # line example:
                # 0:(0,0),(4,4),

                parts = line.split(":")[1].strip().split("),")

                step_positions = []

                for p in parts:
                    p = p.replace("(", "").replace(")", "").strip()
                    if not p:
                        continue

                    x, y = map(int, p.split(","))
                    step_positions.append((x, y))

                timesteps.append(step_positions)
        
        # ✅ SAFETY CHECK (this fixes your crash)
        if not timesteps:
            return None
  
        # ✅ TRANSPOSE (this is the fix)
        num_agents = len(timesteps[0])
        paths = [[] for _ in range(num_agents)]

        for step in timesteps:
            for i in range(num_agents):
                paths[i].append(step[i])

        return paths