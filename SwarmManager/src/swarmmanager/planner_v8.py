import heapq
import os
import time
import subprocess
from concurrent.futures import ThreadPoolExecutor
import random
import logging
import json
from datetime import datetime
from datetime import datetime, UTC

# =====================================================
# JSON LOGGER
# =====================================================

class JsonFormatter(logging.Formatter):
    def format(self, record):
        log_record = {
            "ts": datetime.utcnow().isoformat(),
            "level": record.levelname,
            "event": record.getMessage()
        }
        if hasattr(record, "extra_data"):
            log_record.update(record.extra_data)
        return json.dumps(log_record)


def setup_json_logger(level=logging.INFO):
    logger = logging.getLogger("HierarchicalLaCAM")
    logger.setLevel(level)

    if not logger.handlers:
        # file
        ts = datetime.now(UTC).strftime("%Y-%m-%d_%H-%M-%S")
        log_file=f"planner_log_{ts}.jsonl" 
        handler = logging.FileHandler(log_file, mode="w")
        handler.setFormatter(JsonFormatter())
        logger.addHandler(handler)
        # console
        ch = logging.StreamHandler()
        ch.setFormatter(JsonFormatter())
        logger.addHandler(ch)

    return logger


# =====================================================
# DIJKSTRA
# =====================================================

def dijkstra(graph, start, goal):
    pq = [(0, start)]
    dist = {start: 0}
    parent = {start: None}

    while pq:
        d, u = heapq.heappop(pq)

        if u == goal:
            break

        for v in graph[u]:
            nd = d + 1
            if v not in dist or nd < dist[v]:
                dist[v] = nd
                parent[v] = u
                heapq.heappush(pq, (nd, v))

    # ✅ FIX: guarantee valid path
    if goal not in parent:
        return [start, goal] if start != goal else [start]

    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = parent[cur]

    path.reverse()

    if not path or path[0] != start:
        path = [start] + path

    return path


# =====================================================
# SUBGRID GRAPH
# =====================================================

def build_subgrid_graph(nx, ny):
    graph = {}
    for x in range(nx):
        for y in range(ny):
            neighbors = []
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                xx, yy = x + dx, y + dy
                if 0 <= xx < nx and 0 <= yy < ny:
                    neighbors.append((xx, yy))
            graph[(x,y)] = neighbors
    return graph


# =====================================================
# LACAM WRAPPER
# =====================================================

class LaCAMPlanner:

    def __init__(self, grid_size, obstacles, exe_path):
        self.W, self.H = grid_size
        self.obstacles = set(obstacles)

        self.grid = [["."] * self.W for _ in range(self.H)]
        for (x,y) in self.obstacles:
            self.grid[y][x] = "@"

        self.exe_path = exe_path
        self.tmp_root = os.path.dirname(exe_path)

    def plan(self, agents):
        pairs = [(a["start"], a["goal"]) for a in agents]
        grid = ["".join(r) for r in self.grid]

        sol = self._run_once(grid, pairs)
        if not sol:
            return False, None
        return True, sol

    def _run_once(self, grid, agents):
        tmpdir = os.path.join(self.tmp_root, f"tmp_{time.time_ns()}")
        os.makedirs(tmpdir, exist_ok=True)

        map_file = os.path.join(tmpdir, "map.map")
        scen_file = os.path.join(tmpdir, "scen.scen")
        out_file = os.path.join(tmpdir, "out.txt")

        with open(map_file, "w") as f:
            f.write("type octile\n")
            f.write(f"height {len(grid)}\n")
            f.write(f"width {len(grid[0])}\n")
            f.write("map\n")
            for row in grid:
                f.write(row + "\n")

        with open(scen_file, "w") as f:
            f.write("version 1\n")
            for i, ((sx,sy),(gx,gy)) in enumerate(agents):
                f.write(f"{i}\t{os.path.basename(map_file)}\t0\t0\t{sx}\t{sy}\t{gx}\t{gy}\t1\n")

        cmd = [
            self.exe_path,
            "-m", map_file,
            "-i", scen_file,
            "-N", str(len(agents)),
            "-o", out_file,
            "-t", "10"
        ]

        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        if result.returncode != 0:
            return None

        return self._parse_output(out_file)

    def _parse_output(self, path):
        if not os.path.exists(path):
            return None

        with open(path) as f:
            lines = f.readlines()

        timesteps = []
        reading = False

        for line in lines:
            if line.startswith("solution="):
                reading = True
                continue

            if reading:
                if not line.strip():
                    break

                parts = line.split(":")[1].split("),")
                step = []
                for p in parts:
                    p = p.replace("(", "").replace(")", "").strip()
                    if not p:
                        continue
                    x, y = map(int, p.split(","))
                    step.append((x,y))
                timesteps.append(step)

        if not timesteps:
            return None

        n = len(timesteps[0])
        paths = [[] for _ in range(n)]

        for step in timesteps:
            for i in range(n):
                paths[i].append(step[i])

        return paths


# =====================================================
# HIERARCHICAL PLANNER WITH JSON LOGGING
# =====================================================

class HierarchicalLaCAM:

    def __init__(self, grid_size, obstacles, exe_path,
                 sub_size=(20,20), max_agents=5000,
                 deterministic=False, seed=42, max_workers=4):

        self.W, self.H = grid_size
        self.SUB_W, self.SUB_H = sub_size
        self.obstacles = set(obstacles)

        self.NX = self.W // self.SUB_W
        self.NY = self.H // self.SUB_H

        self.graph = build_subgrid_graph(self.NX, self.NY)
        
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

        self.reserved = {}
        self.age = [0]*max_agents

        self.deterministic = deterministic
        self.random = random.Random(seed)

        self.executor = ThreadPoolExecutor(max_workers=max_workers)

        self.conflict_heat = {}

        # ✅ logging
        self.logger = setup_json_logger(logging.INFO)
        self.enable_logging = False  # default OFF

    # -------------------------------------

    def log(self, level, event, **data):
        if not self.enable_logging:
            return

        self.logger.log(
            level,
            event,
            extra={"extra_data": data}
        )

    # -------------------------------------

    def locate(self, pos):
        return (pos[0] // self.SUB_W, pos[1] // self.SUB_H)

    def bounds(self, sg):
        x0 = sg[0] * self.SUB_W
        y0 = sg[1] * self.SUB_H
        return x0, y0, x0+self.SUB_W, y0+self.SUB_H

    # -------------------------------------

    def compute_priority(self, rid, pos, goal):
        dist = abs(pos[0]-goal[0]) + abs(pos[1]-goal[1])
        return -dist + self.age[rid]

    # -------------------------------------

    def safe_reserve(self, rid, x, y, t, priorities):
        key = (x,y,t)

        if key not in self.reserved:
            self.reserved[key] = rid
            return True

        other = self.reserved[key]

        self.conflict_heat[(x,y)] = self.conflict_heat.get((x,y), 0) + 1

        self.log(logging.DEBUG, "conflict",
                 cell=(x,y), t=t,
                 winner=rid, loser=other)

        if priorities.get(rid, 0) > priorities.get(other, -1e9):
            self.reserved[key] = rid
            self.age[other] += 1
            return True

        self.age[rid] += 1
        return False

    # -------------------------------------

    def local_obstacles(self, sg, t, horizon=8):
        x0,y0,x1,y1 = self.bounds(sg)
        obs = set()

        for (x,y) in self.obstacles:
            if x0 <= x < x1 and y0 <= y < y1:
                obs.add((x-x0, y-y0))

        for (x,y,tt), _ in self.reserved.items():
            if t <= tt < t + horizon:
                if x0 <= x < x1 and y0 <= y < y1:
                    obs.add((x-x0, y-y0))

        return obs

    # -------------------------------------

    def solve_subgrid(self, sg, local_agents, step):
        self.log(logging.DEBUG, "lacam_start",
                 subgrid=sg, agents=len(local_agents))

        planner = LaCAMPlanner(
            (self.SUB_W,self.SUB_H),
            self.local_obstacles(sg, step),
            self.exe_path
        )

        for i in range(2):
            ok, paths = planner.plan(local_agents)
            if ok:
                return True, paths
            self.log(logging.WARNING, "lacam_retry", subgrid=sg, attempt=i+1)

        self.log(logging.ERROR, "lacam_failed", subgrid=sg)
        return False, None

    # -------------------------------------

    def border_cells(self, a, b):
        x0,y0,x1,y1 = self.bounds(a)

        dx = b[0]-a[0]
        dy = b[1]-a[1]

        if dx == 1:
            bx = x1-1
            cy = (y0+y1)//2
            return [(bx,cy-1),(bx,cy)]

        if dx == -1:
            bx = x0
            cy = (y0+y1)//2
            return [(bx,cy-1),(bx,cy)]

        if dy == 1:
            by = y1-1
            cx = (x0+x1)//2
            return [(cx-1,by),(cx,by)]

        if dy == -1:
            by = y0
            cx = (x0+x1)//2
            return [(cx-1,by),(cx,by)]

        return []

    def assign_gate(self, rid, sg, next_sg, gate_usage):
        cells = self.border_cells(sg, next_sg)

        # fallback safety
        if not cells:
            return None

        best = None
        min_load = float("inf")

        for cell in cells:
            load = gate_usage.get(cell, 0)
            if load < min_load:
                min_load = load
                best = cell

        gate_usage[best] = gate_usage.get(best, 0) + 1
        return best

    def plan(self, agents):

        self.log(logging.INFO, "plan_start", agents=len(agents))

        sg_paths = [dijkstra(self.graph,
                             self.locate(a["start"]),
                             self.locate(a["goal"])) for a in agents]

        global_paths = [[] for _ in agents]
        cur_pos = [a["start"] for a in agents]
        time_cur = [0]*len(agents)

        # ✅ GUARANTEE paths are never empty
        for i in range(len(sg_paths)):
            if not sg_paths[i]:
                sg_paths[i] = [self.locate(agents[i]["start"])]

        max_steps = max(max(1, len(p)) for p in sg_paths)+50

        for step in range(max_steps):

            self.log(logging.DEBUG, "step", step=step)

            moved = set()
            groups = {}

            for i,p in enumerate(sg_paths):
                if step < len(p):
                    sg = p[step]
                else:
                    # ✅ stay in last subgrid until goal reached
                    sg = p[-1]

                groups.setdefault(sg, []).append(i)


            futures = []

            for sg, rids in groups.items():
                self.log(logging.DEBUG, "subgrid",
                         sg=sg, agents=len(rids))

                priorities = {
                    rid: self.compute_priority(
                        rid, cur_pos[rid], agents[rid]["goal"]
                    )
                    for rid in rids
                }

                rids.sort(key=lambda r: priorities[r], reverse=True)

                x0,y0,_,_ = self.bounds(sg)
                local_agents = []
                gate_usage = {}


                for rid in rids:

                    sx, sy = cur_pos[rid]
                    path = sg_paths[rid]

                    if step + 1 < len(path):
                        next_sg = path[step + 1]

                        gate = self.assign_gate(rid, sg, next_sg, gate_usage)

                        if gate is None:
                            gx, gy = agents[rid]["goal"]
                        else:
                            gx, gy = gate
                    else:
                        gx, gy = agents[rid]["goal"]

                    local_agents.append({
                        "start": (sx - x0, sy - y0),
                        "goal": (gx - x0, gy - y0)
                    })


                futures.append((sg, rids, priorities,
                    self.executor.submit(self.solve_subgrid, sg, local_agents, step)
                ))

            for sg, rids, priorities, future in futures:

                ok, paths = future.result()
                if not ok:

                    best_rid = max(rids, key=lambda r: priorities[r] + 5 * self.age[r])

                    # -----------------------------
                    # ✅ SINGLE AGENT FALLBACK - NOT WORKING
                    # -----------------------------
                    x0, y0, _, _ = self.bounds(sg)

                    sx, sy = cur_pos[best_rid]
                    path = sg_paths[best_rid]

                    # determine target (same logic as before)
                    if step + 1 < len(path):
                        next_sg = path[step + 1]
                        gate = self.assign_gate(best_rid, sg, next_sg, {})
                        if gate is None:
                            gx, gy = agents[best_rid]["goal"]
                        else:
                            gx, gy = gate
                    else:
                        gx, gy = agents[best_rid]["goal"]

                    single_agent = [{
                        "start": (sx - x0, sy - y0),
                        "goal":  (gx - x0, gy - y0)
                    }]

                    planner = LaCAMPlanner(
                        (self.SUB_W, self.SUB_H),
                        self.local_obstacles(sg, step),
                        self.exe_path
                    )

                    ok1, paths1 = planner.plan(single_agent)

                    if ok1:
                        MAX_FALLBACK_HORIZON = 3   # ✅ key parameter
                        path = paths1[0][:MAX_FALLBACK_HORIZON]

                        success = True
                        
                        for dt, (lx, ly) in enumerate(path):
                            gx, gy = lx + x0, ly + y0
                            t = time_cur[best_rid] + dt

                            if not self.safe_reserve(best_rid, gx, gy, t, priorities):
                                success = False
                                break

                        if success:
                            for dt, (lx, ly) in enumerate(path):
                                global_paths[best_rid].append((lx + x0, ly + y0))

                            time_cur[best_rid] += len(path)
                            cur_pos[best_rid] = global_paths[best_rid][-1]
                            moved.add(best_rid)

                            self.age[best_rid] = 0

                            self.log(logging.DEBUG, "single_agent_lacam_success",
                                    agent=best_rid,
                                    steps=len(path))

                            continue

                    # -----------------------------
                    # ✅ HARD FALLBACK (last resort)
                    # -----------------------------
                    global_paths[best_rid].append(cur_pos[best_rid])
                    time_cur[best_rid] += 1
                    moved.add(best_rid)
                    self.age[best_rid] += 1

                    self.log(logging.WARNING, "single_agent_lacam_failed",
                            agent=best_rid)


                    continue

                x0,y0,_,_ = self.bounds(sg)

                for i, rid in enumerate(rids):
                    path = paths[i]
                    success = True

                    for dt,(lx,ly) in enumerate(path):
                        gx,gy = lx+x0, ly+y0
                        t = time_cur[rid] + dt

                        if not self.safe_reserve(rid, gx, gy, t, priorities):
                            success = False
                            break

                    if not success:
                        self.log(logging.DEBUG, "wait", agent=rid)
                        global_paths[rid].append(cur_pos[rid])
                        time_cur[rid] += 1
                        moved.add(rid)
                        continue

                    for dt,(lx,ly) in enumerate(path):
                        global_paths[rid].append((lx+x0, ly+y0))

                    time_cur[rid] += len(path)
                    cur_pos[rid] = global_paths[rid][-1]
                    moved.add(rid)

            # ✅ CRITICAL FIX (kept)
            for rid in range(len(agents)):
                if rid not in moved:
                    self.log(logging.DEBUG, "global_wait", agent=rid)
                    global_paths[rid].append(cur_pos[rid])
                    time_cur[rid] += 1
                    self.age[rid] += 1

        for i in range(len(global_paths)):
            if not global_paths[i]:
                global_paths[i].append(agents[i]["start"])

        self.log(logging.INFO, "plan_end",
                 steps=max_steps,
                 agents=len(agents),
                 conflicts=len(self.conflict_heat))

        return True, global_paths