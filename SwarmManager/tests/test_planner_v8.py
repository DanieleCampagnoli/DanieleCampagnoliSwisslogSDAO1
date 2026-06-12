import unittest
import random

from swarmmanager.planner_v8 import HierarchicalLaCAM  # <-- replace with your file


# ============================================
# HELPER VALIDATION FUNCTIONS
# ============================================

def paths_reach_goals(agents, paths):
    for i, a in enumerate(agents):
        if not paths[i]:
            return False
        if paths[i][-1] != a["goal"]:
            return False
    return True


def no_collisions(paths):
    max_len = max(len(p) for p in paths)

    for t in range(max_len):
        occupied = {}

        for i, p in enumerate(paths):
            pos = p[t] if t < len(p) else p[-1]

            # vertex conflict
            if pos in occupied:
                return False
            occupied[pos] = i

        # edge conflict
        if t > 0:
            for i, p1 in enumerate(paths):
                prev1 = p1[t-1] if t-1 < len(p1) else p1[-1]
                curr1 = p1[t]   if t   < len(p1) else p1[-1]

                for j, p2 in enumerate(paths):
                    if i >= j:
                        continue
                    prev2 = p2[t-1] if t-1 < len(p2) else p2[-1]
                    curr2 = p2[t]   if t   < len(p2) else p2[-1]

                    if prev1 == curr2 and prev2 == curr1:
                        return False

    return True


# ============================================
# TEST CLASS
# ============================================

class TestHierarchicalLaCAM(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.exe_path = "C:/dev/workspaces/DanieleCampagnoliSwisslogSDAO1/SwarmManager/lacam/lacam.exe"  # <-- set correct path


    # ----------------------------------------
    # BASIC FUNCTIONALITY
    # ----------------------------------------

    def test_single_agent(self):
        planner = HierarchicalLaCAM((50,50), [], self.exe_path)

        agents = [
            {"start": (0,0), "goal": (10,10)}
        ]

        ok, paths = planner.plan(agents)

        self.assertTrue(ok)
        self.assertTrue(paths_reach_goals(agents, paths))
        self.assertTrue(no_collisions(paths))


    # ----------------------------------------
    # MULTI-AGENT SMALL
    # ----------------------------------------

    def test_small_multi_agent(self):
        planner = HierarchicalLaCAM((50,50), [], self.exe_path)

        agents = [
            {"start": (0,0), "goal": (10,10)},
            {"start": (10,10), "goal": (0,0)},
            {"start": (5,0), "goal": (5,10)},
        ]

        ok, paths = planner.plan(agents)

        self.assertTrue(ok)
        self.assertTrue(paths_reach_goals(agents, paths))
        self.assertTrue(no_collisions(paths))


    # ----------------------------------------
    # BORDER CONFLICT TEST
    # ----------------------------------------

    def test_border_conflict(self):
        planner = HierarchicalLaCAM((40,40), [], self.exe_path, sub_size=(20,20))

        agents = [
            {"start": (19,10), "goal": (30,10)},
            {"start": (20,10), "goal": (5,10)},
        ]

        # crossing subgrid boundary in opposite directions

        ok, paths = planner.plan(agents)

        self.assertTrue(ok)
        self.assertTrue(no_collisions(paths))


    # ----------------------------------------
    # PRIORITY TEST (NO DEADLOCK)
    # ----------------------------------------

    def test_priority_deadlock(self):
        planner = HierarchicalLaCAM((40,40), [], self.exe_path)

        # classic cycle conflict
        agents = [
            {"start": (10,10), "goal": (10,12)},
            {"start": (10,12), "goal": (12,12)},
            {"start": (12,12), "goal": (12,10)},
            {"start": (12,10), "goal": (10,10)},
        ]

        ok, paths = planner.plan(agents)

        self.assertTrue(ok)
        self.assertTrue(no_collisions(paths))
        self.assertTrue(paths_reach_goals(agents, paths))


    # ----------------------------------------
    # OBSTACLE TEST
    # ----------------------------------------

    def test_with_obstacles(self):
        obstacles = [(x,10) for x in range(5,15)]

        planner = HierarchicalLaCAM((50,50), obstacles, self.exe_path)

        agents = [
            {"start": (0,0), "goal": (20,20)},
            {"start": (20,0), "goal": (0,20)},
        ]

        ok, paths = planner.plan(agents)

        self.assertTrue(ok)
        self.assertTrue(no_collisions(paths))


    # ----------------------------------------
    # RANDOM STRESS TEST (MEDIUM)
    # ----------------------------------------

    def test_random_50_agents(self):
        planner = HierarchicalLaCAM((60,60), [], self.exe_path)

        agents = []
        for _ in range(50):
            sx, sy = random.randint(0,59), random.randint(0,59)
            gx, gy = random.randint(0,59), random.randint(0,59)
            agents.append({"start": (sx,sy), "goal": (gx,gy)})

        ok, paths = planner.plan(agents)

        self.assertTrue(ok)
        self.assertTrue(no_collisions(paths))


    # ----------------------------------------
    # STARVATION TEST (AGING)
    # ----------------------------------------

    def test_starvation_prevention(self):
        planner = HierarchicalLaCAM((30,30), [], self.exe_path)

        # one robot will initially lose priority repeatedly
        agents = [
            {"start": (0,0), "goal": (25,25)},   # long path
            {"start": (1,0), "goal": (25,25)},
            {"start": (2,0), "goal": (25,25)},
            {"start": (3,0), "goal": (25,25)},
        ]

        ok, paths = planner.plan(agents)

        self.assertTrue(ok)
        self.assertTrue(paths_reach_goals(agents, paths))


    # ----------------------------------------
    # PERFORMANCE SMOKE TEST
    # ----------------------------------------

    def test_200_agents_smoke(self):
        planner = HierarchicalLaCAM((100,100), [], self.exe_path)

        agents = []
        for _ in range(200):
            sx, sy = random.randint(0,99), random.randint(0,99)
            gx, gy = random.randint(0,99), random.randint(0,99)
            agents.append({"start": (sx,sy), "goal": (gx,gy)})

        ok, paths = planner.plan(agents)

        self.assertTrue(ok)
        self.assertTrue(no_collisions(paths))


# ============================================
# RUN
# ============================================

if __name__ == "__main__":
    unittest.main()