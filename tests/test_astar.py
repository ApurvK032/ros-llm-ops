"""Grid A* returns shortest 4-connected paths and never invents a route."""

import math
import unittest
from itertools import pairwise

from warehouse_agent.astar import a_star

GRID = [[0, 0, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 0, 0]]


class AStarTests(unittest.TestCase):
    def test_shortest_path_is_valid_for_both_heuristics(self):
        for heuristic in ("manhattan", "euclidean"):
            with self.subTest(heuristic=heuristic):
                result = a_star(GRID, (0, 0), (4, 4), heuristic)
                path = result["path"]
                self.assertTrue(result["found"])
                self.assertEqual(result["cost"], 8)
                self.assertEqual((path[0], path[-1], len(path)), ((0, 0), (4, 4), 9))
                for (r0, c0), (r1, c1) in pairwise(path):
                    self.assertEqual(abs(r0-r1)+abs(c0-c1), 1)
                    self.assertEqual(GRID[r1][c1], 0)

    def test_start_equals_goal(self):
        self.assertEqual(a_star(GRID, (2, 2), (2, 2))["cost"], 0)

    def test_blocked_out_of_bounds_and_walled_off_goals_are_unreachable(self):
        walled = [[0, 1, 0], [1, 1, 0], [0, 0, 0]]
        for grid, start, goal in ((GRID, (0, 0), (0, 3)), (GRID, (0, 0), (9, 9)), (GRID, (1, 1), (0, 0)),
                                  (walled, (0, 0), (2, 2))):
            with self.subTest(start=start, goal=goal):
                result = a_star(grid, start, goal)
                self.assertFalse(result["found"])
                self.assertEqual((result["cost"], result["path"]), (math.inf, []))


if __name__ == "__main__":
    unittest.main()
