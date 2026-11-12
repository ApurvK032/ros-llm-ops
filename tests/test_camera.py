"""Where points in the warehouse land in a camera frame (plain geometry, no simulator)."""

import math
import unittest

from warehouse_agent.camera import OVERVIEW, aim, project

DOWN = {"position": (2.0, -3.0, 10.0), "look_at": (2.0, -3.0, 0.0), "hfov": math.pi/2, "width": 800, "height": 600}
LEVEL = {"position": (0.0, 0.0, 1.0), "look_at": (10.0, 0.0, 1.0), "hfov": math.pi/2, "width": 800, "height": 600}


class Projection(unittest.TestCase):
    def assertPixel(self, got, expected):
        self.assertIsNotNone(got)
        for value, wanted in zip(got, expected, strict=True):
            self.assertAlmostEqual(value, wanted, places=6)

    def test_the_point_looked_at_is_the_centre_of_the_frame(self):
        self.assertPixel(project(OVERVIEW["look_at"]), (OVERVIEW["width"]/2, OVERVIEW["height"]/2))
        self.assertPixel(project((10.0, 0.0, 1.0), LEVEL), (400, 300))

    def test_left_is_left_and_up_is_up(self):
        # Looking along +x with a 90 degree field of view: one metre aside at one metre ahead is the frame's edge.
        self.assertPixel(project((1.0, 1.0, 1.0), LEVEL), (0, 300))
        self.assertPixel(project((1.0, -0.5, 1.0), LEVEL), (600, 300))
        self.assertPixel(project((2.0, 0.0, 2.0), LEVEL), (400, 100))

    def test_a_camera_looking_straight_down_sees_the_floor_as_a_plan(self):
        # Its yaw defaults to +x, so +x on the floor is up the frame and +y is to the left.
        self.assertPixel(project((2.0, -3.0, 0.0), DOWN), (400, 300))
        self.assertPixel(project((7.0, -3.0, 0.0), DOWN), (400, 100))
        self.assertPixel(project((2.0, 2.0, 0.0), DOWN), (200, 300))

    def test_a_point_behind_the_camera_is_not_drawn(self):
        self.assertIsNone(project((-5.0, 0.0, 1.0), LEVEL))

    def test_the_overview_has_the_whole_floor_in_frame(self):
        yaw, pitch = aim(OVERVIEW)
        self.assertAlmostEqual(yaw, math.pi/2)
        self.assertTrue(0.5 < pitch < 0.9)
        for corner in ((-9, -7), (9, -7), (9, 7), (-9, 7)):
            column, row = project((*corner, 0.0))
            self.assertTrue(0 < column < OVERVIEW["width"] and 0 < row < OVERVIEW["height"], corner)


if __name__ == "__main__":
    unittest.main()
