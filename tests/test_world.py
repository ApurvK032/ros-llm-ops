"""Every approach must be reachable, face its parcel, and keep clear of it."""
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from warehouse_agent.planner import Planner
from warehouse_agent.world import World


class WarehouseTests(unittest.TestCase):
    def setUp(self):
        self.world = World()

    def test_all_stations_are_reachable_through_shelf_aisles(self):
        planner = Planner(self.world)
        home = self.world.waypoints["HOME"]
        for name, pose in self.world.waypoints.items():
            with self.subTest(station=name):
                self.assertFalse(self.world.blocked(*pose[:2], self.world.config["planning_inflation"]))
                self.assertTrue(math.isfinite(planner.distance(home, pose)))
        a, b = (self.world.waypoints[name] for name in ("PICK_A", "PICK_B"))
        self.assertGreater(planner.distance(a, b), math.dist(a[:2], b[:2])+3.0)

    def test_snapping_never_crosses_a_rack(self):
        planner, home = Planner(self.world), self.world.waypoints["HOME"]
        grid = self.world.grid
        free = [c for c in ((r, k) for r in range(len(grid)) for k in range(len(grid[0]))) if self.world.free(c)]
        pose = [-5.8, 0.0, 0.0]
        self.assertFalse(self.world.free(self.world.cell(pose)))
        # West of the rack, 0.1 m off each axis; the equidistant (35, 15) loses the tie to the lower row.
        cell, offset = self.world.snap(pose)
        self.assertEqual(cell, (34, 15))
        self.assertAlmostEqual(offset, math.hypot(0.1, 0.1))
        self.assertAlmostEqual(planner.distance(pose, home), math.hypot(0.1, 0.1)+planner.distance([-5.9, -0.1, 0.0], home))
        # Two poses in one blocked cell by the rack's south-west corner snap south and west respectively;
        # the A* cache must be keyed on the snapped cell, not the shared blocked one.
        south, west = [-5.76, -3.56, 0.0], [-5.76, -3.44, 0.0]
        self.assertEqual(self.world.cell(south), self.world.cell(west))
        self.assertNotEqual(self.world.snap(south)[0], self.world.snap(west)[0])
        for pose in (south, west):
            cell, offset = self.world.snap(pose)
            fresh = Planner(self.world).distance([*self.world.centre(cell), 0.0], home)
            self.assertAlmostEqual(planner.distance(pose, home), offset+fresh)
        for shelf in self.world.config["shelves"]:
            self.assertEqual(self.world.snap([*shelf["center"], 0.0]), (None, math.inf))
            for axis, side in ((0, -1), (0, 1), (1, -1), (1, 1)):
                face = shelf["center"][axis]+side*shelf["size"][axis]/2
                for depth in (self.world.config["robot_radius"], self.world.config["planning_inflation"]-0.05):
                    pose = [*shelf["center"], 0.0]
                    pose[axis] = face+side*depth
                    if self.world.blocked(*pose[:2]):
                        continue  # This face meets another rack.
                    with self.subTest(shelf=shelf["name"], axis=axis, side=side, depth=depth):
                        cell, offset = self.world.snap(pose)
                        self.assertGreater(side*(self.world.centre(cell)[axis]-face), 0)
                        if self.world.free(self.world.cell(pose)):
                            self.assertEqual((cell, offset), (self.world.cell(pose), 0.0))
                        else:
                            self.assertAlmostEqual(offset, math.dist(pose[:2], self.world.centre(cell)))
                            self.assertAlmostEqual(offset, min(math.dist(pose[:2], self.world.centre(c)) for c in free))

    def test_station_approaches_face_parcels_without_overlap(self):
        for name in self.world.config["stations"]:
            with self.subTest(station=name):
                pose = self.world.waypoints[name]
                report = self.world.arrival(pose, name)
                self.assertTrue(report["accepted"])
                self.assertAlmostEqual(report["parcel_distance"], 1.0)
                self.assertAlmostEqual(report["facing_error"], 0.0)
                self.assertGreater(report["parcel_clearance"], 0.5)
                wrapped = [*pose[:2], pose[2]+math.tau]
                self.assertTrue(self.world.arrival(wrapped, name)["accepted"])

    def test_correct_goal_heading_must_also_face_actual_parcel(self):
        self.world.config["arrival_tolerance"] = 1.0
        x, y, yaw = self.world.waypoints["PICK_A"]
        report = self.world.arrival([x, y+0.6, yaw], "PICK_A")
        self.assertEqual(report["heading_error"], 0)
        self.assertGreater(report["facing_error"], self.world.config["facing_tolerance"])
        self.assertFalse(report["accepted"])

    def test_generated_collisions_match_shelves_and_drop_pedestals(self):
        with tempfile.TemporaryDirectory() as directory:
            self.world.generate(directory)
            root = ET.parse(Path(directory)/"warehouse.sdf").getroot()
            models = {model.attrib["name"]: model for model in root.findall("world/model")}
            for obstacle in self.world.obstacles():
                model = models[obstacle["name"]]
                size = list(map(float, model.findtext("link/collision/geometry/box/size").split()))
                self.assertEqual(size[:2], obstacle["size"])
            self.assertEqual(models["PICK_A"].findtext("pose").split()[:2], ["-6.3", "-1.5"])
            gui = (Path(directory)/"warehouse.gui.config").read_text()
            self.assertIn("11.0 -14.0 18.0", gui)


if __name__ == "__main__":
    unittest.main()
