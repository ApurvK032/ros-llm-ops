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
