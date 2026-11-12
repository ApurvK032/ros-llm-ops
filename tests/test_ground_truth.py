"""The true-pose check on pickups and deliveries, on synthetic journals and recordings (no simulator)."""

import json
import math
import tempfile
import unittest
from pathlib import Path

from warehouse_agent.ground_truth import check_transfers, parse_pose, pose_at, read_truth
from warehouse_agent.world import World

WORLD = World()


def transfer(kind, parcel, waypoint, at, pose=None):
    """A journal event as the supervisor writes it when it accepts a pickup or a delivery."""
    return {"type": "cargo_" + kind, "monotonic": at, "parcel": parcel, "waypoint": waypoint,
            "position_error": 0.02, "pose": list(pose or WORLD.waypoints[waypoint])}


def finished(at, **parcels):
    return {"type": "mission_finished", "monotonic": at,
            "state": {"parcels": {pid: {"state": state} for pid, state in parcels.items()}}}


class PoseStream(unittest.TestCase):
    def test_the_robot_is_found_among_the_models_and_yaw_comes_from_the_quaternion(self):
        half = math.pi/4
        line = json.dumps({"pose": [
            {"name": "base_link", "position": {"x": 9.0}},
            {"name": "turtlebot3_waffle", "position": {"x": 1.5, "y": -0.9, "z": 0.01},
             "orientation": {"z": math.sin(half), "w": math.cos(half)}}]})
        x, y, yaw = parse_pose(line)
        self.assertEqual((x, y), (1.5, -0.9))
        self.assertAlmostEqual(yaw, math.pi/2)

    def test_fields_that_are_zero_are_left_out_of_the_message(self):
        line = json.dumps({"pose": [{"name": "turtlebot3_waffle", "position": {"y": 2.0}, "orientation": {"w": 1}}]})
        self.assertEqual(parse_pose(line), (0.0, 2.0, 0.0))

    def test_a_message_without_the_robot_or_a_broken_line_gives_nothing(self):
        self.assertIsNone(parse_pose(json.dumps({"pose": [{"name": "crate"}]})))
        self.assertIsNone(parse_pose('{"pose": [{"name": "turtlebot3_w'))
        self.assertIsNone(parse_pose("[]"))

    def test_a_partly_written_last_row_is_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "truth.jsonl"
            path.write_text("[1.0, 0.0, 0.0, 0.0]\n[1.1, 0.1, 0.0, 0.0]\n[1.2, 0.2")
            self.assertEqual(read_truth(path), [[1.0, 0.0, 0.0, 0.0], [1.1, 0.1, 0.0, 0.0]])
            self.assertEqual(read_truth(Path(directory) / "missing.jsonl"), [])

    def test_the_nearest_recorded_pose_is_used_and_a_distant_one_is_not(self):
        rows = [[10.0, 1.0, 0.0, 0.0], [10.1, 2.0, 0.0, 0.0], [10.2, 3.0, 0.0, 0.0]]
        self.assertEqual(pose_at(rows, 10.09), (2.0, 0.0, 0.0))
        self.assertEqual(pose_at(rows, 9.5), (1.0, 0.0, 0.0))
        self.assertEqual(pose_at(rows, 10.9), (3.0, 0.0, 0.0))
        self.assertIsNone(pose_at(rows, 12.0))
        self.assertIsNone(pose_at([], 10.0))


class Transfers(unittest.TestCase):
    def test_a_delivery_made_where_the_journal_says_is_verified(self):
        events = [transfer("pickup", "P2", "PICK_B", 50.0), transfer("drop", "P2", "DROP_B", 80.0),
                  finished(81.0, P2="delivered")]
        rows = [[50.0, *WORLD.waypoints["PICK_B"]], [80.02, 7.05, 0.03, 0.02]]
        check = check_transfers(events, rows, WORLD)
        self.assertEqual(check["delivered_verified"], ["P2"])
        self.assertEqual(check["delivered_not_checked"], [])
        self.assertTrue(all(entry["verified"] for entry in check["transfers"]))
        self.assertAlmostEqual(check["worst_position_error"], 0.058, places=3)

    def test_a_delivery_the_robot_made_somewhere_else_is_not_verified(self):
        # The estimate says DROP_B; the robot is really a metre and a half away, as after a pose jump.
        events = [transfer("pickup", "P2", "PICK_B", 50.0), transfer("drop", "P2", "DROP_B", 80.0),
                  finished(81.0, P2="delivered")]
        rows = [[50.0, *WORLD.waypoints["PICK_B"]], [80.0, 5.6, 0.0, 0.0]]
        check = check_transfers(events, rows, WORLD)
        self.assertEqual(check["delivered_verified"], [])
        drop = check["transfers"][1]
        self.assertFalse(drop["verified"])
        self.assertEqual((drop["position_error"], drop["estimate_off_by"]), (1.5, 1.5))

    def test_without_a_recorded_pose_near_the_event_nothing_is_claimed_either_way(self):
        events = [transfer("pickup", "P2", "PICK_B", 50.0), transfer("drop", "P2", "DROP_B", 80.0),
                  finished(81.0, P2="delivered")]
        check = check_transfers(events, [[50.0, *WORLD.waypoints["PICK_B"]]], WORLD)
        self.assertEqual(check["delivered_verified"], [])
        self.assertEqual(check["delivered_not_checked"], ["P2"])
        self.assertIsNone(check["transfers"][1]["verified"])

    def test_a_parcel_still_on_board_is_not_a_delivery(self):
        events = [transfer("pickup", "P1", "PICK_A", 20.0), finished(60.0, P1="onboard")]
        check = check_transfers(events, [[20.0, *WORLD.waypoints["PICK_A"]]], WORLD)
        self.assertEqual((check["delivered_verified"], check["delivered_not_checked"]), ([], []))


if __name__ == "__main__":
    unittest.main()
