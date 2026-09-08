"""Cargo, precedence, cancellation and failure checks independent of ROS."""

import tempfile
import math
import unittest
from pathlib import Path

from warehouse_agent.mission import Mission
from warehouse_agent.world import World


class Backend:
    name = "test_double"

    def __init__(self, world):
        self.pose = list(world.waypoints["HOME"])
        self.target = None
        self.result = None
        self.cancelled = False
        self.dispatched = 0

    def navigate(self, pose):
        self.target = list(pose)
        self.result = None
        self.cancelled = False
        self.dispatched += 1

    def arrive(self):
        self.pose = self.target
        self.result = "succeeded"

    def spin(self):
        pass

    def poll(self):
        return self.result

    def cancel(self):
        self.cancelled = True

    def close(self):
        pass


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.world = World()
        self.backend = Backend(self.world)
        self.events = []
        self.mission = Mission(self.world, self.backend, Path(self.tmp.name)/"episode.jsonl", self.events.append)

    def tearDown(self):
        self.backend.result = "cancelled"
        self.mission.close()
        self.tmp.cleanup()

    def start(self, parcels=("P1", "P2", "P3")):
        self.mission.apply({"operation": "create", "parcels": list(parcels)})
        self.mission.tick()

    def test_three_deliveries_preserve_cargo_order(self):
        self.start()
        for _ in range(6):
            self.backend.arrive()
            self.mission.tick()
        self.assertTrue(self.mission.finished)
        self.assertTrue(all(p["state"] == "delivered" for p in self.mission.parcels.values()))
        cargo = [e for e in self.events if e["type"].startswith("cargo_")]
        for pid in self.mission.parcels:
            self.assertEqual([e["type"] for e in cargo if e["parcel"] == pid], ["cargo_pickup", "cargo_drop"])

    def test_success_outside_tolerance_does_not_pick_up(self):
        self.start(["P1"])
        self.backend.result = "succeeded"
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"]["state"], "awaiting_pickup")

    def test_success_at_approach_facing_away_does_not_pick_up(self):
        self.start(["P1"])
        target = list(self.backend.target)
        self.backend.pose = [*target[:2], target[2]+math.pi]
        self.backend.result = "succeeded"
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"]["state"], "awaiting_pickup")
        self.assertFalse(any(e["type"] == "cargo_pickup" for e in self.events))
        self.backend.arrive()
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"]["state"], "onboard")

    def test_overlapping_parcel_does_not_pick_up_even_with_loose_position_tolerance(self):
        self.start(["P1"])
        self.world.config["arrival_tolerance"] = 2.0
        x, y, _ = self.world.station_position("PICK_A")
        self.backend.pose = [x-0.01, y, 0.0]
        self.backend.result = "succeeded"
        # This synthetic pose is inside a rack; retry planning must also refuse it.
        with self.assertRaisesRegex(ValueError, "No estimated route"):
            self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"]["state"], "awaiting_pickup")
        retry = next(e for e in self.events if e["type"] == "navigation_retry")
        self.assertLess(retry["arrival"]["parcel_clearance"], 0)

    def test_drop_requires_facing_the_delivery_station(self):
        self.start(["P1"])
        self.backend.arrive()
        self.mission.tick()
        self.backend.pose = [*self.backend.target[:2], math.pi]
        self.backend.result = "succeeded"
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"]["state"], "onboard")
        self.backend.arrive()
        self.mission.tick()
        drop = next(e for e in self.events if e["type"] == "cargo_drop")
        self.assertTrue(drop["accepted"])
        self.assertGreaterEqual(drop["parcel_clearance"], self.world.config["minimum_parcel_clearance"])

    def test_cancel_waits_for_terminal_and_suppresses_pickup(self):
        self.start(["P1", "P2"])
        self.mission.apply({"operation": "cancel", "parcels": ["P1"]})
        self.mission.tick()
        self.assertEqual(self.backend.dispatched, 1)
        self.assertTrue(self.backend.cancelled)
        # A success racing with cancel is still only arrival, never pickup.
        self.backend.arrive()
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"]["state"], "awaiting_pickup")
        self.mission.tick()
        self.assertEqual(self.mission.active.parcel, "P2")

    def test_language_inference_fences_cargo(self):
        self.start(["P1"])
        self.mission.language_pending = True
        self.backend.arrive()
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"]["state"], "awaiting_pickup")
        self.mission.apply({"operation": "cancel", "parcels": ["P1"]})
        self.mission.language_pending = False
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"]["disposition"], "cancelled")
        self.assertFalse(any(e["type"] == "cargo_pickup" for e in self.events))

    def test_onboard_cancel_holds_and_preserves_cargo(self):
        self.start(["P1"])
        self.backend.arrive()
        self.mission.tick()
        self.mission.apply({"operation": "cancel", "parcels": ["P1"]})
        self.assertTrue(self.mission.paused)
        self.assertEqual(self.mission.parcels["P1"]["state"], "onboard")
        self.assertEqual(self.mission.parcels["P1"]["disposition"], "active")

    def test_two_failures_defer_and_continue(self):
        self.start(["P1", "P2"])
        for _ in range(2):
            self.backend.result = "failed"
            self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"]["disposition"], "deferred")
        self.assertEqual(self.mission.active.parcel, "P2")

    def test_priority_preserves_all_obligations(self):
        self.mission.apply({"operation": "create", "parcels": ["P1", "P2", "P3"]})
        self.mission.apply({"operation": "prioritize", "parcels": ["P3"]})
        route, _ = self.mission.planner.plan(self.mission.parcels, self.backend.pose, self.mission.priority)
        self.assertEqual([(s.parcel, s.kind) for s in route[:2]], [("P3", "pickup"), ("P3", "drop")])
        self.assertEqual(len(route), 6)

    def test_onboard_first_overrides_priority(self):
        self.mission.apply({"operation": "create", "parcels": ["P1", "P2", "P3"]})
        self.mission.parcels["P1"]["state"] = "onboard"
        route, _ = self.mission.planner.plan(self.mission.parcels, self.backend.pose, "P3", {"P1"})
        self.assertEqual((route[0].parcel, route[0].kind), ("P1", "drop"))
        self.assertEqual(len(route), 5)

    def test_unknown_ids_do_not_start_mission(self):
        with self.assertRaises(ValueError):
            self.mission.apply({"operation": "create", "parcels": ["P1", "P99"]})
        self.assertEqual(self.mission.parcels["P1"]["disposition"], "inactive")


if __name__ == "__main__":
    unittest.main()
