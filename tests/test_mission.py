"""Cargo, precedence, cancellation and failure checks independent of ROS."""

import tempfile
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
