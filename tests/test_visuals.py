"""Visuals must follow confirmed cargo and the executor's actual stop order."""
import math
import unittest

import test_mission as fixtures
from warehouse_agent.visual_state import COLORS, parcel_label, project


class VisualTests(unittest.TestCase):
    setUp = fixtures.MissionTests.setUp
    tearDown = fixtures.MissionTests.tearDown
    start = fixtures.MissionTests.start
    def cube(self, pid):
        key = 100+list(self.mission.parcels).index(pid)*2
        return next(v for v in project(self.world, self.mission.snapshot()) if v.key == key)

    def test_visuals_wait_for_pickup_ack_then_follow_and_deliver(self):
        self.start(["P1"])
        pickup = tuple(self.world.waypoints["PICK_A"][:2])
        self.backend.pose = self.backend.target
        self.mission.tick()  # Being at the coordinates is not a pickup acknowledgement.
        self.assertEqual(self.cube("P1").position[:2], pickup)
        self.assertEqual(self.cube("P1").color, COLORS["awaiting_pickup"])
        self.backend.arrive()
        self.mission.tick()
        self.backend.pose = [0.2, 0.3, math.pi/2]
        self.assertEqual(self.cube("P1").position[:2], (0.2, 0.3))
        self.assertEqual(self.cube("P1").color, COLORS["onboard"])
        self.backend.arrive()
        self.mission.tick()
        self.assertEqual(self.cube("P1").position[:2], tuple(self.world.waypoints["DROP_A"][:2]))
        self.assertEqual(self.cube("P1").color, COLORS["delivered"])

    def test_cancelled_parcel_remains_at_pickup(self):
        self.start(["P1"])
        self.mission.apply({"operation": "cancel", "parcels": ["P1"]})
        self.assertEqual(self.cube("P1").position[:2], tuple(self.world.waypoints["PICK_A"][:2]))
        self.assertEqual(self.cube("P1").color, COLORS["cancelled"])
        self.assertEqual(parcel_label(self.mission.parcels["P1"]), "Cancelled · waiting")

    def test_deferred_onboard_cargo_stays_on_robot(self):
        self.start(["P1"])
        self.backend.arrive()
        self.mission.tick()
        for _ in range(2):
            self.backend.result = "failed"
            self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"]["state"], "onboard")
        self.assertEqual(self.cube("P1").position[:2], tuple(self.backend.pose[:2]))
        self.assertEqual(self.cube("P1").color, COLORS["deferred"])

    def test_remaining_list_matches_dispatch_and_hides_superseded_order(self):
        self.start()
        last_plan = next(e for e in reversed(self.events) if e["type"] == "plan_selected")
        self.assertEqual(self.mission.snapshot()["remaining_stops"], last_plan["stops"][1:])
        self.mission.apply({"operation": "prioritize", "parcels": ["P3"]})
        self.assertTrue(self.mission.snapshot()["plan_pending"])
        self.assertEqual(self.mission.snapshot()["remaining_stops"], [])
        self.backend.arrive()
        self.mission.tick()
        snapshot = self.mission.snapshot()
        self.assertFalse(snapshot["plan_pending"])
        self.assertEqual(snapshot["active"]["parcel"], "P3")
        self.assertNotIn({"parcel": "P1", "kind": "pickup", "waypoint": "PICK_A"}, snapshot["remaining_stops"])
