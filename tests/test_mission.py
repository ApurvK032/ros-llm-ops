"""Cargo, precedence, cancellation and failure checks independent of ROS."""

import copy
import json
import math
import tempfile
import unittest
from pathlib import Path

from warehouse_agent.mission import Mission
from warehouse_agent.state import Disposition, HoldKind, Physical
from warehouse_agent.verify import verify_file
from warehouse_agent.visual_state import mission_label
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
        self.assertTrue(all(p.state == "delivered" for p in self.mission.parcels.values()))
        cargo = [e for e in self.events if e["type"].startswith("cargo_")]
        for pid in self.mission.parcels:
            self.assertEqual([e["type"] for e in cargo if e["parcel"] == pid], ["cargo_pickup", "cargo_drop"])

    def test_success_outside_tolerance_does_not_pick_up(self):
        self.start(["P1"])
        self.backend.result = "succeeded"
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].state, "awaiting_pickup")

    def test_success_at_approach_facing_away_does_not_pick_up(self):
        self.start(["P1"])
        target = list(self.backend.target)
        self.backend.pose = [*target[:2], target[2]+math.pi]
        self.backend.result = "succeeded"
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].state, "awaiting_pickup")
        self.assertFalse(any(e["type"] == "cargo_pickup" for e in self.events))
        self.backend.arrive()
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].state, "onboard")

    def test_overlapping_parcel_does_not_pick_up_even_with_loose_position_tolerance(self):
        self.start(["P1"])
        self.world.config["arrival_tolerance"] = 2.0
        x, y, _ = self.world.station_position("PICK_A")
        self.backend.pose = [x-0.01, y, 0.0]
        self.backend.result = "succeeded"
        # This synthetic pose is inside a rack; retry planning holds instead of dispatching.
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].state, "awaiting_pickup")
        self.assertFalse(any(e["type"] == "cargo_pickup" for e in self.events))
        retry = next(e for e in self.events if e["type"] == "navigation_retry")
        self.assertLess(retry["arrival"]["parcel_clearance"], 0)
        self.assertTrue(self.mission.paused)
        self.assertEqual(self.backend.dispatched, 1)
        self.assertEqual([e["type"] for e in self.events].count("planning_failed"), 1)

    def test_resume_beside_rack_plans_from_nearest_free_cell(self):
        self.start()
        self.mission.apply({"operation": "pause", "parcels": []})
        # 0.30 m west of shelf_west_spine: legal for Nav2, but inside the planning inflation.
        self.backend.pose = [-5.8, 0.0, 0.0]
        self.backend.result = "cancelled"
        self.mission.tick()
        self.assertFalse(self.world.free(self.world.cell(self.backend.pose)))
        self.mission.apply({"operation": "resume", "parcels": []})
        self.mission.tick()
        self.assertFalse(self.mission.paused)
        self.assertEqual(self.backend.dispatched, 2)
        self.assertIsNotNone(self.mission.active)
        self.assertFalse(any(e["type"] == "planning_failed" for e in self.events))

    def test_unplannable_pose_holds_until_resume(self):
        self.start(["P1", "P2"])
        self.backend.arrive()
        self.mission.tick()
        self.mission.apply({"operation": "pause", "parcels": []})
        self.backend.pose = [-5.0, 0.0, 0.0]  # Centre of shelf_west_spine: no free cell within reach.
        self.backend.result = "cancelled"
        self.mission.tick()
        cargo, dispatched = copy.deepcopy(self.mission.parcels), self.backend.dispatched
        self.mission.apply({"operation": "resume", "parcels": []})
        for _ in range(3):
            self.mission.tick()
        self.assertTrue(self.mission.paused)
        self.assertIsNone(self.mission.active)
        self.assertEqual(self.backend.dispatched, dispatched)
        self.assertEqual(self.mission.parcels, cargo)
        self.assertEqual(cargo["P1"].state, "onboard")
        journal = [json.loads(line) for line in (Path(self.tmp.name)/"episode.jsonl").read_text().splitlines()]
        failed = [e for e in journal if e["type"] == "planning_failed"]
        self.assertEqual(len(failed), 1)
        self.assertEqual(failed[0]["pose"], [-5.0, 0.0, 0.0])
        self.assertIn("No estimated route", failed[0]["reason"])
        self.assertEqual(mission_label(self.mission.snapshot()), "Held: no route from current pose")
        # Resuming while still unplannable records one new failure and keeps holding.
        self.mission.apply({"operation": "resume", "parcels": []})
        self.mission.tick()
        journal = [json.loads(line) for line in (Path(self.tmp.name)/"episode.jsonl").read_text().splitlines()]
        self.assertEqual([e["type"] for e in journal].count("planning_failed"), 2)
        self.assertEqual(self.backend.dispatched, dispatched)
        self.backend.pose = list(self.world.waypoints["PICK_A"])
        self.mission.apply({"operation": "resume", "parcels": []})
        self.mission.tick()
        self.assertFalse(self.mission.paused)
        self.assertIsNone(self.mission.hold)
        self.assertEqual(self.backend.dispatched, dispatched+1)
        self.assertEqual(self.mission.parcels["P1"].state, "onboard")

    def test_drop_requires_facing_the_delivery_station(self):
        self.start(["P1"])
        self.backend.arrive()
        self.mission.tick()
        self.backend.pose = [*self.backend.target[:2], math.pi]
        self.backend.result = "succeeded"
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].state, "onboard")
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
        self.assertEqual(self.mission.parcels["P1"].state, "awaiting_pickup")
        self.mission.tick()
        self.assertEqual(self.mission.active.parcel, "P2")

    def test_language_inference_fences_cargo(self):
        self.start(["P1"])
        self.mission.language_pending = True
        self.backend.arrive()
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].state, "awaiting_pickup")
        self.mission.apply({"operation": "cancel", "parcels": ["P1"]})
        self.mission.language_pending = False
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].disposition, "cancelled")
        self.assertFalse(any(e["type"] == "cargo_pickup" for e in self.events))

    def test_onboard_cancel_holds_and_preserves_cargo(self):
        self.start(["P1"])
        self.backend.arrive()
        self.mission.tick()
        self.mission.apply({"operation": "cancel", "parcels": ["P1"]})
        self.assertTrue(self.mission.paused)
        self.assertEqual(self.mission.parcels["P1"].state, "onboard")
        self.assertEqual(self.mission.parcels["P1"].disposition, "active")

    def test_mixed_cancel_with_onboard_cargo_cancels_nothing_and_says_so(self):
        self.start(["P1", "P2"])
        self.backend.arrive()
        self.mission.tick()
        onboard = next(pid for pid, p in self.mission.parcels.items() if p.state == "onboard")
        waiting = next(pid for pid in ("P1", "P2") if pid != onboard)
        self.mission.apply({"operation": "cancel", "parcels": [waiting, onboard]})
        self.assertEqual(self.mission.hold.kind, "clarification")
        self.assertEqual({p.disposition for p in self.mission.parcels.values() if p.id in ("P1", "P2")}, {"active"})
        question = next(e for e in self.events if e["type"] == "clarification_required")["question"]
        self.assertIn(f"{onboard} is onboard, so nothing was cancelled", question)
        self.assertIn(f"resend the cancel for {waiting}", question)

    def test_every_pause_has_a_reason(self):
        self.start(["P1"])
        self.mission.apply({"operation": "pause", "parcels": []})
        self.assertEqual(self.mission.hold.kind, "operator")
        self.mission.apply({"operation": "resume", "parcels": []})
        self.assertIsNone(self.mission.hold)
        self.mission.fail_request("model timed out")
        self.assertEqual((self.mission.hold.kind, self.mission.hold.message), ("request_failed", "model timed out"))
        self.assertEqual(self.events[-1]["type"], "request_failed")

    def test_two_failures_defer_and_continue(self):
        self.start(["P1", "P2"])
        for _ in range(2):
            self.backend.result = "failed"
            self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].disposition, "deferred")
        self.assertEqual(self.mission.active.parcel, "P2")

    # --- recovery: fault holds and the direct defer / retry commands ---------------------------------------
    def held_mission(self, **options):
        """A mission that holds on faults, with a clock the test controls."""
        self.backend.result = "cancelled"
        self.mission.close()
        self.now = 1000.0
        self.mission = Mission(self.world, self.backend, Path(self.tmp.name)/"held.jsonl", self.events.append,
                               on_fault="hold", clock=lambda: self.now, **options)
        self.events.clear()
        self.start()

    def kinds(self):
        return [e["type"] for e in self.events]

    def test_a_failed_goal_holds_for_recovery_before_the_retry(self):
        self.held_mission()
        first = self.mission.active
        self.backend.result = "failed"
        self.mission.tick()
        self.assertEqual(self.mission.hold.kind, HoldKind.FAULT)
        self.assertIsNone(self.mission.active)
        self.assertEqual(self.kinds()[-2:], ["navigation_retry", "fault_hold"])
        dispatched = self.backend.dispatched
        self.mission.tick()
        self.assertEqual(self.backend.dispatched, dispatched)      # nothing moves while held
        self.mission.command("resume")
        self.mission.tick()
        self.assertEqual(self.mission.active, first)                # the same stop is retried
        self.assertIsNone(self.mission.hold)

    def test_a_fault_hold_ends_by_itself_and_the_normal_retry_policy_continues(self):
        self.held_mission(fault_hold_s=30.0)
        first = self.mission.active
        self.backend.result = "failed"
        self.mission.tick()
        self.now += 29.0
        self.mission.tick()
        self.assertEqual(self.mission.hold.kind, HoldKind.FAULT)
        self.now += 2.0
        self.mission.tick()
        self.assertIn("fault_hold_expired", self.kinds())
        self.assertEqual(self.mission.active, first)
        self.backend.result = "failed"                              # the retry fails too: deferred as before
        self.mission.tick()
        self.assertEqual(self.mission.parcels[first.parcel].disposition, Disposition.DEFERRED)
        self.assertIsNone(self.mission.hold)

    def test_an_operator_pause_replaces_a_fault_hold_and_does_not_expire(self):
        self.held_mission(fault_hold_s=30.0)
        self.backend.result = "failed"
        self.mission.tick()
        self.mission.command("pause")
        self.now += 300.0
        self.mission.tick()
        self.assertEqual(self.mission.hold.kind, HoldKind.OPERATOR)
        self.assertNotIn("fault_hold_expired", self.kinds())

    def test_a_clarification_that_replaces_a_fault_hold_does_not_expire_either(self):
        self.held_mission(fault_hold_s=30.0)
        self.backend.result = "failed"
        self.mission.tick()
        request = self.mission.submit("what?", "test-model")
        self.mission.resolve(request, {"operation": "clarify", "parcels": [], "message": "Which parcel?"})
        self.now += 300.0
        self.mission.tick()
        self.assertEqual(self.mission.hold.kind, HoldKind.CLARIFICATION)
        self.assertNotIn("fault_hold_expired", self.kinds())

    def test_without_the_option_a_failure_retries_at_once_as_before(self):
        self.start()
        first = self.mission.active
        self.backend.result = "failed"
        self.mission.tick()
        self.assertIsNone(self.mission.hold)
        self.assertEqual(self.mission.active, first)

    def test_defer_skips_a_parcel_and_the_rest_are_delivered(self):
        self.start()
        target = self.mission.active.parcel
        self.assertEqual(self.mission.command("defer", [target], "PICK is blocked"), "applied")
        self.assertTrue(self.backend.cancelled)                     # the goal toward it is cancelled first
        self.backend.result = "cancelled"
        for _ in range(8):
            self.mission.tick()
            if self.mission.active:
                self.backend.arrive()
        self.mission.tick()
        self.assertEqual(self.mission.parcels[target].as_dict()["reason"], "PICK is blocked")
        self.assertEqual({p.disposition for pid, p in self.mission.parcels.items() if pid != target},
                         {Disposition.ACTIVE})
        self.assertTrue(all(p.state is Physical.DELIVERED for pid, p in self.mission.parcels.items()
                            if pid != target))
        finished = [e for e in self.events if e["type"] == "mission_finished"]
        self.assertEqual(finished[-1]["result"], "partial")

    def test_retry_takes_a_deferred_parcel_up_again_with_fresh_attempts(self):
        self.start(["P1"])
        for _ in range(2):
            self.backend.result = "failed"
            self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].disposition, Disposition.DEFERRED)
        self.assertTrue(self.mission.finished)
        self.mission.command("retry", ["P1"])
        self.mission.tick()
        self.assertEqual(self.mission.active.parcel, "P1")
        self.assertIsNone(self.mission.parcels["P1"].reason)
        self.backend.result = "failed"                              # one failure is a retry again, not a deferral
        self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].disposition, Disposition.ACTIVE)
        for _ in range(2):
            self.backend.arrive()
            self.mission.tick()
        self.assertEqual(self.mission.parcels["P1"].state, Physical.DELIVERED)
        results = [e["result"] for e in self.events if e["type"] == "mission_finished"]
        self.assertEqual(results, ["partial", "complete"])

    def test_refused_recovery_commands_change_nothing_and_are_recorded(self):
        self.start(["P1"])
        before = self.mission.snapshot()["parcels"]
        for operation, parcels in (("defer", []), ("defer", ["P2"]), ("retry", ["P1"]), ("defer", ["P9"]),
                                   ("create", ["P2"])):
            with self.subTest(operation=operation, parcels=parcels), self.assertRaises(ValueError):
                self.mission.command(operation, parcels)
        self.assertEqual(self.mission.snapshot()["parcels"], before)
        self.assertEqual(self.kinds().count("command_rejected"), 5)

    def test_the_language_model_can_never_defer_or_retry(self):
        self.start(["P1"])
        for operation in ("defer", "retry"):
            request = self.mission.submit(f"{operation} P1", "test-model")
            outcome = self.mission.resolve(request, {"operation": operation, "parcels": ["P1"], "message": ""})
            self.assertEqual(outcome, "failed")
            self.assertEqual(self.mission.parcels["P1"].disposition, Disposition.ACTIVE)
            self.mission.command("resume")

    def test_journals_with_recovery_pass_the_independent_checker(self):
        self.held_mission(fault_hold_s=30.0)
        first = self.mission.active.parcel
        self.backend.result = "failed"
        self.mission.tick()                                         # fault hold
        self.mission.command("defer", [first], "blocked")
        self.mission.command("resume")
        self.mission.tick()
        self.backend.result = "failed"
        self.mission.tick()                                         # a second parcel: fault hold, then expiry
        self.now += 31.0
        self.mission.tick()
        self.mission.command("retry", [first])
        for _ in range(12):
            self.mission.tick()
            if self.mission.active:
                self.backend.arrive()
        self.mission.tick()
        self.assertTrue(all(p.state is Physical.DELIVERED for p in self.mission.parcels.values()))
        self.mission.close()
        report = verify_file(Path(self.tmp.name)/"held.jsonl")
        self.assertEqual([(v.code, v.message) for v in report.violations], [])
        self.assertEqual(report.warnings, [])
        self.mission = Mission(self.world, self.backend, Path(self.tmp.name)/"after.jsonl")

    def test_priority_preserves_all_obligations(self):
        self.mission.apply({"operation": "create", "parcels": ["P1", "P2", "P3"]})
        self.mission.apply({"operation": "prioritize", "parcels": ["P3"]})
        route, _ = self.mission.planner.plan(self.mission.parcels, self.backend.pose, self.mission.priority)
        self.assertEqual([(s.parcel, s.kind) for s in route[:2]], [("P3", "pickup"), ("P3", "drop")])
        self.assertEqual(len(route), 6)

    def test_onboard_first_overrides_priority(self):
        self.mission.apply({"operation": "create", "parcels": ["P1", "P2", "P3"]})
        self.mission.parcels["P1"].apply("pick_up")
        route, _ = self.mission.planner.plan(self.mission.parcels, self.backend.pose, "P3", {"P1"})
        self.assertEqual((route[0].parcel, route[0].kind), ("P1", "drop"))
        self.assertEqual(len(route), 5)

    def test_unknown_ids_do_not_start_mission(self):
        with self.assertRaises(ValueError):
            self.mission.apply({"operation": "create", "parcels": ["P1", "P99"]})
        self.assertEqual(self.mission.parcels["P1"].disposition, "inactive")


if __name__ == "__main__":
    unittest.main()
