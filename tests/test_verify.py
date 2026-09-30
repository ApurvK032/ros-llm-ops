"""The independent checker accepts real and scripted runs, and catches journals that break the rules."""

import copy
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from test_mission import Backend

from warehouse_agent.mission import Mission
from warehouse_agent.verify import check_snapshot, verify_events, verify_file
from warehouse_agent.world import ROOT, World

EVIDENCE = sorted((ROOT/"docs/evidence").rglob("*.jsonl"))
MAZE = ROOT/"docs/evidence/maze/delivery.jsonl"


def load(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def renumber(events):
    for number, event in enumerate(events, 1):
        event["sequence"] = number
    return events


def codes(report):
    return {finding.code for finding in report.violations}


class RecordedRunTests(unittest.TestCase):
    def test_every_recorded_gazebo_journal_verifies(self):
        self.assertGreaterEqual(len(EVIDENCE), 6)
        for path in EVIDENCE:
            with self.subTest(journal=path.relative_to(ROOT)):
                report = verify_file(path)
                self.assertTrue(report.ok, report.violations)

    def test_command_line_exit_codes(self):
        def verify(path):
            return subprocess.run([sys.executable, "-B", "-m", "warehouse_agent", "verify", str(path)],
                                  cwd=ROOT, capture_output=True, text=True).returncode
        self.assertEqual(verify(MAZE), 0)
        events = load(MAZE)
        next(e for e in events if e["type"] == "cargo_drop")["position_error"] = 5.0
        with tempfile.TemporaryDirectory() as directory:
            bad = Path(directory)/"bad.jsonl"
            bad.write_text("".join(json.dumps(e)+"\n" for e in events))
            self.assertEqual(verify(bad), 1)


class ScriptedRunTests(unittest.TestCase):
    """Drive the real supervisor with a backend double; every snapshot and the final journal must pass."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.world = World()
        self.backend = Backend(self.world)
        self.events = []
        self.mission = Mission(self.world, self.backend, Path(self.tmp.name)/"episode.jsonl", self.events.append)

    def tearDown(self):
        self.tmp.cleanup()

    def check(self):
        self.assertEqual(check_snapshot(self.mission.snapshot()), [])

    def tick(self, result=None):
        if result == "arrive":
            self.backend.arrive()
        elif result:
            self.backend.result = result
        self.mission.tick()
        self.check()

    def apply(self, operation, *parcels):
        self.mission.apply({"operation": operation, "parcels": list(parcels)})
        self.check()

    def interpret(self, operation, *parcels):
        """Submit a request through the supervisor's request API, as the CLI does."""
        intent = {"operation": operation, "parcels": list(parcels), "message": ""}
        return self.mission.submit(f"{operation} {parcels}", "test"), intent

    def resolve(self, request):
        request_id, intent = request
        self.mission.resolve(request_id, intent, model="test")
        self.check()

    def finish(self):
        if self.mission.active:
            self.backend.result = "cancelled"
        self.mission.close()
        report = verify_events(self.events, "scripted")
        self.assertTrue(report.ok, report.violations)
        self.assertEqual(report.warnings, [])
        return report

    def test_full_delivery(self):
        self.apply("create", "P1", "P2", "P3")
        self.tick()
        for _ in range(6):
            self.tick("arrive")
        self.assertEqual({p["state"] for p in self.finish().parcels.values()}, {"delivered"})

    def test_cancel_racing_an_arrival(self):
        self.apply("create", "P1", "P2")
        self.tick()
        self.apply("cancel", self.mission.active.parcel)
        self.tick("arrive")  # Nav2 reports success after the cancel was requested: arrival only.
        for _ in range(3):
            self.tick("arrive")
        parcels = self.finish().parcels
        self.assertEqual(sorted(p["disposition"] for p in parcels.values()), ["active", "cancelled", "inactive"])

    def test_cancelling_onboard_cargo_asks_then_resumes(self):
        self.apply("create", "P1")
        self.tick()
        self.tick("arrive")
        self.apply("cancel", "P1")
        self.assertTrue(self.mission.paused)
        self.tick("cancelled")
        self.apply("resume")
        self.tick()
        self.tick("arrive")
        self.assertEqual(self.finish().parcels["P1"]["state"], "delivered")

    def test_two_failures_defer_and_the_rest_continues(self):
        self.apply("create", "P1", "P2")
        self.tick()
        first = self.mission.active.parcel
        self.tick("failed")
        self.tick("failed")
        for _ in range(2):
            self.tick("arrive")
        parcels = self.finish().parcels
        self.assertEqual(parcels[first]["disposition"], "deferred")
        self.assertIn("delivered", {p["state"] for p in parcels.values()})

    def test_priority_change_mid_route(self):
        self.apply("create", "P1", "P2", "P3")
        self.tick()
        self.apply("prioritize", "P3")
        for _ in range(6):
            self.tick("arrive")
        self.finish()

    def test_pause_and_resume_mid_navigation(self):
        self.apply("create", "P1", "P2")
        self.tick()
        self.apply("pause")
        self.tick("cancelled")
        self.apply("resume")
        self.tick()
        for _ in range(4):
            self.tick("arrive")
        self.finish()

    def test_planning_hold_then_resume(self):
        self.apply("create", "P1")
        self.tick()
        self.apply("pause")
        self.backend.pose = [-5.0, 0.0, 0.0]  # Inside a rack: no route can be estimated.
        self.tick("cancelled")
        self.apply("resume")
        self.tick()
        self.assertEqual(self.mission.hold.kind, "no_route")
        self.backend.pose = list(self.world.waypoints["HOME"])
        self.apply("resume")
        self.tick()
        for _ in range(2):
            self.tick("arrive")
        self.finish()

    def test_request_pending_during_arrival_fences_cargo(self):
        self.apply("create", "P1", "P2")
        self.tick()
        request = self.interpret("cancel", self.mission.active.parcel)
        self.tick("arrive")  # The arrival is held while the request is being interpreted.
        self.assertIsNotNone(self.mission.active)
        self.resolve(request)
        intent = request[1]
        self.tick()
        for _ in range(2):
            self.tick("arrive")
        self.assertFalse(any(e["type"] == "cargo_pickup" and e["parcel"] == intent["parcels"][0] for e in self.events))
        self.finish()


class TamperedJournalTests(unittest.TestCase):
    """Each corruption of a real journal must be caught by the rule it breaks."""

    def setUp(self):
        self.events = load(MAZE)

    def find(self, kind, nth=0, **match):
        found = [i for i, e in enumerate(self.events) if e["type"] == kind
                 and all(e.get(k) == v for k, v in match.items())]
        return found[nth]

    def assertCaught(self, code, renumbered=True):
        events = renumber(self.events) if renumbered else self.events
        self.assertIn(code, codes(verify_events(events)))

    def test_original_is_clean(self):
        self.assertTrue(verify_events(self.events).ok)

    def test_transfer_without_its_arrival(self):
        del self.events[self.find("navigation_finished")]
        self.assertCaught("TRANSFER_WITHOUT_ARRIVAL")

    def test_transfer_after_a_failed_arrival(self):
        self.events[self.find("navigation_finished")]["outcome"] = "failed"
        self.assertCaught("TRANSFER_WITHOUT_ARRIVAL")

    def test_duplicated_transfer(self):
        i = self.find("cargo_pickup")
        self.events.insert(i+1, copy.deepcopy(self.events[i]))
        self.assertCaught("ILLEGAL_TRANSITION")

    def test_position_out_of_tolerance(self):
        self.events[self.find("cargo_drop")]["position_error"] = 0.5
        self.assertCaught("TRANSFER_OUT_OF_TOLERANCE")

    def test_facing_away_from_the_parcel(self):
        self.events[self.find("cargo_pickup")]["facing_error"] = 1.0
        self.assertCaught("TRANSFER_OUT_OF_TOLERANCE")

    def test_transfer_at_the_wrong_station(self):
        self.events[self.find("cargo_pickup")]["waypoint"] = "DROP_A"
        self.assertCaught("TRANSFER_WRONG_PLACE")

    def test_plan_that_forgets_a_stop(self):
        self.events[self.find("plan_selected")]["stops"].pop()
        self.assertCaught("PLAN_OBLIGATION_MISMATCH")

    def test_plan_that_drops_before_picking_up(self):
        stops = self.events[self.find("plan_selected", 1)]["stops"]
        pid = stops[0]["parcel"]
        pickup = next(i for i, s in enumerate(stops) if s == {**s, "parcel": pid, "kind": "pickup"})
        drop = next(i for i, s in enumerate(stops) if s == {**s, "parcel": pid, "kind": "drop"})
        stops[pickup], stops[drop] = stops[drop], stops[pickup]
        self.assertCaught("PLAN_PRECEDENCE")

    def test_dispatch_that_was_not_planned(self):
        i = self.find("navigation_started")
        self.events[i]["stop"] = {"parcel": "P3", "kind": "drop", "waypoint": "DROP_C"}
        self.assertCaught("DISPATCH_NOT_PLANNED")

    def test_two_goals_at_once(self):
        i = self.find("navigation_started")
        self.events.insert(i+1, copy.deepcopy(self.events[i]))
        self.assertCaught("CONCURRENT_GOALS")

    def test_supervisor_state_disagrees_with_events(self):
        self.events[self.find("mission_finished")]["state"]["parcels"]["P2"]["state"] = "onboard"
        self.assertCaught("STATE_MISMATCH")

    def test_wrong_final_result(self):
        self.events[self.find("mission_finished")]["result"] = "partial"
        self.assertCaught("RESULT_MISMATCH")

    def test_missing_event_leaves_a_sequence_gap(self):
        del self.events[self.find("plan_selected", 2)]
        self.assertCaught("SEQUENCE_GAP", renumbered=False)

    def test_deferral_without_a_retry(self):
        events = load(ROOT/"docs/evidence/maze/delivery.jsonl")
        # Turn the first failed-free arrival into a failure deferred immediately, skipping the retry policy.
        i = next(i for i, e in enumerate(events) if e["type"] == "cargo_pickup")
        events[i-1]["outcome"] = "failed"
        events[i] = {**events[i], "type": "parcel_deferred", "reason": "tampered"}
        self.events = events[:i+1] + [{**events[-1], "type": "episode_closed", "state": events[-1]["state"]}]
        self.assertCaught("RETRY_POLICY")

    def test_journal_without_a_start(self):
        del self.events[0]
        self.assertCaught("MISSING_START")

    def test_malformed_event_is_reported(self):
        del self.events[self.find("navigation_started")]["stop"]
        self.assertCaught("MALFORMED_EVENT")

    def test_never_crashes_on_missing_fields(self):
        """Deleting any single field from any event yields a report, never an exception."""
        for index, event in enumerate(load(MAZE)):
            for key in event:
                events = load(MAZE)
                del events[index][key]
                with self.subTest(event=index, field=key):
                    verify_events(events)


class RequestLifecycleTests(unittest.TestCase):
    """Every request gets an ID and exactly one recorded outcome; the checker proves it from the journal."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        world = World()
        self.backend = Backend(world)
        self.events = []
        self.mission = Mission(world, self.backend, Path(self.tmp.name)/"episode.jsonl", self.events.append)

    def tearDown(self):
        if self.mission.active:
            self.backend.result = "cancelled"
        if not self.mission.closed:
            self.mission.close()
        self.tmp.cleanup()

    def ask(self, operation, *parcels, error=None):
        request_id = self.mission.submit(f"{operation} {parcels}", "test")
        intent = {"operation": operation, "parcels": list(parcels), "message": ""}
        return request_id, self.mission.resolve(request_id, None if error else intent, error=error, model="test")

    def journal(self):
        if self.mission.active:
            self.backend.result = "cancelled"  # The double confirms the cancel close() requests.
        self.mission.close()
        return copy.deepcopy(self.events)

    def test_each_request_has_one_outcome(self):
        self.assertEqual(self.ask("create", "P1", "P1", "P2"), ("R1", "applied"))
        self.mission.tick()
        self.assertEqual(self.ask("status"), ("R2", "status"))
        self.assertEqual(self.ask("prioritize", "P9"), ("R3", "failed"))
        self.mission.command("resume")
        self.assertEqual(self.ask("cancel", "P2", error="model timed out"), ("R4", "failed"))
        rid = self.mission.submit("cancel P2", "test")
        self.mission.command("pause")
        self.assertEqual(self.mission.resolve(rid, {"operation": "cancel", "parcels": ["P2"]}), "superseded")
        with self.assertRaises(RuntimeError):
            self.mission.resolve(rid, {"operation": "status", "parcels": []})  # Already resolved.
        events = self.journal()
        report = verify_events(events)
        self.assertTrue(report.ok, report.violations)
        outcomes = {}
        for e in events:
            if e["type"] in {"instruction_applied", "status_reported", "clarification_required", "request_failed",
                             "request_superseded"} and str(e.get("request_id")).startswith("R"):
                outcomes.setdefault(e["request_id"], []).append(e["type"])
        self.assertEqual(outcomes, {"R1": ["instruction_applied"], "R2": ["status_reported"],
                                    "R3": ["request_failed"], "R4": ["request_failed"], "R5": ["request_superseded"]})
        applied = next(e for e in events if e["type"] == "instruction_applied" and e["request_id"] == "R1")
        self.assertEqual(applied["intent"]["parcels"], ["P1", "P2"])  # The normalized command, not raw output.

    def test_one_request_at_a_time(self):
        self.mission.submit("deliver P1", "test")
        with self.assertRaises(RuntimeError):
            self.mission.submit("deliver P2", "test")

    def tampered(self, change):
        self.ask("create", "P1")
        self.mission.tick()
        self.ask("status")
        events = self.journal()
        change(events)
        return codes(verify_events(renumber(events)))

    def test_checker_catches_a_second_outcome(self):
        def duplicate(events):
            i = next(i for i, e in enumerate(events) if e["type"] == "status_reported")
            events.insert(i+1, copy.deepcopy(events[i]))
        self.assertIn("DUPLICATE_OUTCOME", self.tampered(duplicate))

    def test_checker_catches_an_outcome_for_the_wrong_request(self):
        def mislabel(events):
            next(e for e in events if e["type"] == "status_reported")["request_id"] = "R1"
        self.assertIn("DUPLICATE_OUTCOME", self.tampered(mislabel))

    def test_checker_catches_acting_before_interpretation(self):
        def skip(events):
            del events[next(i for i, e in enumerate(events) if e["type"] == "language_interpreted")]
        self.assertIn("REQUEST_NOT_INTERPRETED", self.tampered(skip))

    def test_checker_catches_a_reused_request_id(self):
        def reuse(events):
            next(e for e in reversed(events) if e["type"] == "language_requested")["request_id"] = "R1"
        self.assertIn("DUPLICATE_REQUEST_ID", self.tampered(reuse))

    def test_checker_catches_a_raw_intent_in_the_journal(self):
        def raw(events):
            next(e for e in events if e["type"] == "instruction_applied")["intent"]["parcels"] = ["P1", "P1"]
        self.assertIn("NON_NORMALIZED_INTENT", self.tampered(raw))


class SnapshotRuleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        world = World()
        self.backend = Backend(world)
        self.mission = Mission(world, self.backend, Path(self.tmp.name)/"episode.jsonl")
        self.mission.apply({"operation": "create", "parcels": ["P1", "P2"]})
        self.mission.tick()
        self.snapshot = self.mission.snapshot()
        self.assertEqual(check_snapshot(self.snapshot), [])

    def tearDown(self):
        self.backend.result = "cancelled"
        self.mission.close()
        self.tmp.cleanup()

    def assertFlags(self, code, change):
        snapshot = copy.deepcopy(self.snapshot)
        change(snapshot)
        self.assertIn(code, {finding.code for finding in check_snapshot(snapshot)})

    def test_rules(self):
        active = self.snapshot["active"]["parcel"]
        other = next(pid for pid in ("P1", "P2") if pid != active)
        self.assertFlags("ILLEGAL_STATE", lambda s: s["parcels"]["P3"].update(state="onboard"))
        self.assertFlags("OBLIGATIONS_MISMATCH", lambda s: s["remaining_stops"].pop())
        self.assertFlags("OBLIGATIONS_MISMATCH", lambda s: s["parcels"][other].update(disposition="inactive"))
        self.assertFlags("ACTIVE_STOP_INCONSISTENT", lambda s: s["parcels"][active].update(state="delivered"))
        self.assertFlags("ACTIVE_STOP_INCONSISTENT", lambda s: s["parcels"][active].update(disposition="cancelled"))
        self.assertFlags("FINISHED_FLAG_WRONG", lambda s: s.update(finished=True))
        self.assertFlags("HOLD_MISMATCH", lambda s: s.update(hold={"kind": "no_route", "message": "No route"}))
        self.assertFlags("HOLD_MISMATCH", lambda s: s.update(paused=True))


if __name__ == "__main__":
    unittest.main()
