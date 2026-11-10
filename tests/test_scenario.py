"""Scenario files, fault triggers and outcome classification, independent of ROS."""

import json
import tempfile
import unittest
from pathlib import Path

from warehouse_agent.scenario import classify, load_scenario, read_events, trigger_time
from warehouse_agent.world import World

ROOT = Path(__file__).resolve().parent.parent
WAYPOINTS = World().waypoints
BASE = {"name": "case", "description": "A test case.", "command": "Deliver all three parcels", "timeout_s": 60}
FAULT = {"trigger": {"event": "cargo_pickup"}, "fault": {"type": "blocked_path", "at": "PICK_B"},
         "ground_truth": {"category": "blocked_station"}}


def parcel(state="awaiting_pickup", disposition="active"):
    return {"pickup": "PICK_A", "drop": "DROP_A", "state": state, "disposition": disposition}


def event(kind, at, **data):
    return {"type": kind, "monotonic": float(at), **data}


class ScenarioFiles(unittest.TestCase):
    def load(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scenario.json"
            path.write_text(json.dumps(data))
            return load_scenario(path, WAYPOINTS)

    def test_every_shipped_scenario_loads(self):
        files = sorted((ROOT / "scenarios").glob("*.json"))
        self.assertTrue(files)
        for path in files:
            with self.subTest(path.name):
                scenario = load_scenario(path, WAYPOINTS)
                self.assertEqual(scenario.name, path.stem)

    def test_a_control_run_has_no_fault(self):
        scenario = self.load(BASE)
        self.assertIsNone(scenario.fault)
        self.assertIsNone(scenario.trigger)

    def test_rejected_files(self):
        cases = {
            "unknown key": {**BASE, "extra": 1},
            "empty command": {**BASE, "command": " "},
            "bad timeout": {**BASE, "timeout_s": 0},
            "fault without trigger": {**BASE, "fault": FAULT["fault"], "ground_truth": FAULT["ground_truth"]},
            "trigger without fault": {**BASE, "trigger": FAULT["trigger"]},
            "unknown fault type": {**BASE, **FAULT, "fault": {"type": "gremlins"}},
            "unknown waypoint": {**BASE, **FAULT, "fault": {"type": "blocked_path", "at": "PICK_Z"}},
            "position and waypoint": {**BASE, **FAULT,
                                      "fault": {"type": "blocked_path", "at": "PICK_B", "position": [0, 0]}},
            "bad size": {**BASE, **FAULT, "fault": {"type": "blocked_path", "at": "PICK_B", "size": [1, 1, 0]}},
            "uncrashable node": {**BASE, **FAULT, "fault": {"type": "node_crash", "node": "gazebo"}},
            "bad dropout mode": {**BASE, **FAULT, "fault": {"type": "sensor_dropout", "mode": "melt"}},
            "zero pose offset": {**BASE, **FAULT, "fault": {"type": "localization_jump", "offset": [0, 0, 0]}},
            "parameter without a value": {**BASE, **FAULT, "fault": {"type": "bad_parameter",
                                                                     "node": "/velocity_smoother",
                                                                     "name": "max_velocity"}},
            "parameter node without a slash": {**BASE, **FAULT, "fault": {"type": "bad_parameter", "value": 0,
                                                                          "node": "velocity_smoother",
                                                                          "name": "max_velocity"}},
            "unknown trigger event": {**BASE, **FAULT, "trigger": {"event": "robot_exploded"}},
            "bad trigger count": {**BASE, **FAULT, "trigger": {"event": "cargo_pickup", "count": 0}},
            "negative delay": {**BASE, **FAULT, "trigger": {"event": "cargo_pickup", "delay_s": -1}},
            "no ground truth": {**BASE, "trigger": FAULT["trigger"], "fault": FAULT["fault"]},
        }
        for name, data in cases.items():
            with self.subTest(name), self.assertRaises(ValueError):
                self.load(data)

    def test_a_fault_may_name_a_position_instead_of_a_waypoint(self):
        scenario = self.load({**BASE, **FAULT, "fault": {"type": "blocked_path", "position": [1.0, -2.5]}})
        self.assertEqual(scenario.fault["position"], [1.0, -2.5])


class Triggers(unittest.TestCase):
    EVENTS = [event("navigation_started", 10, stop={"parcel": "P1", "kind": "pickup"}),
              event("cargo_pickup", 20, parcel="P1"),
              event("navigation_started", 20, stop={"parcel": "P2", "kind": "pickup"}),
              event("cargo_pickup", 50, parcel="P2")]

    def test_not_due_before_the_event_has_happened(self):
        self.assertIsNone(trigger_time({"event": "cargo_drop"}, self.EVENTS))
        self.assertIsNone(trigger_time({"event": "cargo_pickup", "count": 3}, self.EVENTS))

    def test_due_a_delay_after_the_counted_event(self):
        self.assertEqual(trigger_time({"event": "cargo_pickup"}, self.EVENTS), 20)
        self.assertEqual(trigger_time({"event": "cargo_pickup", "count": 2, "delay_s": 1.5}, self.EVENTS), 51.5)

    def test_where_filters_on_dotted_keys(self):
        trigger = {"event": "navigation_started", "where": {"stop.parcel": "P2"}, "delay_s": 3}
        self.assertEqual(trigger_time(trigger, self.EVENTS), 23)
        self.assertIsNone(trigger_time({"event": "navigation_started", "where": {"stop.parcel": "P9"}}, self.EVENTS))

    def test_a_partly_written_last_line_is_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "journal.jsonl"
            path.write_text(json.dumps(self.EVENTS[0]) + "\n" + json.dumps(self.EVENTS[1])[:20])
            self.assertEqual(read_events(path), [self.EVENTS[0]])
            self.assertEqual(read_events(Path(directory) / "missing.jsonl"), [])


class Classification(unittest.TestCase):
    def test_the_recorded_delivery_is_complete(self):
        outcome = classify(read_events(ROOT / "docs/evidence/maze/delivery.jsonl"))
        self.assertEqual(outcome["result"], "complete")
        self.assertEqual(outcome["delivered"], ["P1", "P2", "P3"])
        self.assertEqual(outcome["deferred"], [])
        self.assertEqual(outcome["retries"], 0)
        self.assertGreater(outcome["mission_seconds"], 0)

    def test_a_deferred_parcel_makes_the_mission_partial(self):
        state = {"parcels": {"P1": parcel("delivered"), "P2": parcel(disposition="deferred")}}
        events = [event("instruction_applied", 0, state={"parcels": {"P1": parcel(), "P2": parcel()}}),
                  event("navigation_finished", 40, outcome="failed", cancellation_requested=False),
                  event("navigation_retry", 40),
                  event("navigation_finished", 80, outcome="failed", cancellation_requested=False),
                  event("parcel_deferred", 80, parcel="P2", reason="Navigation failed"),
                  event("navigation_finished", 120, outcome="succeeded", cancellation_requested=False),
                  event("mission_finished", 120, result="partial", state=state)]
        outcome = classify(events, injected_at=10)
        self.assertEqual(outcome["result"], "partial")
        self.assertEqual(outcome["delivered"], ["P1"])
        self.assertEqual(outcome["deferred"], ["P2"])
        self.assertEqual(outcome["deferral_reasons"], {"P2": "Navigation failed"})
        self.assertEqual(outcome["navigation_outcomes"], {"failed": 2, "succeeded": 1})
        self.assertEqual(outcome["retries"], 1)
        self.assertEqual(outcome["mission_seconds"], 120.0)
        self.assertEqual(outcome["seconds_to_first_abnormal_event"], 30.0)
        self.assertEqual(outcome["first_abnormal_event"], "navigation_finished")

    def test_a_supervisor_error_is_reported_with_its_message_and_stranded_cargo(self):
        events = [event("instruction_applied", 0, state={"parcels": {"P1": parcel()}}),
                  event("cargo_pickup", 15, parcel="P1"),
                  event("episode_closed", 60, state={"parcels": {"P1": parcel("onboard")}})]
        outcome = classify(events, "Interpreting R1 locally…\nerror: Nav2 did not acknowledge the goal\n")
        self.assertEqual(outcome["result"], "supervisor_error")
        self.assertEqual(outcome["supervisor_error"], "Nav2 did not acknowledge the goal")
        self.assertEqual(outcome["stranded_onboard"], ["P1"])

    def test_timeout_no_route_and_not_started(self):
        started = [event("instruction_applied", 0, state={"parcels": {"P1": parcel()}})]
        self.assertEqual(classify(started, "Demo timed out; cancelling navigation.")["result"], "timeout")
        self.assertEqual(classify(started, "Mission held: no route from the current pose; exiting.")["result"],
                         "no_route")
        self.assertEqual(classify(started)["result"], "incomplete")
        self.assertEqual(classify([])["result"], "not_started")

    def test_a_cancelled_goal_is_not_an_abnormal_event(self):
        events = [event("instruction_applied", 0, state={"parcels": {}}),
                  event("navigation_finished", 30, outcome="cancelled", cancellation_requested=True)]
        self.assertIsNone(classify(events, injected_at=5)["seconds_to_first_abnormal_event"])


if __name__ == "__main__":
    unittest.main()
