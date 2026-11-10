"""Diagnostic checks, the rule-based baseline, the agent loop and scoring, on synthetic runs (no ROS, no model)."""

import json
import tempfile
import unittest
from pathlib import Path

from warehouse_agent import diagnosis
from warehouse_agent.diagnosis import Incident, diagnose, health_report, rule_based, score
from warehouse_agent.evidence import _buckets, _clusters, compose, process_events

WAYPOINTS = {"HOME": [-7.2, -5.6, 0.0], "PICK_B": [1.5, -0.9, 1.57], "DROP_B": [7.1, 0.0, 0.0]}
END = 100.0


def healthy_evidence():
    """A run in which nothing went wrong: steady heartbeats and scans, motion that follows its commands."""
    return {
        "meta": {"recording_start": -3.0, "recording_end": END, "bucket_s": 5.0, "message_counts": {},
                 "costmap_snapshots": 50},
        "rates": {"/scan_nav": {"start": -5.0, "counts": [10] + [25]*20}},
        "logs": [],
        "bonds": {name: {"first": -2.0, "last": END, "silent_for_s": 0.0, "last_active_flag": True, "restarts": 0}
                  for name in ("amcl", "controller_server", "planner_server", "bt_navigator")},
        "behaviour_tree": {"failure_counts": {}, "failures": []},
        "parameter_changes": [], "pose_resets": [], "processes_died": [], "localization_jumps": [],
        "motion": [[second, 0.26, 0.0, 0.26, 0.0] for second in range(0, int(END))],
        "pose": [], "map_to_odom": [], "amcl": [[10.0, 0.0, 0.0, 0.0, 0.05]], "plans": [],
        "lidar": {"/scan_nav": [[t, 25, 1.0, 0.8] for t in range(0, int(END), 5)]},
        "new_obstacles": [],
    }


def journal(goals):
    """Journal events for navigation goals given as (start, end, waypoint, outcome)."""
    events = [{"type": "episode_started", "monotonic": 0.0, "config": {"waypoints": WAYPOINTS}},
              {"type": "language_requested", "monotonic": 0.0, "text": "Deliver P2"},
              {"type": "instruction_applied", "monotonic": 0.0, "state": {"parcels": {}}}]
    for start, end, waypoint, outcome in goals:
        stop = {"parcel": "P2", "kind": "pickup" if waypoint.startswith("PICK") else "drop", "waypoint": waypoint}
        events.append({"type": "navigation_started", "monotonic": float(start), "stop": stop})
        if outcome:
            events.append({"type": "navigation_finished", "monotonic": float(end), "stop": stop,
                           "outcome": outcome, "cancellation_requested": False})
    complete = all(outcome == "succeeded" for *_, outcome in goals)
    state = {"parcels": {"P2": {"state": "delivered" if complete else "awaiting_pickup",
                                "disposition": "active" if complete else "deferred"}}}
    events.append({"type": "mission_finished", "monotonic": float(goals[-1][1]),
                   "result": "complete" if complete else "partial", "state": state})
    return events


class Case(unittest.TestCase):
    def incident(self, evidence=None, goals=None, truth=None):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        run = Path(directory.name)
        (run / "evidence.json").write_text(json.dumps(evidence or healthy_evidence()))
        goals = goals or [(0, 30, "PICK_B", "succeeded"), (30, 60, "DROP_B", "succeeded")]
        (run / "journal.jsonl").write_text("\n".join(json.dumps(e) for e in journal(goals)) + "\n")
        if truth:
            (run / "result.json").write_text(json.dumps({"fault": {"injected": True},
                                                         "scenario": {"ground_truth": truth}}))
        return Incident(run)


FAILED = [(0, 90, "PICK_B", "failed")]


class Checks(Case):
    def test_a_healthy_run_has_nothing_unusual_and_is_no_fault(self):
        incident = self.incident()
        report = health_report(incident)
        self.assertEqual(report["mission"], "all navigation goals succeeded")
        self.assertTrue(all(value == "nothing unusual" for key, value in report.items() if key != "mission"))
        self.assertEqual(rule_based(incident)["diagnosis"]["category"], "no_fault")
        self.assertEqual(incident.truth, {"category": "no_fault"})

    def test_a_dead_process_and_a_silent_heartbeat_are_reported(self):
        evidence = healthy_evidence()
        evidence["bonds"]["controller_server"].update(last=19.3, silent_for_s=END - 19.3)
        evidence["processes_died"] = [{"t": 19.4, "process": "controller_server", "exit_code": -9}]
        incident = self.incident(evidence, FAILED)
        unusual = diagnosis.node_health(incident)["unusual"]
        self.assertIn("controller_server process died at about 19.4 s (exit code -9)", unusual)
        self.assertTrue(any("controller_server heartbeat stopped at 19.3 s" in text for text in unusual))
        answer = rule_based(incident)["diagnosis"]
        self.assertEqual((answer["category"], answer["component"]), ("node_crash", "controller_server"))

    def test_when_every_heartbeat_stops_the_first_to_stop_is_named(self):
        evidence = healthy_evidence()
        for name, bond in evidence["bonds"].items():
            bond.update(last=19.3 if name == "controller_server" else 23.4, silent_for_s=70.0)
        unusual = diagnosis.node_health(self.incident(evidence, FAILED))["unusual"]
        self.assertEqual(unusual, ["all 4 heartbeats stopped; the first to stop was controller_server at 19.3 s"])

    def test_scans_that_stop_are_reported_with_the_time(self):
        evidence = healthy_evidence()
        evidence["rates"]["/scan_nav"]["counts"] = [10] + [25]*5 + [0]*15
        incident = self.incident(evidence, FAILED)
        self.assertEqual(diagnosis.lidar_status(incident)["unusual"],
                         ["scans stopped arriving at about 25.0 s and never resumed (normal rate 5.0 Hz)"])
        self.assertEqual(rule_based(incident)["diagnosis"]["category"], "sensor_dropout")

    def test_a_thin_first_interval_before_the_mission_is_start_up_not_a_dropout(self):
        evidence = healthy_evidence()
        evidence["rates"]["/scan_nav"]["counts"] = [3] + [25]*20
        self.assertEqual(diagnosis.lidar_status(self.incident(evidence))["unusual"], [])

    def test_a_short_last_interval_is_not_mistaken_for_a_dropout(self):
        evidence = healthy_evidence()
        evidence["meta"]["recording_end"] = 97.0   # the last 5 s interval is only 2 s long
        evidence["rates"]["/scan_nav"]["counts"] = [10] + [25]*19 + [10]
        self.assertEqual(diagnosis.lidar_status(self.incident(evidence))["unusual"], [])

    def test_standing_still_without_commands_during_a_goal_is_reported(self):
        evidence = healthy_evidence()
        for row in evidence["motion"][22:]:
            row[1:] = [0.0, 0.0, None, None]
        unusual = diagnosis.motion_status(self.incident(evidence, FAILED))["unusual"]
        self.assertEqual(len(unusual), 1)
        self.assertIn("from 22 s (goal: pickup P2) the robot stood still and received no non-zero", unusual[0])

    def test_commands_without_motion_point_to_the_drive(self):
        evidence = healthy_evidence()
        for row in evidence["motion"][40:]:
            row[1:] = [0.0, 0.0, 0.26, 0.0]
        incident = self.incident(evidence, FAILED)
        self.assertIn("velocity commands were sent but the robot did not move",
                      diagnosis.motion_status(incident)["unusual"][0])
        self.assertEqual(rule_based(incident)["diagnosis"]["category"], "actuator_fault")

    def test_a_pose_jump_and_reset_are_reported(self):
        evidence = healthy_evidence()
        evidence["localization_jumps"] = [{"t": 22.5, "shift_m": 2.51, "turn_rad": 0.53}]
        evidence["pose_resets"] = [{"t": 22.3, "x": -4.67, "y": -3.68, "yaw": -0.48}]
        incident = self.incident(evidence, FAILED)
        unusual = diagnosis.localization_status(incident)["unusual"]
        self.assertIn("the pose estimate jumped 2.51 m and 0.53 rad in one update at 22.5 s", unusual)
        self.assertTrue(any("/initialpose" in text for text in unusual))
        self.assertEqual(rule_based(incident)["diagnosis"]["category"], "localization_lost")

    def test_small_corrections_alone_are_not_a_lost_pose(self):
        evidence = healthy_evidence()
        evidence["localization_jumps"] = [{"t": 40.0, "shift_m": 0.35, "turn_rad": 0.02}]
        incident = self.incident(evidence)
        self.assertEqual(diagnosis.localization_status(incident)["unusual"], [])
        self.assertEqual(rule_based(incident)["diagnosis"]["category"], "no_fault")

    def test_a_runtime_parameter_change_is_reported_but_harness_nodes_are_hidden(self):
        evidence = healthy_evidence()
        evidence["parameter_changes"] = [
            {"t": 21.0, "node": "/scan_relay", "changed": [{"name": "mode", "value": "drop"}]},
            {"t": 21.2, "node": "/velocity_smoother", "changed": [{"name": "max_velocity", "value": [0.0, 0.0, 0.0]}]}]
        incident = self.incident(evidence, FAILED)
        self.assertEqual(diagnosis.parameter_changes(incident)["unusual"],
                         ["/velocity_smoother parameter max_velocity was set to [0.0, 0.0, 0.0] at 21.2 s"])
        answer = rule_based(incident)["diagnosis"]
        self.assertEqual((answer["category"], answer["component"]),
                         ("bad_parameter", "/velocity_smoother max_velocity"))

    def test_an_obstacle_on_a_failed_goals_stopping_pose_is_a_blocked_station(self):
        evidence = healthy_evidence()
        evidence["new_obstacles"] = [{"x": 1.5, "y": -0.85, "extent_m": 1.0, "first_seen": 45.0, "last_seen": 99.0,
                                      "snapshots": 30}]
        incident = self.incident(evidence, FAILED)
        unusual = diagnosis.obstacles(incident)["unusual"]
        self.assertIn("on the PICK_B stopping pose (a goal to that waypoint failed)", unusual[0])
        answer = rule_based(incident)["diagnosis"]
        self.assertEqual((answer["category"], answer["component"]), ("blocked_station", "PICK_B"))

    def test_an_obstacle_elsewhere_is_a_blocked_path_and_a_brief_one_is_ignored(self):
        evidence = healthy_evidence()
        evidence["new_obstacles"] = [
            {"x": 4.5, "y": -2.6, "extent_m": 1.0, "first_seen": 45.0, "last_seen": 99.0, "snapshots": 30},
            {"x": 0.0, "y": 0.0, "extent_m": 0.5, "first_seen": 10.0, "last_seen": 12.0, "snapshots": 2}]
        incident = self.incident(evidence, FAILED)
        found = diagnosis.obstacles(incident)
        self.assertEqual(len(found["new_obstacles"]), 1)
        self.assertIn("m from", found["unusual"][0])
        self.assertEqual(rule_based(incident)["diagnosis"]["category"], "blocked_path")

    def test_wall_sized_structures_are_described_as_misaligned_scans_not_obstacles(self):
        evidence = healthy_evidence()
        evidence["new_obstacles"] = [{"x": -2.4, "y": 0.9, "extent_m": 5.0, "first_seen": 30.0, "last_seen": 99.0,
                                      "snapshots": 60}]
        unusual = diagnosis.obstacles(self.incident(evidence, FAILED))["unusual"]
        self.assertEqual(len(unusual), 1)
        self.assertIn("scans are not lining up with the map", unusual[0])

    def test_warnings_are_grouped_and_startup_noise_is_dropped(self):
        evidence = healthy_evidence()
        evidence["logs"] = [
            {"t": 0.0, "t_last": 0.0, "level": "WARN", "node": "controller_server", "text": "No goal checker", "count": 1},
            {"t": 30.8, "t_last": 40.0, "level": "ERROR", "node": "controller_server",
             "text": "Failed to make progress", "count": 5},
            {"t": 60.0, "t_last": 61.0, "level": "ERROR", "node": "controller_server",
             "text": "Failed to make progress", "count": 2}]
        warnings = diagnosis.nav2_warnings(self.incident(evidence, FAILED))
        self.assertEqual(warnings["distinct_messages"], 1)
        self.assertEqual(warnings["messages"][0]["count"], 7)
        self.assertEqual(warnings["messages"][0]["first_s"], 30.8)


class Agent(Case):
    def test_the_agent_runs_the_tools_the_model_asks_for_then_takes_a_structured_answer(self):
        evidence = healthy_evidence()
        evidence["parameter_changes"] = [
            {"t": 21.2, "node": "/velocity_smoother", "changed": [{"name": "max_velocity", "value": [0.0, 0.0, 0.0]}]}]
        incident = self.incident(evidence, FAILED)
        final = {"category": "bad_parameter", "component": "velocity_smoother max_velocity",
                 "root_cause": "The speed limit was set to zero at 21.2 s.", "evidence": ["x"], "confidence": "high"}
        seen = []

        def ask(endpoint, payload):
            seen.append(payload)
            if "format" in payload:
                return {"message": {"content": json.dumps(final)}, "eval_count": 40}
            if len(seen) == 1:
                return {"message": {"role": "assistant", "content": "", "tool_calls": [
                    {"function": {"name": "health_report", "arguments": {}}},
                    {"function": {"name": "not_a_tool", "arguments": {}}}]}, "eval_count": 10}
            return {"message": {"role": "assistant", "content": "I have enough."}, "eval_count": 5}

        outcome = diagnose(incident, ask=ask)
        self.assertEqual(outcome["diagnosis"], final)
        self.assertEqual(outcome["tool_calls"], ["health_report", "not_a_tool"])
        self.assertEqual(outcome["output_tokens"], 55)
        tool_messages = [m for m in seen[1]["messages"] if m["role"] == "tool"]
        self.assertIn("max_velocity was set to", tool_messages[0]["content"])
        self.assertIn("no tool named not_a_tool", tool_messages[1]["content"])
        self.assertEqual(seen[-1]["format"]["properties"]["category"]["enum"], list(diagnosis.CATEGORIES))

    def test_without_tools_the_model_is_asked_once_and_offered_none(self):
        seen = []

        def ask(endpoint, payload):
            seen.append(payload)
            return {"message": {"content": "not json"}}

        outcome = diagnose(self.incident(), use_tools=False, ask=ask)
        self.assertEqual(len(seen), 1)
        self.assertNotIn("tools", seen[0])
        self.assertNotIn("health_report first", seen[0]["messages"][0]["content"])
        self.assertEqual(outcome["diagnosis"]["category"], "unknown")   # an unusable answer is never a guess


class Scoring(unittest.TestCase):
    TRUTH = {"category": "node_crash", "node": "controller_server"}

    def answer(self, category, component="", cause=""):
        return {"category": category, "component": component, "root_cause": cause, "evidence": []}

    def test_category_and_specific_cause_are_scored_separately(self):
        right = score(self.answer("node_crash", "controller_server"), self.TRUTH)
        self.assertEqual((right["category_correct"], right["named_the_specific_cause"]), (True, True))
        vague = score(self.answer("node_crash", "a Nav2 node"), self.TRUTH)
        self.assertEqual((vague["category_correct"], vague["named_the_specific_cause"]), (True, False))

    def test_a_wrong_category_never_gets_credit_for_the_specific_cause(self):
        wrong = score(self.answer("bad_parameter", "controller_server"), self.TRUTH)
        self.assertEqual((wrong["category_correct"], wrong["named_the_specific_cause"]), (False, False))

    def test_no_fault_has_no_specific_cause_to_name(self):
        self.assertIsNone(score(self.answer("no_fault"), {"category": "no_fault"})["named_the_specific_cause"])
        self.assertFalse(score(self.answer("node_crash"), {"category": "no_fault"})["category_correct"])

    def test_every_listed_specific_must_be_named(self):
        truth = {"category": "bad_parameter", "node": "velocity_smoother", "parameter": "max_velocity"}
        self.assertTrue(score(self.answer("bad_parameter", "/velocity_smoother", "max_velocity is zero"),
                              truth)["named_the_specific_cause"])
        self.assertFalse(score(self.answer("bad_parameter", "/velocity_smoother"), truth)["named_the_specific_cause"])


class Scoreboard(unittest.TestCase):
    def test_saved_diagnoses_are_tabulated_by_scenario_and_diagnoser(self):
        with tempfile.TemporaryDirectory() as directory:
            first, second = Path(directory, "node_crash", "run1"), Path(directory, "nominal", "run1")
            for run in (first, second):
                run.mkdir(parents=True)
            good = {"category_correct": True, "named_the_specific_cause": True}
            vague = {"category_correct": True, "named_the_specific_cause": False}
            clean = {"category_correct": True, "named_the_specific_cause": None}
            wrong = {"category_correct": False, "named_the_specific_cause": None}
            for run, name, label, marks in ((first, "diagnosis-rules-1.json", "rules", good),
                                            (first, "diagnosis-m-1.json", "m", vague),
                                            (second, "diagnosis-rules-1.json", "rules", clean),
                                            (second, "diagnosis-m-1.json", "m", wrong)):
                (run / name).write_text(json.dumps({"model": label, "label": label, "score": marks}))
            table = diagnosis.scoreboard([first, second]).splitlines()
        self.assertEqual(table[0], "| Scenario | m | rules |")
        self.assertEqual(table[2], "| node_crash | 1/1 (0/1 specific) | 1/1 (1/1 specific) |")
        self.assertEqual(table[3], "| nominal | 0/1 | 1/1 |")
        self.assertEqual(table[4], "| **all** | 1/2 (0/1 specific) | 2/2 (1/1 specific) |")


class EvidenceHelpers(unittest.TestCase):
    def test_compose_applies_a_planar_transform(self):
        x, y, yaw = compose((1.0, 2.0, 1.5707963267948966), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 3.0)
        self.assertAlmostEqual(yaw, 1.5707963267948966)

    def test_buckets_include_silent_intervals_up_to_the_end(self):
        self.assertEqual(_buckets({0: 25, 1: 25}, end=22.0), {"start": 0.0, "counts": [25, 25, 0, 0, 0]})
        self.assertEqual(_buckets({}, end=10.0), {"start": 0, "counts": []})

    def test_neighbouring_bins_merge_into_one_obstacle(self):
        bins = {(3, -2): {"first": 20.0, "last": 90.0, "snapshots": 30, "cells": 40},
                (3, -1): {"first": 22.0, "last": 88.0, "snapshots": 28, "cells": 40},
                (20, 20): {"first": 50.0, "last": 52.0, "snapshots": 2, "cells": 5}}
        clusters = _clusters(bins)
        self.assertEqual(len(clusters), 2)
        self.assertEqual((clusters[0]["x"], clusters[0]["y"], clusters[0]["extent_m"]), (1.75, -0.5, 1.0))
        self.assertEqual((clusters[0]["first_seen"], clusters[0]["last_seen"]), (20.0, 90.0))

    def test_a_heartbeat_ends_when_the_first_of_its_two_ends_goes_quiet(self):
        from warehouse_agent.evidence import _bond
        bond = {"first": -2.0, "active": True, "instances": {"node-side": 19.4, "manager-side": 23.3}}
        self.assertEqual(_bond(bond, end=30.0), {"first": -2.0, "last": 19.4, "silent_for_s": 10.6,
                                                 "last_active_flag": True, "restarts": 0})
        bond["instances"]["node-after-restart"] = 30.0
        self.assertEqual(_bond(bond, end=30.0)["restarts"], 1)

    def test_process_deaths_are_read_from_the_launch_log(self):
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "simulation.log").write_text(
                "[amcl-7] [INFO] [1000000010.500000000] [amcl]: ready\n"
                "[ERROR] [controller_server-9]: process has died [pid 650884, exit code -9, cmd '/opt/x']\n")
            self.assertEqual(process_events(directory, origin=1000000000.0),
                             [{"t": 10.5, "process": "controller_server", "exit_code": -9}])
            self.assertEqual(process_events(directory, origin=1000000100.0), [])   # before this mission started


if __name__ == "__main__":
    unittest.main()
