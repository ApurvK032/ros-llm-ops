"""Judging single runs and summarizing a batch, on synthetic result files (no ROS, no model)."""

import json
import tempfile
import unittest
from pathlib import Path

from warehouse_agent.batch import collect, judge, normalize, plan, summarize, table


def action(name, target="", permitted=True, error=None):
    entry = {"action": name, "target": target, "permitted": permitted, "why": ""}
    if error:
        entry["error"] = error
    elif permitted:
        entry["result"] = {}
    return entry


def result(scenario="lidar_dropout", expected=None, delivered=("P1", "P2", "P3"), verified=None, mode=None,
           incidents=(), injected=True, additional=(), onboard=(), violations=(), seconds=140.0, resume=12.0):
    """A result file as the scenario runner writes it, reduced to what judging reads."""
    fault = {"type": "sensor_dropout"}
    recovery = None
    if mode:
        actions = [a for incident in incidents for a in incident]
        recovery = {"mode": mode, "seconds_fault_to_resume": resume,
                    "incidents": [{"seconds": 6.0, "diagnosis": {"category": "sensor_dropout"}, "actions": list(i)}
                                  for i in incidents],
                    "actions_refused": sum(not a["permitted"] for a in actions),
                    "actions_failed": sum("error" in a for a in actions)}
    return {
        "scenario": {"name": scenario, "fault": fault, "additional": [{"fault": fault}]*len(additional),
                     "expected": expected or {"delivered": 3, "actions": [["restart_process", "lidar_driver"]]}},
        "run": {"fault_requested": True},
        "fault": {"injected": injected}, "additional_faults": [{"injected": done} for done in additional],
        "outcome": {"result": "complete", "delivered": list(delivered), "stranded_onboard": list(onboard),
                    "mission_seconds": seconds},
        "pose_check": {"samples": 900, "delivered_verified": list(delivered if verified is None else verified)},
        "recovery": recovery,
        "checker": {"ok": not violations, "violations": list(violations)},
    }


class Judging(unittest.TestCase):
    def test_the_right_action_and_every_parcel_confirmed_is_a_success(self):
        got = judge(result(mode="rules", incidents=[[action("restart_process", "lidar")]]))
        self.assertTrue(got["valid"] and got["success"])
        self.assertEqual((got["decider"], got["delivered"], got["incidents"]), ("rules", 3, 1))
        self.assertEqual((got["unneeded"], got["missing"]), ([], []))
        self.assertEqual(got["carried_out"], [["restart_process", "lidar_driver"]])

    def test_a_delivery_the_true_pose_does_not_confirm_is_not_counted(self):
        got = judge(result(mode="rules", verified=["P1", "P2"], incidents=[[action("restart_process", "lidar")]]))
        self.assertFalse(got["success"])
        self.assertEqual((got["delivered"], got["delivered_claimed"]), (2, 3))

    def test_without_a_pose_recording_the_journal_is_used_and_the_run_is_marked(self):
        saved = result()
        saved["pose_check"] = {"samples": 0, "delivered_verified": []}
        got = judge(saved)
        self.assertEqual((got["delivered"], got["pose_checked"], got["success"]), (3, False, True))

    def test_actions_beyond_what_the_scenario_needs_are_listed(self):
        got = judge(result(mode="qwen3.5:9b", incidents=[
            [action("restore_parameter", "velocity_smoother max_velocity"), action("clear_costmaps"),
             action("escalate", "tell someone")],
            [action("restart_process", "scan_relay")]]))
        self.assertTrue(got["success"])
        self.assertEqual(got["unneeded"], [["restore_parameter", "/velocity_smoother max_velocity"]])
        self.assertEqual(got["incidents"], 2)
        self.assertEqual(got["held_seconds"], 12.0)

    def test_refused_and_failed_actions_were_not_carried_out(self):
        got = judge(result(mode="qwen3.5:4b", delivered=[], incidents=[
            [action("defer_parcel", "P1", permitted=False), action("restart_process", "lidar_driver", error="x")]]))
        self.assertEqual((got["carried_out"], got["unneeded"]), ([], []))
        self.assertEqual(got["missing"], [["restart_process", "lidar_driver"]])
        self.assertEqual((got["refused"], got["failed"], got["success"]), (1, 1, False))

    def test_an_action_the_scenario_calls_acceptable_is_neither_needed_nor_unneeded(self):
        expected = {"delivered": 3, "actions": [["restart_process", "amcl"]], "acceptable": [["relocalize", ""]]}
        got = judge(result(expected=expected, mode="rules",
                           incidents=[[action("restart_process", "amcl")], [action("relocalize", "now")]]))
        self.assertEqual((got["unneeded"], got["missing"], got["success"]), ([], [], True))

    def test_a_run_whose_fault_never_landed_is_invalid_not_a_success(self):
        self.assertEqual(judge(result(injected=False))["valid"], False)
        self.assertIsNone(judge(result(injected=False))["success"])
        self.assertFalse(judge(result(additional=[False]))["valid"])
        self.assertTrue(judge(result(additional=[True]))["valid"])

    def test_a_stranded_parcel_or_a_checker_violation_is_a_failure(self):
        self.assertFalse(judge(result(delivered=["P1", "P2"], onboard=["P3"],
                                      expected={"delivered": 2, "actions": []}))["success"])
        self.assertFalse(judge(result(violations=[{"code": "DISPATCH_WHILE_PAUSED"}]))["success"])

    def test_a_fault_free_run_alone_is_judged_on_deliveries(self):
        saved = result(scenario="nominal", expected={"delivered": 3, "actions": []})
        saved["scenario"]["fault"] = None
        saved["fault"] = {"injected": False}
        got = judge(saved)
        self.assertEqual((got["valid"], got["success"], got["decider"]), (True, True, "none"))

    def test_targets_are_compared_in_one_spelling(self):
        self.assertEqual(normalize("restart_process", " scan_relay "), ("restart_process", "lidar_driver"))
        self.assertEqual(normalize("restart_process", "planner_server"), ("restart_process", "planner_server"))
        self.assertEqual(normalize("restore_parameter", "velocity_smoother  max_velocity"),
                         ("restore_parameter", "/velocity_smoother max_velocity"))
        self.assertEqual(normalize("relocalize", "the robot"), ("relocalize", ""))
        self.assertEqual(normalize("defer_parcel", "P2"), ("defer_parcel", "P2"))


class Summary(unittest.TestCase):
    def runs(self):
        good = [action("restart_process", "lidar")]
        return [
            judge(result(seconds=462.0, delivered=[])),
            judge(result(mode="rules", incidents=[good], seconds=130.0, resume=9.0)),
            judge(result(mode="rules", incidents=[good], seconds=140.0, resume=11.0)),
            judge(result(mode="rules", incidents=[good], seconds=90.0, delivered=["P1"], resume=None)),
            judge(result(mode="rules", injected=False)),
            judge(result(mode="qwen3.5:9b", seconds=150.0, resume=20.0,
                         incidents=[[action("restore_parameter", "/a b")], good])),
        ]

    def test_rates_count_valid_runs_and_times_come_from_the_runs_that_worked(self):
        rows = {(row["scenario"], row["decider"]): row for row in summarize(self.runs())}
        rules = rows[("lidar_dropout", "rules")]
        self.assertEqual((rules["runs"], rules["invalid"], rules["successes"]), (3, 1, 2))
        self.assertEqual((rules["median_mission_seconds"], rules["median_fault_to_resume"]), (135.0, 10.0))
        self.assertEqual(rules["delivered"], [3, 3, 1])
        model = rows[("lidar_dropout", "qwen3.5:9b")]
        self.assertEqual((model["interruptions"], model["unneeded_actions"], model["runs_with_unneeded_actions"]),
                         (2, 1, 1))
        alone = rows[("lidar_dropout", "none")]
        self.assertEqual((alone["successes"], alone["median_mission_seconds"]), (0, None))

    def test_the_table_has_a_row_per_scenario_a_column_per_decider_and_totals(self):
        text = table(summarize(self.runs()), ["none", "rules", "qwen3.5:9b"])
        self.assertIn("| Scenario | Supervisor alone | Playbook | qwen3.5:9b |", text)
        self.assertIn("| `lidar_dropout` | 0 of 1 | 2 of 3, 135 s | 1 of 1, 150 s |", text)
        self.assertIn("| **all** | **0 of 1** | **2 of 3** | **1 of 1** |", text)
        self.assertIn("| `lidar_dropout` | 1.0 per run; 0 unneeded in 0 of 3 | 2.0 per run; 1 unneeded in 1 of 1 |",
                      text)

    def test_a_batch_with_nobody_attached_has_no_table_of_actions(self):
        text = table(summarize([judge(result(seconds=462.0, delivered=[]))]))
        self.assertEqual(text.splitlines()[-1], "| **all** | **0 of 1** |")

    def test_finished_runs_are_collected_from_a_batch_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            for index, name in enumerate(("run-01", "run-02", "run-02-incomplete-17")):
                run = Path(directory) / "lidar_dropout" / "rules" / name
                run.mkdir(parents=True)
                if index < 2:
                    (run / "result.json").write_text(json.dumps(result(mode="rules")))
            self.assertEqual(len(collect(directory)), 2)

    def test_the_plan_goes_round_by_round_and_baselines_stop_early(self):
        self.assertEqual(plan(["a", "b"], ["none", "rules"], 2), [
            (1, "a", "none"), (1, "a", "rules"), (1, "b", "none"), (1, "b", "rules"),
            (2, "a", "rules"), (2, "b", "rules")])


if __name__ == "__main__":
    unittest.main()
