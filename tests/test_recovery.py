"""The recovery agent: what triggers it, what the gate lets through, and what it does, with no ROS or model."""

import json
import tempfile
import unittest
from pathlib import Path

from test_diagnosis import healthy_evidence, journal

from warehouse_agent.diagnosis import Incident
from warehouse_agent.recovery import RecoveryAgent, ToolError, gate, next_waypoint, playbook, triggers

PROCESSES = {"controller_server": {"pid": 11, "argv": ["/opt/ros/jazzy/lib/nav2_controller/controller_server"],
                                   "cwd": "/"},
             "scan_relay": {"pid": 12, "argv": ["python3", "-m", "warehouse_agent.sensor_relay"], "cwd": "/"}}
ZEROED = {"t": 21.2, "node": "/velocity_smoother", "changed": [{"name": "max_velocity", "value": [0.0, 0.0, 0.0]}]}
JUMP = {"t": 22.5, "shift_m": 2.5, "turn_rad": 0.5, "before": [0.1, -0.2, 0.01]}


def status(paused=False, hold=None, **parcels):
    """A supervisor snapshot; parcels given as P2="awaiting_pickup" (active) or P1="delivered"."""
    stations = {"P1": ("PICK_A", "DROP_A"), "P2": ("PICK_B", "DROP_B"), "P3": ("PICK_C", "DROP_C")}
    return {"paused": paused or bool(hold), "hold": hold, "finished": False, "closed": False,
            "parcels": {pid: {"pickup": stations[pid][0], "drop": stations[pid][1], "state": state,
                              "disposition": "active"} for pid, state in parcels.items()}}


class Case(unittest.TestCase):
    def live(self, evidence=None, mission=None, goals=None):
        """A run directory holding a live snapshot, a journal and a process table."""
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        run = Path(directory.name)
        evidence = evidence or healthy_evidence()
        evidence["meta"].update(live=True, mission_started=True)
        evidence["mission_status"] = mission or status(P1="onboard", P2="awaiting_pickup")
        (run / "live_evidence.json").write_text(json.dumps(evidence))
        (run / "journal.jsonl").write_text("\n".join(
            json.dumps(e) for e in journal(goals or [(0, 30, "PICK_B", None)])[:-1]) + "\n")
        (run / "processes.json").write_text(json.dumps(PROCESSES))
        return run

    def incident(self, *args, **kwargs):
        return Incident(self.live(*args, **kwargs), live=True)


class Triggers(Case):
    def test_a_healthy_mission_triggers_nothing(self):
        incident = self.incident()
        self.assertEqual(triggers(incident, incident.evidence["mission_status"]), [])

    def test_a_supervisor_fault_hold_is_a_trigger_by_itself(self):
        mission = status(hold={"kind": "fault", "message": "Navigation failed on the way to PICK_B"},
                         P2="awaiting_pickup")
        incident = self.incident(mission=mission)
        self.assertIn("holding after a failed goal", triggers(incident, mission)[0])

    def test_an_operator_pause_is_not_a_trigger(self):
        mission = status(hold={"kind": "operator", "message": "Paused by the operator"}, P2="awaiting_pickup")
        self.assertEqual(triggers(self.incident(mission=mission), mission), [])

    def test_a_blocked_stopping_pose_triggers_only_while_a_parcel_still_has_to_go_there(self):
        evidence = healthy_evidence()
        evidence["new_obstacles"] = [{"x": 1.5, "y": -0.9, "extent_m": 0.9, "first_seen": 45.0, "last_seen": 99.0,
                                      "snapshots": 20}]
        needed = status(P1="onboard", P2="awaiting_pickup")
        self.assertIn("on the PICK_B stopping pose", triggers(self.incident(evidence, needed), needed)[0])
        collected = status(P1="onboard", P2="onboard")
        self.assertEqual(triggers(self.incident(evidence, collected), collected), [])

    def test_small_pose_corrections_do_not_trigger_but_a_jump_does(self):
        evidence = healthy_evidence()
        evidence["localization_jumps"] = [{"t": 40.0, "shift_m": 0.35, "turn_rad": 0.02, "before": [0, 0, 0]}]
        incident = self.incident(evidence)
        self.assertEqual(triggers(incident, incident.evidence["mission_status"]), [])
        evidence["localization_jumps"].append(JUMP)
        incident = self.incident(evidence)
        self.assertEqual(triggers(incident, incident.evidence["mission_status"]),
                         ["the pose estimate jumped 2.5 m and 0.5 rad at 22.5 s"])


class LiveWording(Case):
    def test_a_goal_in_progress_is_not_described_as_a_recording_that_ended(self):
        from warehouse_agent.diagnosis import mission_summary
        goal = mission_summary(self.incident())["navigation_goals"][-1]
        self.assertEqual(goal["outcome"], "still under way")


class Gate(Case):
    def decide(self, action, target, evidence=None, mission=None):
        run = self.live(evidence, mission)
        incident = Incident(run, live=True)
        return gate(action, target, incident, incident.evidence["mission_status"], run, reason="because")

    def test_unknown_actions_are_refused(self):
        permitted, why, arguments = self.decide("format_disk", "/dev/sda")
        self.assertEqual((permitted, arguments), (False, None))
        self.assertIn("not one of the permitted actions", why)

    def test_a_parameter_is_restored_only_if_it_was_seen_to_change(self):
        evidence = healthy_evidence()
        evidence["parameter_changes"] = [ZEROED]
        self.assertEqual(self.decide("restore_parameter", "/velocity_smoother max_velocity", evidence),
                         (True, "this parameter changed at runtime",
                          ["restore-parameter", "/velocity_smoother", "max_velocity"]))
        self.assertEqual(self.decide("restore_parameter", "velocity_smoother max_velocity", evidence)[2],
                         ["restore-parameter", "/velocity_smoother", "max_velocity"])
        for target in ("/controller_server FollowPath.vx_max", "/velocity_smoother", "", "a b c"):
            with self.subTest(target=target):
                self.assertFalse(self.decide("restore_parameter", target, evidence)[0])
        self.assertFalse(self.decide("restore_parameter", "/velocity_smoother max_velocity")[0])

    def test_a_process_is_restarted_only_if_this_run_started_it_and_it_looks_dead(self):
        self.assertFalse(self.decide("restart_process", "controller_server")[0])       # alive and well
        evidence = healthy_evidence()
        evidence["processes_died"] = [{"t": 19.4, "process": "controller_server", "exit_code": -9}]
        self.assertEqual(self.decide("restart_process", "controller_server", evidence)[0], True)
        self.assertFalse(self.decide("restart_process", "planner_server", evidence)[0])  # not in the table
        self.assertFalse(self.decide("restart_process", "rm -rf /", evidence)[0])

    def test_the_lidar_driver_is_restarted_only_when_scans_have_stopped(self):
        self.assertFalse(self.decide("restart_process", "lidar_driver")[0])
        evidence = healthy_evidence()
        evidence["rates"]["/scan_nav"]["counts"] = [10] + [25]*5 + [0]*15
        permitted, _, arguments = self.decide("restart_process", "lidar_driver", evidence)
        self.assertTrue(permitted)
        self.assertEqual(arguments[0::2], ["restart", "scan_relay"])

    def test_relocalize_needs_an_observed_jump_and_takes_its_pose_from_the_evidence(self):
        self.assertFalse(self.decide("relocalize", "")[0])
        evidence = healthy_evidence()
        evidence["localization_jumps"] = [JUMP]
        self.assertEqual(self.decide("relocalize", "to (3, 4)", evidence)[2], ["relocalize", 0.1, -0.2, 0.01])

    def test_a_parcel_is_deferred_only_when_something_blocks_where_it_must_go_next(self):
        mission = status(P1="delivered", P2="awaiting_pickup", P3="awaiting_pickup")
        blocked = healthy_evidence()
        blocked["new_obstacles"] = [{"x": 1.5, "y": -0.9, "extent_m": 0.7, "first_seen": 45.0, "last_seen": 99.0,
                                     "snapshots": 20}]
        permitted, why, arguments = self.decide("defer_parcel", "P2", blocked, mission)
        self.assertEqual((permitted, arguments[:5]), (True, ["command", "defer", "--parcels", "P2", "--reason"]))
        self.assertIn("PICK_B", why)
        # Without an obstacle there, an active parcel is still not the agent's to give up on.
        self.assertEqual(self.decide("defer_parcel", "P2", mission=mission)[:2],
                         (False, "nothing was seen blocking PICK_B, where P2 has to go next"))
        self.assertFalse(self.decide("defer_parcel", "P3", blocked, mission)[0])     # PICK_C is clear
        for target in ("P1", "P9", ""):
            with self.subTest(target=target):
                self.assertFalse(self.decide("defer_parcel", target, blocked, mission)[0])

    def test_escalation_changes_nothing_on_the_robot(self):
        self.assertEqual(self.decide("escalate", "Remove the box at PICK_B"),
                         (True, "a message for a person; nothing on the robot changes", None))
        self.assertEqual(self.decide("clear_costmaps", "")[2], ["clear-costmaps"])


class Playbook(unittest.TestCase):
    def actions(self, category, component="", mission=None):
        decision = playbook({"category": category, "component": component}, mission or status())
        return [(a["action"], a["target"]) for a in decision["actions"]]

    def test_each_diagnosis_maps_to_its_remedy(self):
        self.assertEqual(self.actions("bad_parameter", "/velocity_smoother max_velocity"),
                         [("restore_parameter", "/velocity_smoother max_velocity")])
        self.assertEqual(self.actions("node_crash", "controller_server"),
                         [("restart_process", "controller_server")])
        self.assertEqual(self.actions("sensor_dropout", "lidar"), [("restart_process", "lidar_driver")])
        self.assertEqual(self.actions("localization_lost", "amcl"), [("relocalize", ""), ("clear_costmaps", "")])
        self.assertEqual(self.actions("no_fault"), [])
        self.assertEqual([a for a, _ in self.actions("unknown")], ["escalate"])

    def test_a_blocked_station_defers_the_parcels_that_must_go_there_next(self):
        mission = status(P1="onboard", P2="awaiting_pickup", P3="awaiting_pickup")
        self.assertEqual(self.actions("blocked_station", "PICK_B", mission),
                         [("defer_parcel", "P2"),
                          ("escalate", "Remove the obstacle on the PICK_B stopping pose")])
        self.assertEqual(self.actions("blocked_station", "DROP_A", mission)[0], ("defer_parcel", "P1"))

    def test_next_waypoint(self):
        parcels = status(P1="onboard", P2="awaiting_pickup", P3="delivered")["parcels"]
        self.assertEqual([next_waypoint(parcels[p]) for p in ("P1", "P2", "P3")], ["DROP_A", "PICK_B", None])
        self.assertIsNone(next_waypoint({**parcels["P2"], "disposition": "deferred"}))


class Agent(Case):
    def agent(self, run, mode="rules", fail=(), ask=None, settle_s=0.0):
        self.calls = []

        def tool(*arguments):
            self.calls.append(list(arguments))
            if arguments[0] in fail:
                raise ToolError(f"{arguments[0]} failed")
            return {"done": arguments[0]}

        return RecoveryAgent(run, mode, tool=tool, ask=ask, settle_s=settle_s, sleep=lambda seconds: None)

    def test_a_healthy_mission_is_left_alone(self):
        agent = self.agent(self.live())
        self.assertIsNone(agent.step())
        self.assertEqual(self.calls, [])

    def test_a_changed_parameter_is_paused_restored_and_resumed(self):
        evidence = healthy_evidence()
        evidence["parameter_changes"] = [ZEROED]
        run = self.live(evidence)
        agent = self.agent(run)
        record = agent.step()
        self.assertEqual(self.calls, [["command", "pause"],
                                      ["restore-parameter", "/velocity_smoother", "max_velocity"],
                                      ["command", "resume"]])
        self.assertEqual(record["diagnosis"]["category"], "bad_parameter")
        self.assertEqual([(a["action"], a["permitted"]) for a in record["actions"]], [("restore_parameter", True)])
        self.assertGreater(agent.since, 100.0)       # the evidence up to now has been dealt with
        self.calls.clear()
        self.assertIsNone(agent.step())              # the same snapshot no longer triggers
        self.assertEqual(self.calls, [])

    def test_a_mission_already_held_is_not_paused_again(self):
        evidence = healthy_evidence()
        evidence["processes_died"] = [{"t": 19.4, "process": "controller_server", "exit_code": -9}]
        mission = status(hold={"kind": "fault", "message": "Navigation failed"}, P2="awaiting_pickup")
        agent = self.agent(self.live(evidence, mission))
        agent.step()
        self.assertEqual([call[0] for call in self.calls], ["restart", "command"])
        self.assertEqual(self.calls[-1], ["command", "resume"])

    def test_it_waits_for_evidence_that_a_restart_worked_and_retries_parcels_the_fault_cost(self):
        evidence = healthy_evidence()
        evidence["rates"]["/scan_nav"]["counts"] = [10] + [25]*5 + [0]*15
        evidence["meta"]["recent_rate_hz"] = {"/scan_nav": 0.0}
        mission = status(hold={"kind": "fault", "message": "Navigation failed"}, P1="onboard", P3="awaiting_pickup")
        mission["parcels"]["P2"] = {"pickup": "PICK_B", "drop": "DROP_B", "state": "awaiting_pickup",
                                    "disposition": "deferred", "reason": "Navigation failed; approach error 8 m"}
        mission["parcels"]["P3"].update(disposition="deferred", reason="Deferred by the recovery agent")
        run = self.live(evidence, mission)
        agent = self.agent(run, settle_s=30.0)
        record = agent.step()
        self.assertEqual(record["settled"], "lidar scans at 0.0 Hz")            # never came back in this snapshot
        self.assertEqual([call[:2] for call in self.calls],
                         [["restart", str(run)], ["command", "retry"], ["command", "resume"]])
        self.assertEqual(self.calls[1], ["command", "retry", "--parcels", "P2"])   # not the one it deferred itself

    def test_what_a_model_proposes_goes_through_the_gate(self):
        evidence = healthy_evidence()
        evidence["parameter_changes"] = [ZEROED]
        diagnosis = {"category": "bad_parameter", "component": "/velocity_smoother max_velocity",
                     "root_cause": "The speed limit was zeroed.", "evidence": [], "confidence": "high"}
        decision = {"actions": [{"action": "restart_process", "target": "controller_server"},
                                {"action": "restore_parameter", "target": "/velocity_smoother max_velocity"},
                                {"action": "restore_parameter", "target": "/amcl max_particles"},
                                {"action": "defer_parcel", "target": "P2"},
                                {"action": "relocalize", "target": ""}],
                    "note": "Restoring the limit."}

        def ask(endpoint, payload):
            if "format" not in payload:      # the investigation step: this model asks for no tools
                return {"message": {"role": "assistant", "content": "I have what I need."}}
            wanted = decision if payload["format"]["required"][0] == "actions" else diagnosis
            return {"message": {"content": json.dumps(wanted)}}

        agent = self.agent(self.live(evidence), mode="some-model", ask=ask)
        record = agent.step()
        self.assertEqual([a["permitted"] for a in record["actions"]], [False, True, False, False, False])
        self.assertEqual([call[0] for call in self.calls], ["command", "restore-parameter", "command"])
        summary = agent.summary(injected_at=record["monotonic_detected"] - 2.0)
        self.assertEqual((summary["actions_proposed"], summary["actions_refused"], summary["actions_carried_out"],
                          summary["actions_failed"]), (5, 4, 1, 0))
        self.assertEqual(summary["seconds_fault_to_detection"], 2.0)
        # An incident from before the scenario's main fault answered something else, so it is not timed from it.
        self.assertNotIn("seconds_fault_to_detection", agent.summary(injected_at=record["monotonic_detected"] + 40.0))

    def test_a_fix_that_fails_is_tried_again_with_the_mission_held_then_handed_back(self):
        evidence = healthy_evidence()
        evidence["parameter_changes"] = [ZEROED]
        agent = self.agent(self.live(evidence), fail=("restore-parameter",))
        first = agent.step()
        self.assertEqual(first["actions"][0]["error"], "restore-parameter failed")
        self.assertNotIn("resume", first)                      # still held: resuming would only fail again
        self.assertEqual(self.calls[-1][0], "restore-parameter")
        second = agent.step()                                  # the same evidence triggers a second attempt
        self.assertNotIn("resume", second)
        third = agent.step()                                   # the last one: hand the mission back
        self.assertEqual(self.calls[-1], ["command", "resume"])
        self.assertIn("resume", third)
        self.assertIsNone(agent.step())
        self.assertEqual(agent.summary()["actions_failed"], 3)

    def test_an_unusable_model_decision_means_no_actions_and_a_resume(self):
        evidence = healthy_evidence()
        evidence["parameter_changes"] = [ZEROED]

        def ask(endpoint, payload):
            return {"message": {"content": "I think we should reboot everything"}}

        agent = self.agent(self.live(evidence), mode="some-model", ask=ask)
        record = agent.step()
        self.assertEqual(record["actions"], [])
        self.assertEqual([call for call in self.calls], [["command", "pause"], ["command", "resume"]])

    def test_it_gives_up_after_a_few_incidents_and_ignores_finished_missions(self):
        evidence = healthy_evidence()
        evidence["parameter_changes"] = [ZEROED]
        agent = self.agent(self.live(evidence))
        agent.max_incidents = 1
        self.assertIsNotNone(agent.step())
        agent.since = -1e9
        self.assertIsNone(agent.step())
        done = status(P2="awaiting_pickup")
        done["finished"] = True
        self.assertIsNone(self.agent(self.live(evidence, done)).step())
        self.assertIsNone(self.agent(Path("/nonexistent")).step())


if __name__ == "__main__":
    unittest.main()
