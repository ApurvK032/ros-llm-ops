"""Recover a running mission: watch the live evidence, diagnose, decide, act through a gate, resume.

Whatever proposes the actions, a local model or the fixed playbook, every action passes the same gate. It must be
one of a few typed actions, its target must be something this run was actually seen to do wrong, and the mission
is held while anything is touched. No action takes a free-form value. Mission decisions (pause, resume, defer) go
through the supervisor's command service, which checks them again against the mission state.

This module needs no ROS itself; it calls recovery_tools in a ROS subprocess.
"""

import json
import math
import shlex
import subprocess
import time
from pathlib import Path

from .diagnosis import (
    Incident,
    chat,
    diagnose,
    lidar_status,
    motion_status,
    node_health,
    obstacles,
    parameter_changes,
    rule_based,
)
from .faults import ros_env

ROOT = Path(__file__).resolve().parent.parent
RELAY = "scan_relay"
LIDAR_NAMES = {"lidar_driver", "lidar", "scan_relay"}
ACTIONS = {
    "restore_parameter": "Put a parameter that changed during the mission back to its configured value. "
                         "target: the node and the parameter, for example '/velocity_smoother max_velocity'.",
    "restart_process": "Restart a process that died or stopped responding. target: the Nav2 server's name, for "
                       "example 'controller_server', or 'lidar_driver' when scans stopped arriving.",
    "relocalize": "Undo a sudden jump in the pose estimate, using the last good correction and odometry. "
                  "target: leave empty.",
    "clear_costmaps": "Forget obstacles the costmaps have accumulated; needed after relocalize, because scans "
                      "taken with a wrong pose left obstacles in the wrong places. target: leave empty.",
    "defer_parcel": "Give up on one parcel for now, only because an obstacle stands on the stopping pose it has "
                    "to reach next. target: the parcel ID.",
    "escalate": "Ask a person to do what the robot cannot, such as removing an obstacle. target: what to do.",
}
DECISION = {
    "type": "object",
    "properties": {
        "actions": {"type": "array", "items": {
            "type": "object",
            "properties": {"action": {"type": "string", "enum": list(ACTIONS)}, "target": {"type": "string"}},
            "required": ["action", "target"], "additionalProperties": False}},
        "note": {"type": "string", "description": "One sentence for the operator"},
    }, "required": ["actions", "note"], "additionalProperties": False,
}
DECIDE = """You are the on-call engineer for a warehouse robot. The mission is paused while you deal with a fault \
you have just diagnosed. Choose recovery actions, in the order they should run, from this list only:

""" + "\n".join(f"- {name}: {text}" for name, text in ACTIONS.items()) + """

Choose the fewest actions that remove the cause named in the diagnosis, and nothing else. An action whose \
target was not observed to be faulty will be refused. Do not defer a parcel or mention an obstacle unless the \
diagnosis category is blocked_station; in that case the robot cannot move the obstacle, so defer the parcel that \
has to go to that waypoint next and escalate so a person clears it. If nothing needs fixing, give no actions. \
The mission resumes by itself once your actions have run."""


class ToolError(RuntimeError):
    """A recovery tool ran and failed; the action counts as attempted, not as refused."""


def run_tool(*arguments, timeout=120):
    """Run one recovery_tools command in the ROS environment and return the JSON it prints."""
    quoted = " ".join(shlex.quote(str(a)) for a in arguments)
    done = subprocess.run(["bash", "-c", "source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash && "
                           f"exec python3 -B -m warehouse_agent.recovery_tools {quoted}"],
                          cwd=ROOT, env=ros_env(), capture_output=True, text=True, timeout=timeout)
    lines = (done.stdout if done.returncode == 0 else done.stderr or done.stdout).strip().splitlines()
    if done.returncode != 0:
        raise ToolError(lines[-1] if lines else "recovery tool failed")
    return json.loads(lines[-1])


def next_waypoint(parcel):
    """Where an active parcel has to go next, or None when it owes nothing."""
    if parcel["disposition"] != "active" or parcel["state"] == "delivered":
        return None
    return parcel["pickup"] if parcel["state"] == "awaiting_pickup" else parcel["drop"]


def big_jumps(incident):
    return [j for j in incident.evidence["localization_jumps"]
            if incident.recent(j["t"]) and (j["shift_m"] >= 1.0 or j["turn_rad"] >= 0.5)]


def triggers(incident, status):
    """What is wrong right now and worth interrupting the mission for. Empty when all is well."""
    lines = []
    hold = status.get("hold") or {}
    if hold.get("kind") == "fault":
        lines.append("the supervisor is holding after a failed goal: " + hold.get("message", ""))
    lines += node_health(incident)["unusual"] + lidar_status(incident)["unusual"]
    lines += parameter_changes(incident)["unusual"]
    for jump in big_jumps(incident)[:1]:
        lines.append(f"the pose estimate jumped {jump['shift_m']} m and {jump['turn_rad']} rad at {jump['t']} s")
    owed = {next_waypoint(p) for p in status.get("parcels", {}).values()} - {None}
    lines += [text for text in obstacles(incident)["unusual"]
              if any(f"on the {waypoint} stopping pose" in text for waypoint in owed)]
    lines += motion_status(incident)["unusual"]
    return lines


def gate(action, target, incident, status, run, reason=""):
    """Decide whether one proposed action may run. Returns (permitted, why, tool arguments or None)."""
    target = (target or "").strip()
    if action not in ACTIONS:
        return False, "not one of the permitted actions", None
    if action == "restore_parameter":
        parts = target.split()
        if len(parts) != 2:
            return False, "the target must be a node and a parameter", None
        node, name = "/" + parts[0].lstrip("/"), parts[1]
        changed = {(c["node"], p["name"]) for c in parameter_changes(incident)["changes"] for p in c["changed"]}
        if (node, name) not in changed:
            return False, f"{node} {name} was not seen to change during this mission", None
        return True, "this parameter changed at runtime", ["restore-parameter", node, name]
    if action == "restart_process":
        name = RELAY if target in LIDAR_NAMES else target
        table_file = Path(run) / "processes.json"
        table = json.loads(table_file.read_text(encoding="utf-8")) if table_file.exists() else {}
        if name not in table:
            return False, f"{target or 'that'} is not a process this run started", None
        if name == RELAY:
            faulty = bool(lidar_status(incident)["unusual"])
        else:
            faulty = any(line.startswith(name + " ") or f"first to stop was {name} " in line
                         for line in node_health(incident)["unusual"])
        if not faulty:
            return False, f"{target} shows no sign of being dead or silent", None
        return True, "this process was seen dead or silent", ["restart", str(run), name]
    if action == "relocalize":
        jumps = [j for j in big_jumps(incident) if "before" in j]
        if not jumps:
            return False, "no jump in the pose estimate was observed", None
        return True, f"the pose estimate jumped at {jumps[0]['t']} s", ["relocalize", *jumps[0]["before"]]
    if action == "clear_costmaps":
        return True, "harmless while the mission is held", ["clear-costmaps"]
    if action == "defer_parcel":
        parcel = status.get("parcels", {}).get(target)
        waypoint = next_waypoint(parcel) if parcel else None
        if waypoint is None:
            return False, f"{target or 'that'} is not an active, undelivered parcel", None
        # Giving up on a delivery needs a reason the robot cannot fix: something standing where it must stop.
        if not any(f"on the {waypoint} stopping pose" in text for text in obstacles(incident)["unusual"]):
            return False, f"nothing was seen blocking {waypoint}, where {target} has to go next", None
        return True, f"an obstacle was seen on the {waypoint} stopping pose", [
            "command", "defer", "--parcels", target, "--reason", reason or "Deferred by the recovery agent"]
    return True, "a message for a person; nothing on the robot changes", None


def playbook(diagnosis, status):
    """The fixed mapping from a diagnosis to actions: the baseline the model is compared with."""
    category, component = diagnosis["category"], diagnosis.get("component", "")
    actions, note = [], f"Diagnosed {category}."
    if category == "bad_parameter":
        actions = [{"action": "restore_parameter", "target": component}]
    elif category == "node_crash":
        actions = [{"action": "restart_process", "target": component}]
    elif category == "sensor_dropout":
        actions = [{"action": "restart_process", "target": "lidar_driver"}]
    elif category == "localization_lost":
        actions = [{"action": "relocalize", "target": ""}, {"action": "clear_costmaps", "target": ""}]
    elif category == "blocked_station":
        blocked = [pid for pid, parcel in status.get("parcels", {}).items() if next_waypoint(parcel) == component]
        actions = [{"action": "defer_parcel", "target": pid} for pid in blocked]
        actions.append({"action": "escalate", "target": f"Remove the obstacle on the {component} stopping pose"})
    elif category == "blocked_path":
        actions = [{"action": "escalate", "target": "An obstacle is in an aisle; the robot is routing around it"}]
    elif category not in ("no_fault",):
        actions = [{"action": "escalate", "target": f"Unresolved fault ({category}); the mission will retry"}]
    return {"actions": actions, "note": note}


class RecoveryAgent:
    """Handles faults in one running mission. mode is "rules" or the name of a local model."""

    def __init__(self, run, mode="rules", endpoint="http://127.0.0.1:11434", tool=run_tool, ask=chat,
                 max_incidents=3, settle_s=25.0, sleep=time.sleep):
        self.run, self.mode, self.endpoint = Path(run), mode, endpoint
        self.tool, self.ask, self.max_incidents = tool, ask, max_incidents
        self.settle_s, self.sleep = settle_s, sleep
        self.since = -math.inf
        self.incidents = []

    def step(self):
        """Look at the latest evidence and deal with a fault if there is one. Returns the record, or None."""
        try:
            incident = Incident(self.run, live=True)
        except (OSError, json.JSONDecodeError):
            return None
        status = incident.evidence.get("mission_status")
        if not incident.evidence["meta"].get("mission_started") or not status or status.get("closed") \
                or status.get("finished") or len(self.incidents) >= self.max_incidents:
            return None
        if incident.end < self.since:
            return None     # this snapshot predates the last recovery; wait for one that has seen its effect
        incident.since = self.since
        alerts = triggers(incident, status)
        return self.handle(incident, status, alerts) if alerts else None

    def settled(self, fixed):
        """After fixes, wait until the live evidence shows them working before the mission moves again.

        Resuming the instant a sensor or a Nav2 server is back fails the retry: costmaps, the collision monitor
        and localization need a few scans first. Returns what was seen, for the record.
        """
        if not fixed & {"restart_process", "relocalize"} or self.settle_s <= 0:
            return None
        seen = "not confirmed in time"
        for second in range(1, int(self.settle_s) + 1):
            self.sleep(1.0)
            try:
                evidence = Incident(self.run, live=True).evidence
            except (OSError, json.JSONDecodeError):
                continue
            rates = evidence["meta"].get("recent_rate_hz", {})
            scans = rates.get("/scan_nav", rates.get("/scan", 0.0))
            silent = sorted(name for name, bond in evidence["bonds"].items() if bond["silent_for_s"] > 1.0)
            if scans < 3.0:
                seen = f"lidar scans at {scans} Hz"
            elif silent:
                seen = f"no heartbeat from {silent[0]}"
            elif second >= 4:
                return f"scans at {scans} Hz and every heartbeat present, {second} s after the fix"
        return seen

    def latest_parcels(self, fallback):
        """Parcel states from the newest snapshot, which may be later than the one that triggered."""
        try:
            status = Incident(self.run, live=True).evidence.get("mission_status") or fallback
        except (OSError, json.JSONDecodeError):
            status = fallback
        return status.get("parcels", {})

    def decide(self, diagnosis, status):
        if self.mode == "rules":
            return playbook(diagnosis, status)
        parcels = {pid: {"state": p["state"], "disposition": p["disposition"], "pickup": p["pickup"],
                         "drop": p["drop"], "goes_next_to": next_waypoint(p)}
                   for pid, p in status.get("parcels", {}).items()}
        reply = self.ask(self.endpoint, {
            "model": self.mode, "stream": False, "think": False, "format": DECISION,
            "options": {"temperature": 0, "num_ctx": 8192, "num_predict": 500},
            "messages": [{"role": "system", "content": DECIDE},
                         {"role": "user", "content": json.dumps({"diagnosis": diagnosis, "parcels": parcels})}]})
        try:
            decision = json.loads(reply["message"]["content"])
            if not isinstance(decision.get("actions"), list):
                raise ValueError
        except (json.JSONDecodeError, KeyError, ValueError, AttributeError):
            return {"actions": [], "note": "The model gave no usable decision; continuing."}
        return decision

    def handle(self, incident, status, alerts):
        started = time.monotonic()
        record = {"detected_at_s": incident.end, "monotonic_detected": started, "alerts": alerts, "actions": []}
        try:
            if not status.get("paused"):
                record["pause"] = self.tool("command", "pause")
            outcome = rule_based(incident) if self.mode == "rules" else diagnose(
                incident, self.mode, self.endpoint, ask=self.ask)
            diagnosis = outcome["diagnosis"]
            record["diagnosis"], record["diagnosis_tool_calls"] = diagnosis, outcome["tool_calls"]
            decision = self.decide(diagnosis, status)
            record["decision"] = decision
            for item in decision["actions"]:
                action, target = item.get("action"), item.get("target", "")
                permitted, why, arguments = gate(action, target, incident, status, self.run,
                                                 reason=diagnosis.get("root_cause", "")[:160])
                entry = {"action": action, "target": target, "permitted": permitted, "why": why}
                if permitted and arguments:
                    try:
                        entry["result"] = self.tool(*arguments)
                    except (ToolError, subprocess.TimeoutExpired) as exc:
                        entry["error"] = str(exc)
                record["actions"].append(entry)
            failed = any("error" in entry for entry in record["actions"])
            fixed = {entry["action"] for entry in record["actions"] if "result" in entry}
            if fixed and not failed:
                settled = self.settled(fixed)
                if settled:
                    record["settled"] = settled
                # Parcels the supervisor gave up on while the fault was live are owed a fresh attempt.
                lost = sorted(pid for pid, parcel in self.latest_parcels(status).items()
                              if parcel["disposition"] == "deferred" and parcel["state"] != "delivered"
                              and parcel.get("reason", "").startswith("Navigation"))
                if lost and fixed & {"restart_process", "relocalize", "restore_parameter"}:
                    record["retry"] = self.tool("command", "retry", "--parcels", *lost)
            last_chance = len(self.incidents) + 1 >= self.max_incidents
            # A fix that did not take is tried again on the next look, with the mission still held. On the
            # last attempt the mission is handed back regardless, so the supervisor's own policy can finish it.
            if not failed or last_chance:
                record["resume"] = self.tool("command", "resume")
        except (ToolError, subprocess.TimeoutExpired) as exc:
            record["error"] = str(exc)
            failed = True
        record["monotonic_done"] = time.monotonic()
        record["seconds"] = round(record["monotonic_done"] - started, 1)
        if "resume" in record:
            # Mission time now, plus a margin for a snapshot already on its way: what was dealt with is history.
            self.since = incident.end + record["seconds"] + 3.0
        self.incidents.append(record)
        return record

    def summary(self, injected_at=None):
        actions = [a for incident in self.incidents for a in incident["actions"]]
        done = {"mode": self.mode, "incidents": self.incidents, "actions_proposed": len(actions),
                "actions_refused": sum(not a["permitted"] for a in actions),
                "actions_failed": sum("error" in a for a in actions),
                "actions_carried_out": sum(a["permitted"] and "error" not in a for a in actions)}
        if injected_at is not None and self.incidents:
            first = self.incidents[0]
            done["seconds_fault_to_detection"] = round(first["monotonic_detected"] - injected_at, 1)
            done["seconds_fault_to_resume"] = round(first["monotonic_done"] - injected_at, 1) \
                if "resume" in first else None
        return done
