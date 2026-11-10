"""Diagnose a recorded run: read-only checks over its evidence, a local-model agent that uses them, and scoring.

The checks report observations with times, never conclusions. Three things use them: the agent (as tools), a
fixed rule-based baseline (to show what needs no model at all), and the tests. None of this needs ROS: it reads
the run's journal and the evidence.json written by evidence.py.
"""

import json
import math
import re
import statistics
import time
from pathlib import Path
from urllib.request import Request, urlopen

from .faults import CRASHABLE_NODES
from .scenario import read_events

CATEGORIES = {
    "blocked_station": "An obstacle stands on a waypoint's stopping pose, so the robot cannot reach that goal.",
    "blocked_path": "An obstacle blocks the route between waypoints, but every stopping pose is free.",
    "node_crash": "A navigation process died or stopped responding.",
    "sensor_dropout": "A sensor stopped delivering data to navigation.",
    "localization_lost": "The robot's pose estimate no longer matches where it really is.",
    "bad_parameter": "A parameter changed at runtime to a value that breaks navigation.",
    "actuator_fault": "Velocity commands are sent but the robot does not move, with configuration unchanged.",
    "no_fault": "The mission ran normally.",
    "unknown": "The evidence does not support any of the above.",
}
# Introduced by the scenario harness, not part of the robot: a real sensor fault leaves no such trace.
HARNESS_NODES = {"/scan_relay", "/fault_tools"}
SYSTEM = """You are the on-call engineer for a warehouse robot that runs ROS 2 and Nav2. A delivery mission has \
just ended and you must find out what, if anything, went wrong. You have read-only tools that report \
observations from the recording of this run. Times are seconds from the start of the mission.

Work like an engineer: call health_report first, then call the detailed tool for anything unusual before \
deciding. Several things can look wrong at once: the root cause is usually the earliest unusual observation, \
and later ones are often its consequences, so compare their times. Base the diagnosis only on what the tools \
report; if nothing is unusual and the mission completed, the answer is no_fault. Name the specific component \
(the node, sensor, parameter or waypoint) in your answer.

Fault categories:
""" + "\n".join(f"- {name}: {text}" for name, text in CATEGORIES.items())
ANSWER = {
    "type": "object",
    "properties": {
        "category": {"type": "string", "enum": list(CATEGORIES)},
        "component": {"type": "string", "description": "The node, sensor, parameter or waypoint at fault"},
        "root_cause": {"type": "string", "description": "One or two sentences: what happened and when"},
        "evidence": {"type": "array", "items": {"type": "string"},
                     "description": "The observations that support this, with times"},
        "confidence": {"type": "string", "enum": ["low", "medium", "high"]},
    }, "required": ["category", "component", "root_cause", "evidence", "confidence"], "additionalProperties": False,
}


class Incident:
    """One recorded run: its journal, extracted evidence and, when known, the injected fault."""

    def __init__(self, run, live=False):
        """live=True reads the monitor's latest snapshot of a mission that is still running."""
        self.run = Path(run)
        name = "live_evidence.json" if live else "evidence.json"
        self.evidence = json.loads((self.run / name).read_text(encoding="utf-8"))
        self.events = read_events(self.run / "journal.jsonl")
        result = self.run / "result.json"
        self.result = json.loads(result.read_text(encoding="utf-8")) if result.exists() else {}
        started = next((e for e in self.events if e["type"] == "episode_started"), None)
        self.waypoints = started["config"]["waypoints"] if started else {}
        applied = next((e for e in self.events if e["type"] == "instruction_applied"), None)
        self.origin = applied["monotonic"] if applied else (self.events[0]["monotonic"] if self.events else 0.0)
        self.end = self.evidence["meta"]["recording_end"]
        # Checks ignore what happened up to here. A live agent moves it forward after each recovery, so a fault
        # it has dealt with is not reported again; for a finished recording it stays at the very beginning.
        self.since = -math.inf

    def recent(self, t):
        """True for something that happened during the mission and after the last point already dealt with."""
        return t > max(1.0, self.since)

    def at(self, event):
        return round(event["monotonic"] - self.origin, 1)

    @property
    def truth(self):
        """The injected fault's ground truth; a run with no injected fault is a no_fault case."""
        if self.result.get("fault", {}).get("injected"):
            return self.result["scenario"]["ground_truth"]
        return {"category": "no_fault"}

    def goals(self):
        """Navigation goals as (start, end, stop, outcome); a goal still running at the end has outcome None."""
        goals, open_goal = [], None
        for event in self.events:
            if event["type"] == "navigation_started":
                open_goal = (self.at(event), event["stop"])
            elif event["type"] == "navigation_finished" and open_goal:
                goals.append((open_goal[0], self.at(event), open_goal[1], event["outcome"]))
                open_goal = None
        if open_goal:
            goals.append((open_goal[0], self.end, open_goal[1], None))
        return goals


# --- checks: observations only -------------------------------------------------------------------------------------

def mission_summary(incident):
    """What the supervisor saw: each navigation goal with its outcome, deferred parcels, and how the run ended."""
    events = incident.events
    request = next((e["text"] for e in events if e["type"] == "language_requested"), None)
    finished = next((e for e in reversed(events) if e["type"] == "mission_finished"), None)
    state = next((e["state"] for e in reversed(events) if isinstance(e.get("state"), dict)), {})
    parcels = state.get("parcels", {})
    goals = [{"start": start, "seconds": round(end - start, 1), "stop": f"{stop['kind']} {stop['parcel']}",
              "waypoint": stop["waypoint"], "outcome": outcome or (
                  "still under way" if incident.evidence["meta"].get("live")
                  else "still running when the recording ended")}
             for start, end, stop, outcome in incident.goals()]
    held = next((e["reason"] for e in events if e["type"] == "planning_failed"), None)
    return {
        "operator_request": request,
        "ended": finished["result"] if finished else ("held: " + held if held else "did not finish"),
        "ended_at": incident.at(finished) if finished else incident.end,
        "delivered": sorted(p for p, v in parcels.items() if v["state"] == "delivered"),
        "deferred": sorted(p for p, v in parcels.items() if v["disposition"] == "deferred"),
        "still_onboard": sorted(p for p, v in parcels.items() if v["state"] == "onboard"),
        "navigation_goals": goals,
        "unusual": [f"{g['stop']} to {g['waypoint']} started at {g['start']} s: {g['outcome']} after "
                    f"{g['seconds']} s" for g in goals if g["outcome"] != "succeeded"][:8],
    }


def node_health(incident):
    """Nav2 process health: heartbeats each server sends to the lifecycle manager, and processes that died."""
    bonds, end = incident.evidence["bonds"], incident.end
    silent = sorted(((b["last"], name) for name, b in bonds.items() if b["silent_for_s"] > 1.0))
    # Only Nav2 servers: the launch log also reports helper processes, and the lidar is judged by its scans.
    died = [d for d in incident.evidence["processes_died"]
            if d["t"] > incident.since and d["process"] in CRASHABLE_NODES]
    unusual = [f"{d['process']} process died at about {d['t']} s (exit code {d['exit_code']})" for d in died]
    if silent and len(silent) < len(bonds):
        unusual += [f"{name} heartbeat stopped at {last} s; recording ends at {end} s" for last, name in silent]
    elif silent:
        first = silent[0]
        unusual.append(f"all {len(bonds)} heartbeats stopped; the first to stop was {first[1]} at {first[0]} s")
    restarts = {name: b["restarts"] for name, b in bonds.items() if b["restarts"]}
    return {"nodes_with_heartbeat": len(bonds), "heartbeat_period_s": 0.1,
            "last_heartbeat_s": {name: b["last"] for name, b in bonds.items()} if silent else "all until the end",
            "restarted": restarts, "processes_died": died, "unusual": unusual}


def lidar_status(incident):
    """Lidar scans reaching navigation: message rate over time, share of valid ranges, closest return."""
    topic = "/scan_nav" if "/scan_nav" in incident.evidence["rates"] else "/scan"
    rate = incident.evidence["rates"].get(topic)
    if not rate or not rate["counts"]:
        return {"scans_received": 0, "unusual": ["no lidar scans were recorded at all"]}
    bucket = incident.evidence["meta"]["bucket_s"]
    counts, start = rate["counts"], rate["start"]
    nominal = max(counts)  # the rate while healthy; a median would be zero after a long dropout
    # The first and last buckets are partly outside the recording; scale what a full rate would give them.
    covered = [min(bucket, incident.end - (start + i*bucket)) for i in range(len(counts))]
    covered[0] = min(bucket, start + bucket - incident.evidence["meta"]["recording_start"])
    # Intervals before the mission began are start-up (the recorder and the lidar come up at different moments).
    low = [i for i, (n, seconds) in enumerate(zip(counts, covered, strict=True))
           if start + i*bucket >= max(0.0, incident.since) and seconds >= 1.0
           and n < 0.5*nominal*seconds/bucket]
    unusual = []
    if low and low == list(range(low[0], len(counts))):
        unusual.append(f"scans stopped arriving at about {round(start + low[0]*bucket, 1)} s and never resumed "
                       f"(normal rate {round(nominal/bucket, 1)} Hz)")
    elif low:
        unusual.append(f"scan rate dropped below half of normal in {len(low)} of {len(counts)} intervals, "
                       f"first at {round(start + low[0]*bucket, 1)} s")
    quality = incident.evidence["lidar"].get(topic, [])
    weak = [row[0] for row in quality if row[2] < 0.5]
    if weak:
        unusual.append(f"fewer than half the ranges were valid from {weak[0]} s")
    return {"normal_rate_hz": round(nominal/bucket, 1),
            "rate_hz_by_interval": [[round(start + i*bucket, 1), round(n/max(seconds, 0.1), 1)]
                                    for i, (n, seconds) in enumerate(zip(counts, covered, strict=True))][-12:],
            "unusual": unusual}


def motion_status(incident):
    """Commanded speed against measured speed (wheel odometry), while navigation goals were active."""
    rows = {row[0]: row for row in incident.evidence["motion"]}
    stalled, uncommanded = [], []
    for start, end, stop, _ in incident.goals():
        run_stalled = run_idle = 0
        first = int(start) + 3 if incident.since < start else math.ceil(incident.since)
        for second in range(first, int(end)):
            row = rows.get(second)
            measured = abs(row[1]) if row and row[1] is not None else 0.0
            turning = abs(row[2]) if row and row[2] is not None else 0.0
            commanded = max(abs(row[3] or 0.0), abs(row[4] or 0.0)) if row and row[3] is not None else None
            moving = measured > 0.02 or turning > 0.05
            run_stalled = run_stalled + 1 if (commanded or 0) > 0.05 and not moving else 0
            run_idle = run_idle + 1 if not commanded and not moving else 0
            if run_stalled == 10:
                stalled.append((second - 9, f"{stop['kind']} {stop['parcel']}"))
            if run_idle == 10:
                uncommanded.append((second - 9, f"{stop['kind']} {stop['parcel']}"))
    unusual = [f"from {t} s (goal: {goal}) velocity commands were sent but the robot did not move for 10 s or more"
               for t, goal in stalled[:3]]
    unusual += [f"from {t} s (goal: {goal}) the robot stood still and received no non-zero velocity command for "
                "10 s or more, although a goal was active" for t, goal in uncommanded[:3]]
    table = []
    for begin in range(0, int(incident.end) + 1, 20):
        window = [rows[s] for s in range(begin, begin + 20) if s in rows]
        speeds = [abs(r[1]) for r in window if r[1] is not None]
        commands = [abs(r[3]) for r in window if r[3] is not None]
        table.append([begin, round(sum(speeds)/len(speeds), 2) if speeds else None,
                      round(sum(commands)/len(commands), 2) if commands else "no commands"])
    return {"columns": ["interval_start_s", "mean_measured_speed_m_s", "mean_commanded_speed_m_s"],
            "by_20s_interval": table[:8] + (["..."] + table[-6:] if len(table) > 14 else table[8:]),
            "unusual": unusual}


def localization_status(incident):
    """The pose estimate: sudden jumps in the map-to-odometry correction, external pose resets, AMCL spread."""
    evidence = incident.evidence
    jumps = [j for j in evidence["localization_jumps"] if incident.recent(j["t"])]
    resets = [r for r in evidence["pose_resets"] if incident.recent(r["t"])]
    big = [j for j in jumps if j["shift_m"] >= 1.0 or j["turn_rad"] >= 0.5]
    spread = [row[4] for row in evidence["amcl"]]
    unusual = [f"the pose estimate jumped {j['shift_m']} m and {j['turn_rad']} rad in one update at {j['t']} s"
               for j in big[:3]]
    unusual += [f"an external pose reset (/initialpose) set the estimate to ({r['x']}, {r['y']}) at {r['t']} s"
                for r in resets[:3]]
    if len(jumps) - len(big) >= 3:
        unusual.append(f"{len(jumps) - len(big)} further corrections of 0.3 m or more, from {jumps[0]['t']} s")
    return {"jumps_over_0.3m": jumps[:6], "jump_count": len(jumps), "external_pose_resets": resets,
            "amcl_position_spread_m": {"median": round(statistics.median(spread), 3), "max": round(max(spread), 3)}
            if spread else None, "unusual": unusual}


def parameter_changes(incident):
    """Parameters changed on running nodes during the mission (startup configuration is not listed)."""
    changes = [c for c in incident.evidence["parameter_changes"]
               if c["node"] not in HARNESS_NODES and incident.recent(c["t"])]
    return {"changes": changes,
            "unusual": [f"{c['node']} parameter {p['name']} was set to {json.dumps(p['value'])} at {c['t']} s"
                        for c in changes for p in c["changed"]][:6]}


def obstacles(incident):
    """Obstacles in the global costmap that are not in the static map: where, how large, how long, near what."""
    found = []
    for cluster in incident.evidence["new_obstacles"]:
        if cluster["snapshots"] < 3 or cluster["last_seen"] <= incident.since:
            continue
        name, distance = min(((n, math.dist(p[:2], (cluster["x"], cluster["y"])))
                              for n, p in incident.waypoints.items()), key=lambda item: item[1],
                             default=(None, math.inf))
        found.append({**cluster, "nearest_waypoint": name, "distance_to_it_m": round(distance, 2)})
    failed = {stop["waypoint"] for _, _, stop, outcome in incident.goals() if outcome != "succeeded"}
    unusual = []
    for item in found:
        if item["extent_m"] >= 2.0:
            continue
        where = (f"on the {item['nearest_waypoint']} stopping pose" if item["distance_to_it_m"] <= 0.6
                 else f"{item['distance_to_it_m']} m from {item['nearest_waypoint']}")
        goal = " (a goal to that waypoint failed)" if item["nearest_waypoint"] in failed \
            and item["distance_to_it_m"] <= 0.6 else ""
        unusual.append(f"a {item['extent_m']} m obstacle not in the map appeared {where}{goal}, seen from "
                       f"{item['first_seen']} s to {item['last_seen']} s")
    wide = [item for item in found if item["extent_m"] >= 2.0]
    if wide:
        unusual.append(f"{len(wide)} wall-sized structure(s) of {max(i['extent_m'] for i in wide)} m appeared where "
                       "the map is free, which usually means scans are not lining up with the map")
    return {"new_obstacles": found[:8], "unusual": unusual[:6]}


def planning_status(incident):
    """Planner and controller behaviour: how often each failed, recovery behaviours run, planner messages."""
    counts = dict(incident.evidence["behaviour_tree"]["failure_counts"])
    if incident.since > -math.inf:
        # Totals cover the whole mission; recount the two that matter from the timed list.
        for name in ("ComputePathToPose", "FollowPath"):
            counts[name] = sum(1 for f in incident.evidence["behaviour_tree"]["failures"]
                               if f["node"] == name and f["t"] > incident.since)
    planner_logs = [log for log in incident.evidence["logs"] if log["node"] == "planner_server" and "failed" in
                    log["text"].lower() and incident.recent(log["t_last"])]
    summary = {"planner_failures": counts.get("ComputePathToPose", 0), "controller_failures": counts.get("FollowPath", 0),
               "recoveries_started": {k.split(":")[1]: v for k, v in counts.items() if k.startswith("started:")},
               "planner_messages": [f"{log['t']} s: {log['text'][:140]}" for log in planner_logs[:3]]}
    unusual = []
    if summary["planner_failures"]:
        unusual.append(f"the planner failed to find a path {summary['planner_failures']} time(s)")
    if summary["controller_failures"]:
        unusual.append(f"the controller aborted path following {summary['controller_failures']} time(s)")
    return {**summary, "unusual": unusual}


def nav2_warnings(incident):
    """Warnings and errors logged by Nav2 nodes during the mission, grouped, with first time and count."""
    groups = {}
    for log in incident.evidence["logs"]:
        if not incident.recent(log["t_last"]) or "/" + log["node"] in HARNESS_NODES:
            continue
        key = (log["level"], log["node"], re.sub(r"-?\d+(\.\d+)?", "#", log["text"])[:110])
        group = groups.setdefault(key, {"first_s": log["t"], "count": 0, "example": log["text"][:160]})
        group["count"] += log["count"]
    ordered = sorted(groups.items(), key=lambda item: item[1]["first_s"])
    return {"messages": [{"first_s": g["first_s"], "count": g["count"], "level": key[0], "node": key[1],
                          "text": g["example"]} for key, g in ordered[:14]],
            "distinct_messages": len(ordered),
            "unusual": [f"{key[1]} {key[0]} from {g['first_s']} s (x{g['count']}): {g['example'][:90]}"
                        for key, g in ordered if key[0] in ("ERROR", "FATAL")][:5]}


CHECKS = {"node_health": node_health, "lidar_status": lidar_status, "motion_status": motion_status,
          "localization_status": localization_status, "parameter_changes": parameter_changes,
          "obstacles": obstacles, "planning_status": planning_status, "nav2_warnings": nav2_warnings}


def health_report(incident):
    """Run every check and list what each found unusual. Start here, then open the detailed tool for each."""
    report = {"mission": mission_summary(incident)["unusual"] or "all navigation goals succeeded"}
    for name, check in CHECKS.items():
        report[name] = check(incident)["unusual"] or "nothing unusual"
    return report


TOOLS = {"health_report": health_report, "mission_summary": mission_summary, **CHECKS}


def tool_specs():
    return [{"type": "function", "function": {"name": name, "description": " ".join(function.__doc__.split()),
                                              "parameters": {"type": "object", "properties": {}, "required": []}}}
            for name, function in TOOLS.items()]


# --- the agent -----------------------------------------------------------------------------------------------------

def brief(incident):
    summary = mission_summary(incident)
    goals = "\n".join(f"  {g['start']:>6} s  {g['stop']} at {g['waypoint']}: {g['outcome']} after {g['seconds']} s"
                      for g in summary["navigation_goals"])
    return (f"Operator request: {summary['operator_request']!r}\n"
            f"The mission ended as: {summary['ended']} at {summary['ended_at']} s. Delivered: "
            f"{summary['delivered'] or 'none'}. Deferred: {summary['deferred'] or 'none'}. Still on the robot: "
            f"{summary['still_onboard'] or 'none'}.\nNavigation goals:\n{goals}\n\nDiagnose this run.")


def chat(endpoint, payload):
    request = Request(endpoint.rstrip("/") + "/api/chat", data=json.dumps(payload).encode(),
                      headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=300) as response:
        return json.load(response)


def diagnose(incident, model="qwen3.5:4b", endpoint="http://127.0.0.1:11434", use_tools=True, max_steps=8,
             ask=chat):
    """Let the model investigate with the tools, then give a structured diagnosis. Returns it with a transcript."""
    options = {"temperature": 0, "num_ctx": 8192, "num_predict": 700}
    messages = [{"role": "system", "content": SYSTEM if use_tools else SYSTEM.replace(
                    "call health_report first, then call the detailed tool for anything unusual before deciding",
                    "you have no tools in this session, only the mission report below")},
                {"role": "user", "content": brief(incident)}]
    calls, started, tokens = [], time.monotonic(), 0
    for _ in range(max_steps if use_tools else 0):
        reply = ask(endpoint, {"model": model, "stream": False, "think": False, "options": options,
                               "messages": messages, "tools": tool_specs()})
        tokens += reply.get("eval_count") or 0
        message = reply["message"]
        requested = message.get("tool_calls") or []
        if not requested:
            break
        messages.append(message)
        for call in requested:
            name = call["function"]["name"]
            output = TOOLS[name](incident) if name in TOOLS else {"error": f"no tool named {name}"}
            calls.append(name)
            messages.append({"role": "tool", "tool_name": name, "content": json.dumps(output)})
    messages.append({"role": "user", "content": "Give your diagnosis now as JSON."})
    reply = ask(endpoint, {"model": model, "stream": False, "think": False, "options": options, "messages": messages,
                           "format": ANSWER})
    tokens += reply.get("eval_count") or 0
    try:
        answer = json.loads(reply["message"]["content"])
    except (json.JSONDecodeError, KeyError):
        answer = {}
    if answer.get("category") not in CATEGORIES:
        answer = {"category": "unknown", "component": "", "root_cause": "The model gave no usable answer.",
                  "evidence": [], "confidence": "low"}
    return {"diagnosis": answer, "tool_calls": calls, "model": model, "used_tools": use_tools,
            "seconds": round(time.monotonic() - started, 1), "output_tokens": tokens}


# --- a baseline that needs no model --------------------------------------------------------------------------------

def rule_based(incident):
    """Fixed rules over the same checks, in a fixed order. What the model has to beat."""
    def answer(category, component, because):
        return {"diagnosis": {"category": category, "component": component, "root_cause": because[0] if because
                              else "", "evidence": because, "confidence": "high" if because else "low"},
                "tool_calls": [], "model": "rules", "used_tools": True, "seconds": 0.0, "output_tokens": 0}
    nodes = node_health(incident)
    if nodes["unusual"]:
        died = nodes["processes_died"]
        first = died[0]["process"] if died else min(incident.evidence["bonds"].items(),
                                                    key=lambda item: item[1]["last"])[0]
        return answer("node_crash", first, nodes["unusual"])
    parameters = parameter_changes(incident)
    if parameters["unusual"]:
        change = parameters["changes"][0]
        return answer("bad_parameter", f"{change['node']} {change['changed'][0]['name']}", parameters["unusual"])
    lidar = lidar_status(incident)
    if lidar["unusual"]:
        return answer("sensor_dropout", "lidar", lidar["unusual"])
    localization = localization_status(incident)
    if localization["external_pose_resets"] or any("jumped" in text for text in localization["unusual"]):
        return answer("localization_lost", "amcl pose estimate", localization["unusual"])
    blocking = obstacles(incident)
    on_goal = [text for text in blocking["unusual"] if "stopping pose" in text and "goal to that waypoint" in text]
    if on_goal:
        waypoint = re.search(r"on the (\w+) stopping pose", on_goal[0]).group(1)
        return answer("blocked_station", waypoint, on_goal)
    mission = mission_summary(incident)
    if not mission["unusual"] and mission["ended"] == "complete":
        return answer("no_fault", "none", [])
    if [text for text in blocking["unusual"] if "obstacle not in the map" in text]:
        return answer("blocked_path", "route", blocking["unusual"])
    if any("commands were sent" in text for text in motion_status(incident)["unusual"]):
        return answer("actuator_fault", "drive", motion_status(incident)["unusual"])
    return answer("unknown", "", [])


# --- scoring -------------------------------------------------------------------------------------------------------

def specifics(truth):
    """Words the diagnosis should contain to count as naming the right thing, from the ground truth."""
    return [str(truth[key]).lower() for key in ("node", "parameter", "location") if key in truth] or (
        ["lidar"] if truth["category"] == "sensor_dropout" else
        ["amcl", "pose", "localiz"] if truth["category"] == "localization_lost" else [])


def score(diagnosis, truth):
    """Category must match exactly; naming the specific cause only counts when the category is right."""
    correct = diagnosis["category"] == truth["category"]
    wanted = specifics(truth)
    if not wanted:
        named = None
    elif not correct:
        named = False
    else:
        text = " ".join([diagnosis.get("component", ""), diagnosis.get("root_cause", "")]
                        + list(diagnosis.get("evidence", []))).lower()
        named = (any if truth["category"] == "localization_lost" else all)(word in text for word in wanted)
    return {"expected": truth["category"], "given": diagnosis["category"], "category_correct": correct,
            "named_the_specific_cause": named}


def scoreboard(runs):
    """A table of saved diagnoses: one row per scenario, one column per diagnoser, as correct/total."""
    cells, columns = {}, []
    for run in runs:
        for path in sorted(Path(run).glob("diagnosis-*.json")):
            saved = json.loads(path.read_text(encoding="utf-8"))
            label = saved.get("label", saved["model"])
            if label not in columns:
                columns.append(label)
            cells.setdefault((Path(run).parent.name, label), []).append(saved["score"])
    rows = sorted({scenario for scenario, _ in cells})

    def cell(scores):
        if not scores:
            return "-"
        named = [s["named_the_specific_cause"] for s in scores if s["named_the_specific_cause"] is not None]
        text = f"{sum(s['category_correct'] for s in scores)}/{len(scores)}"
        return text + (f" ({sum(named)}/{len(named)} specific)" if named else "")

    lines = ["| Scenario | " + " | ".join(columns) + " |", "| --- |" + " --- |"*len(columns)]
    for scenario in rows:
        lines.append(f"| {scenario} | " + " | ".join(cell(cells.get((scenario, c), [])) for c in columns) + " |")
    lines.append("| **all** | " + " | ".join(
        cell([s for (_, label), scores in cells.items() if label == c for s in scores]) for c in columns) + " |")
    return "\n".join(lines)
