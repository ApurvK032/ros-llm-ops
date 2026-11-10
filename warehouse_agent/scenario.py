"""Fault-injection scenarios: run one headless mission, inject one fault at a set moment, record it, score it.

The runner only orchestrates processes (simulator, supervisor, recorder) and reads the supervisor's journal; it
never touches mission state. Scenario loading, triggers and outcome classification are pure functions, tested
without ROS.
"""

import json
import os
import shutil
import signal
import subprocess
import time
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

from .faults import FAULTS, FaultError, ros_env
from .verify import KNOWN_EVENTS, verify_file

ROOT = Path(__file__).resolve().parent.parent
MODEL_ENDPOINT = "http://127.0.0.1:11434"
NAV2_ACTIVE = "Managed nodes are active"
# Enough to replay and diagnose a run: motion, localization, sensing, planning, Nav2 health and mission status.
RECORD_TOPICS = ("/tf", "/tf_static", "/scan", "/scan_nav", "/odom", "/amcl_pose", "/cmd_vel", "/plan", "/rosout",
                 "/diagnostics", "/bond", "/behavior_tree_log", "/navigate_to_pose/_action/status",
                 "/global_costmap/costmap", "/local_costmap/costmap", "/particle_cloud", "/initialpose",
                 "/parameter_events", "/warehouse/status", "/warehouse/mission_status")
ABNORMAL = {"navigation_retry", "parcel_deferred", "planning_failed", "request_failed"}


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    command: str
    timeout_s: float
    fault: dict | None = None
    trigger: dict | None = None
    ground_truth: dict | None = None


def load_scenario(path, waypoints):
    """Read and validate a scenario file. Raises ValueError with the reason on any problem."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    unknown = set(data) - {"name", "description", "command", "timeout_s", "fault", "trigger", "ground_truth"}
    if unknown:
        raise ValueError(f"Unknown scenario keys: {', '.join(sorted(unknown))}")
    for key in ("name", "description", "command"):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f"Scenario needs a non-empty string {key!r}")
    timeout = data.get("timeout_s", 600)
    if not isinstance(timeout, int | float) or timeout <= 0:
        raise ValueError("timeout_s must be a positive number")
    fault, trigger = data.get("fault"), data.get("trigger")
    if (fault is None) != (trigger is None):
        raise ValueError("A scenario has both a fault and a trigger, or neither (a control run)")
    if fault is not None:
        if not isinstance(fault, dict) or fault.get("type") not in FAULTS:
            raise ValueError(f"fault.type must be one of {', '.join(FAULTS)}")
        FAULTS[fault["type"]][0](fault, waypoints)
        check_trigger(trigger)
        if not isinstance(data.get("ground_truth"), dict) or "category" not in data["ground_truth"]:
            raise ValueError("A fault scenario needs ground_truth with at least a 'category'")
    return Scenario(data["name"], data["description"], data["command"], float(timeout), fault, trigger,
                    data.get("ground_truth"))


def check_trigger(trigger):
    if not isinstance(trigger, dict) or set(trigger) - {"event", "count", "where", "delay_s"}:
        raise ValueError("trigger takes 'event', and optionally 'count', 'where' and 'delay_s'")
    if trigger.get("event") not in KNOWN_EVENTS:
        raise ValueError(f"trigger.event must be a journal event type, not {trigger.get('event')!r}")
    if not isinstance(trigger.get("count", 1), int) or trigger.get("count", 1) < 1:
        raise ValueError("trigger.count must be a positive integer")
    if not isinstance(trigger.get("delay_s", 0), int | float) or trigger.get("delay_s", 0) < 0:
        raise ValueError("trigger.delay_s must be zero or more")
    if not isinstance(trigger.get("where", {}), dict):
        raise ValueError("trigger.where must map dotted journal keys to required values")


def dig(event, dotted):
    value = event
    for part in dotted.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def trigger_time(trigger, events):
    """Monotonic time at which the fault is due, or None while the triggering event has not happened yet."""
    where = trigger.get("where", {})
    matches = [e for e in events if e.get("type") == trigger["event"]
               and all(dig(e, key) == value for key, value in where.items())]
    count = trigger.get("count", 1)
    if len(matches) < count:
        return None
    return matches[count-1]["monotonic"] + trigger.get("delay_s", 0)


def read_events(path):
    """Journal events written so far; a partly written last line is ignored."""
    events = []
    try:
        lines = Path(path).read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return events
    for line in lines:
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            break
    return events


def classify(events, agent_log="", injected_at=None):
    """Summarize how the mission ended, from the journal and the supervisor's console output alone."""
    by_type = {}
    for event in events:
        by_type.setdefault(event["type"], []).append(event)
    finished = (by_type.get("mission_finished") or [None])[-1]
    started = (by_type.get("instruction_applied") or [None])[0]
    last_state = next((e["state"] for e in reversed(events) if isinstance(e.get("state"), dict)), {})
    parcels = last_state.get("parcels", {})
    if finished:
        result = finished["result"]
    elif "Demo timed out" in agent_log:
        result = "timeout"
    elif "Mission held: no route" in agent_log:
        result = "no_route"
    elif "error:" in agent_log:
        result = "supervisor_error"
    elif not started:
        result = "not_started"
    else:
        result = "incomplete"
    end = finished or (events[-1] if events else None)
    outcome = {
        "result": result,
        "delivered": sorted(p for p, v in parcels.items() if v["state"] == "delivered"),
        "deferred": sorted(p for p, v in parcels.items() if v["disposition"] == "deferred"),
        "stranded_onboard": sorted(p for p, v in parcels.items() if v["state"] == "onboard"),
        "navigation_outcomes": _count(e.get("outcome") for e in by_type.get("navigation_finished", [])),
        "retries": len(by_type.get("navigation_retry", [])),
        "deferral_reasons": {e["parcel"]: e["reason"] for e in by_type.get("parcel_deferred", [])},
        "mission_seconds": round(end["monotonic"] - started["monotonic"], 1) if started and end else None,
    }
    error = next((line for line in agent_log.splitlines() if line.startswith("error:")), None)
    if error:
        outcome["supervisor_error"] = error[len("error:"):].strip()
    if injected_at is not None:
        impact = next((e for e in events if e["monotonic"] >= injected_at and (
            e["type"] in ABNORMAL or (e["type"] == "navigation_finished" and e.get("outcome") != "succeeded"
                                      and not e.get("cancellation_requested")))), None)
        outcome["seconds_to_first_abnormal_event"] = round(impact["monotonic"] - injected_at, 1) if impact else None
        outcome["first_abnormal_event"] = impact["type"] if impact else None
    return outcome


def _count(values):
    counts = {}
    for value in values:
        counts[value] = counts.get(value, 0) + 1
    return counts


# --- process orchestration (needs ROS, Gazebo and the local model) -------------------------------------------------

def _stop(process, grace=15):
    """Interrupt the process, then terminate its group, then kill it: Gazebo's server often ignores the first two.

    The interrupt goes to the leader only, so ros2 launch coordinates its own children; interrupting every child
    as well gives them a duplicate signal mid-shutdown.
    """
    if process is None or (process.poll() is not None and not _group_alive(process.pid)):
        return
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
        try:
            if sig == signal.SIGINT:
                os.kill(process.pid, sig)
            else:
                os.killpg(process.pid, sig)
        except ProcessLookupError:
            if not _group_alive(process.pid):
                return
            continue
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if process.poll() is not None and not _group_alive(process.pid):
                return
            time.sleep(0.2)


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    return True


def _model_ready():
    try:
        with urlopen(MODEL_ENDPOINT + "/api/tags", timeout=2) as response:
            return json.load(response)
    except (URLError, OSError, ValueError):
        return None


def ensure_model(model, log):
    """Use the running Ollama server, or start one. Returns the process to stop afterwards (None if not ours)."""
    process = None
    if _model_ready() is None:
        binary = shutil.which("ollama") or str(Path.home() / ".local/bin/ollama")
        process = subprocess.Popen([binary, "serve"], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 20
        while _model_ready() is None:
            if time.monotonic() > deadline or process.poll() is not None:
                _stop(process)
                raise RuntimeError("The local model server (ollama serve) did not start")
            time.sleep(0.25)
    names = {m["name"] for m in (_model_ready() or {}).get("models", [])}
    if model not in names:
        _stop(process)
        raise RuntimeError(f"Model {model} is not installed; run: ollama pull {model}")
    return process


def start_simulation(log_path, attempts=3):
    """Start the headless simulation and wait for Nav2. Nav2 sometimes aborts its own bringup; retry as demo.sh does."""
    # Every scenario run, including the control, goes through the scan relay so the runs are comparable.
    env = dict(os.environ, HEADLESS="True", USE_RVIZ="False", SHOW_MISSION_PANEL="False", SCAN_RELAY="1")
    for attempt in range(1, attempts+1):
        log = open(log_path, "w", encoding="utf-8")  # noqa: SIM115 - handed to the child process
        # Separate Nav2 processes so a single server can be crashed; a new session so the whole tree can be stopped.
        process = subprocess.Popen(["bash", "scripts/run_sim.sh", "use_composition:=False"], cwd=ROOT, env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 180
        aborted = False
        while time.monotonic() < deadline and process.poll() is None and not aborted:
            text = Path(log_path).read_text(encoding="utf-8", errors="replace")
            if any("lifecycle_manager_navigation" in line and NAV2_ACTIVE in line for line in text.splitlines()):
                return process
            aborted = "Aborting bringup" in text
            time.sleep(0.5)
        _stop(process)
        if not aborted or attempt == attempts:
            raise RuntimeError(f"Simulation not ready (attempt {attempt}); see {log_path}")
    raise RuntimeError("Simulation not ready")


def run_scenario(scenario, output, inject=True, record=True, model="qwen3.5:4b"):
    """Run one scenario end to end and write result.json into the output directory. Returns the result."""
    from .world import World
    world = World()
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    journal = output / "journal.jsonl"
    started_wall = datetime.now(UTC)
    injection = {"injected": False}
    sim = bag = agent = model_server = None
    timed_out = False
    try:
        with open(output / "model.log", "w", encoding="utf-8") as model_log:
            model_server = ensure_model(model, model_log)
        sim = start_simulation(output / "simulation.log")
        if record:
            topics = " ".join(RECORD_TOPICS)
            bag = subprocess.Popen(
                ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash && "
                 f"exec ros2 bag record -s mcap -o {output/'bag'} --topics {topics}"],
                cwd=ROOT, env=ros_env(), stdout=open(output / "recorder.log", "w"),  # noqa: SIM115
                stderr=subprocess.STDOUT, start_new_session=True)
        agent_log = open(output / "agent.log", "w", encoding="utf-8")  # noqa: SIM115
        agent = subprocess.Popen(["bash", "scripts/run_agent.sh", "--command", scenario.command, "--journal",
                                  str(journal), "--timeout", str(scenario.timeout_s), "--model", model],
                                 cwd=ROOT, stdout=agent_log, stderr=subprocess.STDOUT, start_new_session=True)
        hard_deadline = time.monotonic() + scenario.timeout_s + 180
        while agent.poll() is None:
            if time.monotonic() > hard_deadline:
                timed_out = True
                break
            if inject and scenario.fault and not injection["injected"] and "error" not in injection:
                due = trigger_time(scenario.trigger, read_events(journal))
                if due is not None and time.monotonic() >= due:
                    try:
                        details = FAULTS[scenario.fault["type"]][1](scenario.fault, world.waypoints)
                    except (FaultError, subprocess.TimeoutExpired) as exc:
                        injection["error"] = str(exc)
                    else:
                        # "monotonic" is when the fault was in place; the injector itself takes a few seconds.
                        injection = {"injected": True, "time": datetime.now(UTC).isoformat(),
                                     "monotonic": time.monotonic(), "type": scenario.fault["type"],
                                     "seconds_late": round(time.monotonic() - due, 1), "details": details}
            time.sleep(0.1)
    finally:
        _stop(agent, grace=20)
        _stop(bag, grace=10)
        _stop(sim)
        _stop(model_server, grace=5)
    events = read_events(journal)
    log_text = (output / "agent.log").read_text(encoding="utf-8", errors="replace") if agent else ""
    outcome = classify(events, log_text, injection.get("monotonic"))
    if timed_out:
        outcome["result"] = "timeout"
    applied = next((e for e in events if e["type"] == "instruction_applied"), None)
    if injection["injected"] and applied:
        injection["seconds_after_mission_start"] = round(injection["monotonic"] - applied["monotonic"], 1)
    report = verify_file(journal).as_dict() if journal.exists() else None
    result = {
        "scenario": asdict(scenario), "run": {"started": started_wall.isoformat(), "fault_requested": inject,
                                              "recorded": record, "model": model, "commit": _commit(),
                                              "supervisor_exit_code": agent.returncode if agent else None},
        "fault": injection, "outcome": outcome,
        "checker": None if report is None else {"ok": report["ok"], "events": report["events"],
                                                "violations": report["violations"],
                                                "warnings": report["warnings"]},
        "artifacts": {"journal": "journal.jsonl", "bag": "bag" if record else None,
                      "simulation_log": "simulation.log", "supervisor_log": "agent.log"},
    }
    (output / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result


def _commit():
    done = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True)
    return done.stdout.strip() or None


def summary_line(result):
    outcome, fault, checker = result["outcome"], result["fault"], result["checker"]
    injected = (f"{fault['type']} at +{fault.get('seconds_after_mission_start')} s" if fault["injected"]
                else ("fault NOT injected: " + fault["error"] if "error" in fault else "no fault"))
    checked = "no journal" if checker is None else f"{len(checker['violations'])} violations"
    return (f"{result['scenario']['name']}: {outcome['result']} | delivered {len(outcome['delivered'])}, "
            f"deferred {len(outcome['deferred'])}, onboard {len(outcome['stranded_onboard'])} | "
            f"{outcome['mission_seconds']} s | {injected} | checker: {checked}")
