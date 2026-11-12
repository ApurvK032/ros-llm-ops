"""Run every scenario several times against each decision-maker, and turn the results into rates.

One run is an anecdote. A batch runs the same scenarios repeatedly with nobody attached ("none"), with the fixed
playbook ("rules") and with each local model, in rounds, so a batch that is interrupted still holds the same
number of runs of everything. It can be started again and continues where it stopped.

Judging a run and summarizing a batch are pure functions over result files, tested without ROS.
"""

import json
import os
import signal
import statistics
import time
import traceback
from pathlib import Path

from .recovery import LIDAR_NAMES

# Actions that change nothing about the robot's configuration or the mission plan.
FREE = {"clear_costmaps", "escalate"}
LABELS = {"none": "Supervisor alone", "rules": "Playbook"}


def slug(text):
    return "".join(c if c.isalnum() else "_" for c in text)


def normalize(action, target):
    """One spelling per action, so what a model wrote can be compared with what a scenario expects."""
    words = (target or "").split()
    if action == "restart_process":
        return action, "lidar_driver" if " ".join(words) in LIDAR_NAMES else " ".join(words)
    if action == "restore_parameter" and len(words) == 2:
        return action, "/" + words[0].lstrip("/") + " " + words[1]
    if action in ("relocalize", "clear_costmaps", "escalate"):
        return action, ""
    return action, " ".join(words)


def judge(result):
    """What one run achieved, against what its scenario says a correct response achieves."""
    scenario, outcome = result["scenario"], result["outcome"]
    expected = scenario.get("expected") or {}
    faults = [result["fault"], *result.get("additional_faults", [])]
    wanted_faults = (1 + len(scenario.get("additional") or [])) if scenario.get("fault") \
        and result["run"]["fault_requested"] else 0
    valid = sum(bool(fault.get("injected")) for fault in faults) == wanted_faults
    check = result.get("pose_check") or {}
    # Without a recording of the true pose the journal's word is all there is; say so rather than guess.
    confirmed = check.get("delivered_verified") if check.get("samples") else None
    delivered = len(outcome["delivered"]) if confirmed is None else len(confirmed)
    recovery = result.get("recovery") or {}
    incidents = recovery.get("incidents", [])
    carried = [normalize(a["action"], a.get("target", "")) for incident in incidents for a in incident["actions"]
               if a["permitted"] and "error" not in a]
    needed = {normalize(*pair) for pair in expected.get("actions", [])}
    acceptable = {normalize(*pair) for pair in expected.get("acceptable", [])}
    checker = result.get("checker") or {}
    violations = len(checker.get("violations", []))
    success = None
    if valid and "delivered" in expected:
        success = (delivered >= expected["delivered"] and not outcome["stranded_onboard"] and violations == 0
                   and checker.get("ok") is not False)
    return {
        "scenario": scenario["name"], "decider": recovery.get("mode", "none"), "valid": valid, "success": success,
        "delivered": delivered, "delivered_claimed": len(outcome["delivered"]),
        "pose_checked": confirmed is not None, "result": outcome["result"],
        "mission_seconds": outcome["mission_seconds"],
        "fault_to_resume": recovery.get("seconds_fault_to_resume"),
        "held_seconds": round(sum(i.get("seconds", 0.0) for i in incidents), 1),
        "incidents": len(incidents),
        "diagnoses": [i["diagnosis"]["category"] for i in incidents if "diagnosis" in i],
        "carried_out": [list(pair) for pair in carried],
        "unneeded": [list(pair) for pair in carried
                     if pair not in needed and pair not in acceptable and pair[0] not in FREE],
        "missing": [list(pair) for pair in sorted(needed - set(carried))],
        "refused": recovery.get("actions_refused", 0), "failed": recovery.get("actions_failed", 0),
        "violations": violations,
    }


def summarize(judgements):
    """One row per scenario and decider: how often it worked, how long it took, what it did that it need not."""
    groups = {}
    for item in judgements:
        groups.setdefault((item["scenario"], item["decider"]), []).append(item)
    rows = []
    for (scenario, decider), items in groups.items():
        valid = [item for item in items if item["valid"]]
        won = [item for item in valid if item["success"]]

        def middle(key, pool):
            values = [item[key] for item in pool if item[key] is not None]
            return round(statistics.median(values), 1) if values else None

        rows.append({
            "scenario": scenario, "decider": decider, "runs": len(valid), "invalid": len(items) - len(valid),
            "successes": len(won), "delivered": [item["delivered"] for item in valid],
            # Times are for the runs that worked: a mission abandoned early is short, not fast.
            "median_mission_seconds": middle("mission_seconds", won),
            "median_fault_to_resume": middle("fault_to_resume", won),
            "interruptions": sum(item["incidents"] for item in valid),
            "unneeded_actions": sum(len(item["unneeded"]) for item in valid),
            "runs_with_unneeded_actions": sum(bool(item["unneeded"]) for item in valid),
            "runs_missing_a_needed_action": sum(bool(item["missing"]) for item in valid),
            "actions_refused": sum(item["refused"] for item in valid),
            "actions_failed": sum(item["failed"] for item in valid),
            "violations": sum(item["violations"] for item in valid),
            "not_pose_checked": sum(not item["pose_checked"] for item in valid),
        })
    return rows


def table(rows, deciders=None, scenarios=None):
    """Markdown: success rate and median mission time, then what each decider did beyond what was needed."""
    by_cell = {(row["scenario"], row["decider"]): row for row in rows}
    deciders = deciders or list(dict.fromkeys(row["decider"] for row in rows))
    scenarios = scenarios or list(dict.fromkeys(row["scenario"] for row in rows))
    header = "| Scenario | " + " | ".join(LABELS.get(d, d) for d in deciders) + " |"
    rule = "| --- |" + " --- |"*len(deciders)

    def outcome(row):
        if row is None or not row["runs"]:
            return "-"
        seconds = f", {row['median_mission_seconds']:g} s" if row["median_mission_seconds"] is not None else ""
        return f"{row['successes']} of {row['runs']}{seconds}"

    def extras(row):
        if row is None or not row["runs"] or row["decider"] == "none":
            return "-"
        return (f"{row['interruptions']/row['runs']:.1f} per run; {row['unneeded_actions']} unneeded in "
                f"{row['runs_with_unneeded_actions']} of {row['runs']}")

    lines = ["Runs that achieved the expected result, and their median mission time:", "", header, rule]
    lines += [f"| `{s}` | " + " | ".join(outcome(by_cell.get((s, d))) for d in deciders) + " |" for s in scenarios]
    totals = []
    for decider in deciders:
        cells = [row for row in rows if row["decider"] == decider and row["scenario"] in scenarios]
        totals.append(f"**{sum(r['successes'] for r in cells)} of {sum(r['runs'] for r in cells)}**")
    lines.append("| **all** | " + " | ".join(totals) + " |")
    acting = [d for d in deciders if d != "none"]
    if acting:
        lines += ["", "Interruptions of the mission, and actions carried out that the scenario did not need:", "",
                  "| Scenario | " + " | ".join(LABELS.get(d, d) for d in acting) + " |",
                  "| --- |" + " --- |"*len(acting)]
        lines += [f"| `{s}` | " + " | ".join(extras(by_cell.get((s, d))) for d in acting) + " |" for s in scenarios]
    return "\n".join(lines)


def collect(directory):
    """Judge every finished run under a batch directory."""
    return [judge(json.loads(path.read_text(encoding="utf-8")))
            for path in sorted(Path(directory).glob("*/*/run-*/result.json"))]


def plan(names, deciders, repeats, baseline_repeats=1):
    """Runs in the order they are made: round by round, so every cell grows at the same pace."""
    return [(repeat, name, decider) for repeat in range(1, repeats+1) for name in names for decider in deciders
            if decider != "none" or repeat <= baseline_repeats]


# --- running (needs ROS, Gazebo and the local model) ---------------------------------------------------------------

def stray_processes(domain="42"):
    """This user's processes on the simulation's ROS domain, other than this process and its ancestors.

    A run that ends badly can leave a simulator or a Nav2 server behind, and the next run would then share a
    ROS domain with it. Only processes whose own environment names the domain are listed.
    """
    mine, pid = set(), os.getpid()
    while pid > 1:
        mine.add(pid)
        try:
            pid = int(Path(f"/proc/{pid}/stat").read_text().rsplit(") ", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
    found, marker = [], f"ROS_DOMAIN_ID={domain}".encode()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) in mine:
            continue
        try:
            if entry.stat().st_uid == os.getuid() and marker in (entry / "environ").read_bytes().split(b"\0"):
                found.append(int(entry.name))
        except OSError:
            continue
    return found


def clear_strays(wait=5.0):
    """Stop whatever an earlier run left behind. Returns how many processes had to be stopped."""
    first = stray_processes()
    for sig in (signal.SIGTERM, signal.SIGKILL):
        remaining = stray_processes()
        if not remaining:
            break
        for pid in remaining:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + wait
        while stray_processes() and time.monotonic() < deadline:
            time.sleep(0.25)
    return len(first)


def run_batch(files, deciders, repeats, output, baseline_repeats=1, model="qwen3.5:4b", passes=2, log=print):
    """Make every run in the plan that has no result yet. A run that fails to start is tried once more at the end."""
    from .scenario import load_scenario, run_scenario, summary_line
    from .world import World
    waypoints = World().waypoints
    scenarios = {s.name: s for s in (load_scenario(path, waypoints) for path in files)}
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    # What was asked for, kept with the results: the report reads the column order from here.
    (output / "batch.json").write_text(json.dumps({
        "scenarios": list(scenarios), "deciders": list(deciders), "repeats": repeats,
        "baseline_repeats": baseline_repeats, "model": model}, indent=1) + "\n", encoding="utf-8")
    todo = plan(list(scenarios), deciders, repeats, baseline_repeats)
    for attempt in range(1, passes+1):
        for repeat, name, decider in todo:
            directory = output / name / slug(decider) / f"run-{repeat:02d}"
            if (directory / "result.json").exists():
                continue
            if directory.exists():
                # Left by a run that was interrupted or never started; keep it for the record, out of the way.
                directory.rename(directory.with_name(f"{directory.name}-incomplete-{int(time.time())}"))
            strays = clear_strays()
            started = time.monotonic()
            try:
                result = run_scenario(scenarios[name], directory, record=False, model=model, watch=True,
                                      recover=None if decider == "none" else decider)
            except Exception as exc:  # noqa: BLE001 - one bad run must not end an unattended batch
                directory.mkdir(parents=True, exist_ok=True)
                (directory / "error.txt").write_text(traceback.format_exc(), encoding="utf-8")
                log(f"[pass {attempt}] {name} / {decider} / run {repeat}: did not run: {exc}")
                continue
            log(f"[pass {attempt}] {decider} / run {repeat} / {round(time.monotonic() - started)} s wall"
                f"{f' / {strays} stray processes cleared first' if strays else ''}: {summary_line(result)}")
    clear_strays()
    missing = [(repeat, name, decider) for repeat, name, decider in todo
               if not (output / name / slug(decider) / f"run-{repeat:02d}" / "result.json").exists()]
    log(f"Batch finished: {len(todo) - len(missing)} of {len(todo)} runs have results.")
    return missing


def report(directory, deciders=None):
    """Summary of a batch directory as Markdown, also saved beside the runs with the per-run judgements.

    Columns follow the order the batch was asked for, unless another is given.
    """
    directory = Path(directory)
    asked = directory / "batch.json"
    if deciders is None and asked.exists():
        deciders = json.loads(asked.read_text(encoding="utf-8"))["deciders"]
    judged = collect(directory)
    rows = summarize(judged)
    order = [d for d in (deciders or ["none", "rules"]) if any(r["decider"] == d for r in rows)]
    order += [d for d in dict.fromkeys(r["decider"] for r in rows) if d not in order]
    text = table(rows, order, sorted({r["scenario"] for r in rows}))
    (directory / "summary.json").write_text(json.dumps({"rows": rows, "runs": judged}, indent=1) + "\n",
                                            encoding="utf-8")
    (directory / "summary.md").write_text(text + "\n", encoding="utf-8")
    return text
