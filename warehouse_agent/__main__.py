"""Warehouse robot supervisor, environment inspection and planner reproduction."""

import argparse
import json
from pathlib import Path

from .baseline import reproduce
from .doctor import inspect_environment
from .verify import verify_file


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="subcommand", required=True)
    doctor = sub.add_parser("doctor", help="Report observed environment readiness")
    doctor.add_argument("--output", type=Path)
    baseline = sub.add_parser("baseline", help="Reproduce the pinned offline planner")
    baseline.add_argument("--reference", type=Path, default=Path("references/warehousebot"))
    baseline.add_argument("--output", type=Path, default=Path("artifacts/baseline/static.json"))
    verify = sub.add_parser("verify", help="Check mission journals against the supervisor's rules")
    verify.add_argument("journals", type=Path, nargs="+")
    verify.add_argument("--json", action="store_true", help="Print full machine-readable reports")
    from .cli import add_arguments, run
    add_arguments(sub.add_parser("run", help="Run the local-model ROS/Nav2 mission supervisor"))
    scenario = sub.add_parser("scenario", help="Run a headless fault-injection scenario and score the result")
    scenario.add_argument("file", type=Path, help="Scenario JSON, for example scenarios/blocked_station.json")
    scenario.add_argument("--no-fault", action="store_true", help="Control run: same mission, fault not injected")
    scenario.add_argument("--no-record", action="store_true", help="Skip the MCAP recording")
    scenario.add_argument("--model", default="qwen3.5:4b")
    scenario.add_argument("--recover", metavar="WHO", help="Attach the recovery agent: 'rules' for the fixed "
                          "playbook, or a local model name such as qwen3.5:9b")
    scenario.add_argument("--film", action="store_true", help="Put a camera in the world and save its frames")
    scenario.add_argument("--output", type=Path, help="Result directory (default: artifacts/scenarios/<name>/<time>)")
    diagnose = sub.add_parser("diagnose", help="Diagnose recorded scenario runs and score against the injected fault")
    diagnose.add_argument("runs", type=Path, nargs="+", help="Run directories written by 'scenario'")
    diagnose.add_argument("--model", default="qwen3.5:4b")
    diagnose.add_argument("--rules", action="store_true", help="Use the fixed rule-based baseline, no model")
    diagnose.add_argument("--no-tools", action="store_true", help="Give the model the mission report only")
    diagnose.add_argument("--repeat", type=int, default=1, help="Diagnose each run this many times")
    board = sub.add_parser("scoreboard", help="Tabulate saved diagnoses by scenario and diagnoser")
    board.add_argument("runs", type=Path, nargs="+")
    batch = sub.add_parser("batch", help="Run scenarios repeatedly against each decision-maker; resumes if stopped")
    batch.add_argument("files", type=Path, nargs="+", help="Scenario files")
    batch.add_argument("--output", type=Path, required=True, help="Batch directory, for example artifacts/batch/a")
    batch.add_argument("--deciders", nargs="+", default=["none", "rules"], metavar="WHO",
                       help="'none' (supervisor alone), 'rules' (playbook) and local model names")
    batch.add_argument("--repeats", type=int, default=5)
    batch.add_argument("--baseline-repeats", type=int, default=1, help="Runs of each scenario with nobody attached")
    batch.add_argument("--model", default="qwen3.5:4b", help="The model that interprets the operator's request")
    summary = sub.add_parser("batch-report", help="Summarize a batch directory as rates and median times")
    summary.add_argument("directory", type=Path)
    summary.add_argument("--deciders", nargs="+", metavar="WHO", help="Column order (default: as the batch ran)")
    args = parser.parse_args()
    try:
        if args.subcommand == "scoreboard":
            from .diagnosis import scoreboard
            print(scoreboard(args.runs))
            return 0
        if args.subcommand == "batch":
            return run_batch_files(args)
        if args.subcommand == "batch-report":
            from .batch import report
            print(report(args.directory, args.deciders))
            return 0
        if args.subcommand == "run":
            return run(args)
        if args.subcommand == "scenario":
            return run_scenario_file(args)
        if args.subcommand == "diagnose":
            return diagnose_runs(args)
        if args.subcommand == "verify":
            return verify_journals(args.journals, args.json)
        result = inspect_environment() if args.subcommand == "doctor" else reproduce(args.reference)
    except (OSError, RuntimeError, ValueError) as exc:
        parser.exit(1, f"error: {exc}\n")
    rendered = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


def run_scenario_file(args):
    from datetime import UTC, datetime

    from .scenario import ROOT, load_scenario, run_scenario, summary_line
    from .world import World
    scenario = load_scenario(args.file, World().waypoints)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    suffix = "-control" if args.no_fault and scenario.fault else ""
    if args.recover:
        suffix += "-recover-" + "".join(c if c.isalnum() else "_" for c in args.recover)
    output = args.output or ROOT / "artifacts/scenarios" / scenario.name / (stamp + suffix)
    result = run_scenario(scenario, output, inject=not args.no_fault, record=not args.no_record, model=args.model,
                          recover=args.recover, film=args.film)
    print(summary_line(result))
    print(f"Result: {output / 'result.json'}")
    # A fault that could not be injected makes the run invalid; a failed mission is a valid result.
    return 1 if any("error" in fault for fault in [result["fault"], *result["additional_faults"]]) else 0


def run_batch_files(args):
    import signal

    from .batch import report, run_batch
    from .scenario import _stop, ensure_model
    # A batch started in the background of a script inherits an ignored interrupt, and would pass that on to
    # every process it starts: none of them could then be stopped the gentle way.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    args.output.mkdir(parents=True, exist_ok=True)
    # One model server for the whole batch, so models stay loaded from run to run.
    with open(args.output / "model.log", "a", encoding="utf-8") as model_log:
        server = ensure_model(args.model, model_log)
        try:
            missing = run_batch(args.files, args.deciders, args.repeats, args.output, args.baseline_repeats,
                                args.model, log=lambda text: print(text, flush=True))
        finally:
            _stop(server, grace=5)
    print(report(args.output, args.deciders))
    return 1 if missing else 0


def diagnose_runs(args):
    import subprocess

    from .diagnosis import Incident, diagnose, rule_based, score
    from .scenario import ROOT, _stop, ensure_model
    for run in args.runs:
        if not (run / "evidence.json").exists():
            # Reading the bag needs ROS; everything after this does not.
            subprocess.run(["bash", "-c", "source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash && "
                            f"python3 -B -m warehouse_agent.evidence '{run.resolve()}'"], cwd=ROOT, check=True)
    label = "rules" if args.rules else args.model + (" (no tools)" if args.no_tools else "")
    server = None
    if not args.rules:
        with open("/dev/null", "w") as quiet:
            server = ensure_model(args.model, quiet)
    rows = []
    try:
        for run in args.runs:
            incident = Incident(run)
            for attempt in range(1 if args.rules else args.repeat):
                outcome = rule_based(incident) if args.rules else diagnose(incident, args.model,
                                                                         use_tools=not args.no_tools)
                outcome["score"] = score(outcome["diagnosis"], incident.truth)
                outcome["label"] = label
                rows.append((run, outcome))
                name = "diagnosis-" + "".join(c if c.isalnum() else "_" for c in label) + f"-{attempt+1}.json"
                (run / name).write_text(json.dumps(outcome, indent=2) + "\n", encoding="utf-8")
                marks = outcome["score"]
                specific = {True: "yes", False: "no", None: "-"}[marks["named_the_specific_cause"]]
                print(f"{'OK  ' if marks['category_correct'] else 'MISS'} {run.parent.name:18} expected "
                      f"{marks['expected']:17} got {marks['given']:17} specific cause named: {specific:3} "
                      f"tools: {len(outcome['tool_calls'])}  {outcome['seconds']} s")
    finally:
        _stop(server, grace=5)
    right = sum(o["score"]["category_correct"] for _, o in rows)
    named = [o["score"]["named_the_specific_cause"] for _, o in rows if o["score"]["named_the_specific_cause"]
             is not None]
    print(f"{label}: category correct {right}/{len(rows)}; specific cause named {sum(named)}/{len(named)}")
    return 0


def verify_journals(paths, as_json):
    reports = [verify_file(path) for path in paths]
    if as_json:
        print(json.dumps([r.as_dict() for r in reports], indent=2))
    else:
        for r in reports:
            print(f"{'OK  ' if r.ok else 'FAIL'} {r.path}: {r.events} events, {len(r.violations)} violations, "
                  f"{len(r.warnings)} warnings")
            for f in r.violations:
                print(f"     violation #{f.sequence} {f.code}: {f.message}")
            for f in r.warnings:
                print(f"     warning {f.code}: {f.message}")
    return 0 if all(r.ok for r in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())
