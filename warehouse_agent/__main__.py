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
    args = parser.parse_args()
    try:
        if args.subcommand == "run":
            return run(args)
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
