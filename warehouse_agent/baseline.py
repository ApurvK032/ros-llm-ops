"""Run the unchanged reference in a subprocess and verify its static routes."""

from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import subprocess
import sys

REFERENCE_COMMIT = "f54b393847f7c4929846c0824dc5b4986bd8194b"

# This is fixed application code, never model-generated code.
EXPERIMENT = r'''
import json
import math
import time
from warehousebot.warehouse import make_static_demo, build_stops
from warehousebot.cost_matrix import compute_cost_matrix
from warehousebot.sequencing import greedy_route, hill_climb, simulated_annealing, route_cost

demo = make_static_demo()
stops = build_stops(demo["start"], demo["parcels"])
matrix = compute_cost_matrix(demo["grid"], stops["coords"])["matrix"]
settings = {
    "seed": 42,
    "hill_climb": {"max_iters": 1000, "neighbors_per_iter": 30, "restarts": 5},
    "simulated_annealing": {"max_iters": 2000, "t0": 50.0, "alpha": 0.995},
}
algorithms = {
    "greedy": lambda: greedy_route(matrix, stops["precedence"]),
    "hill_climb": lambda: hill_climb(matrix, stops["precedence"], seed=42,
                                    **settings["hill_climb"])["best_route"],
    "simulated_annealing": lambda: simulated_annealing(matrix, stops["precedence"], seed=42,
                                    **settings["simulated_annealing"])["best_route"],
}
results = {}
for name, run in algorithms.items():
    started = time.perf_counter()
    route = run()
    elapsed = time.perf_counter() - started
    # Expected stops come from the instance, independently of the proposed route.
    expected = set(range(len(stops["names"])))
    valid = bool(route) and route[0] == 0 and len(route) == len(expected) and set(route) == expected
    if not valid:
        raise RuntimeError(f"{name}: omitted, duplicate, or unknown stop")
    positions = {stop: index for index, stop in enumerate(route)}
    if any(positions[pickup] >= positions[drop] for pickup, drop in stops["precedence"]):
        raise RuntimeError(f"{name}: invalid pickup/drop order")
    cost = route_cost(route, matrix)
    if not math.isfinite(cost):
        raise RuntimeError(f"{name}: unreachable segment")
    results[name] = {"route": route, "stop_names": [stops["names"][i] for i in route],
                     "distance_grid_cells": cost, "seconds": elapsed,
                     "independent_route_check": "passed"}
print(json.dumps({"settings": settings, "results": results}, allow_nan=False))
'''


def _run(args: list[str], cwd: Path) -> str:
    try:
        result = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"Reference command exceeded 120 seconds: {args[0]}") from exc
    if result.returncode:
        raise RuntimeError((result.stdout + result.stderr).strip())
    return result.stdout


def reproduce(reference: Path) -> dict:
    reference = reference.resolve()
    if not (reference / "warehousebot" / "smoke_tests.py").is_file():
        raise ValueError("Reference checkout missing; run bash scripts/fetch_reference.sh")
    commit = _run(["git", "rev-parse", "HEAD"], reference).strip()
    if commit != REFERENCE_COMMIT:
        raise ValueError(f"Expected reference {REFERENCE_COMMIT}; found {commit}")
    dirty = _run(["git", "status", "--porcelain", "--untracked-files=all"], reference).strip()
    if dirty:
        raise ValueError("Reference checkout has changes; preserve a clean baseline before reproduction")
    # Do not resolve the venv's python symlink: that would bypass its environment.
    interpreter = str(Path(sys.executable).absolute())
    smoke = _run([interpreter, "-B", "-m", "warehousebot.smoke_tests"], reference)
    experiment = json.loads(_run([interpreter, "-B", "-c", EXPERIMENT], reference))
    return {
        "observed_at_utc": datetime.now(timezone.utc).isoformat(),
        "reference_commit": commit,
        "python": platform.python_version(),
        "scenario": "original_static_8x8_three_parcels",
        "scope": "Offline planner only; no executor, LLM, Gazebo, or full paper reproduction",
        "smoke_tests": {"status": "passed", "stdout": smoke},
        **experiment,
    }
