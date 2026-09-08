# WarehouseBot reference audit

Source: [WarehouseBot-Pick-and-Drop-Optimization](https://github.com/ApurvK032/WarehouseBot-Pick-and-Drop-Optimization). Downloaded 7 September 2026 at commit `f54b393847f7c4929846c0824dc5b4986bd8194b`. The checkout is kept separately under `references/warehousebot/`, ignored by the new repository, and has no source modifications.

## What is reusable

| Module | Reuse plan |
| --- | --- |
| `astar.py` | Grid path/cost calculation; retain coordinate and obstacle semantics |
| `cost_matrix.py` | Distance estimates between current remaining stops |
| `warehouse.py` | Baseline fixtures; use explicit stable parcel IDs in the new application |
| `sequencing.py` | Greedy/local-search ideas and baseline comparison; adapt dependency handling |
| `evaluation.py` | Selected experiment utilities after checking failure accounting |
| `smoke_tests.py` | Original regression smoke checks, preserved unchanged |

Inspection found that the core, smoke tests, and evaluation module use the standard library. Visualization dependencies can be deferred.

## Adaptation requirements

1. `build_stops` always constructs both pickup and drop and renumbers parcels. Build new remaining-work records from authoritative state before translating them into temporary indices.
2. `is_route_feasible` derives the expected stop set from the route's own length. It cannot verify a missing onboard drop with no pickup precedence pair; missing paired indices can instead raise `KeyError`. The application needs its own expected-obligation set and structured errors.
3. `greedy_route` includes infinite-cost candidates. It can return a route containing an unreachable edge. Check reachability while selecting candidates and before committing a plan.
4. Greedy prerequisites use a dictionary from destination to one predecessor. Feeding multiple policy edges to it overwrites predecessors. Use prerequisite sets for priority and onboard-first constraints; do not assume the legacy precedence representation enforces them.
5. The existing offline evaluation skips failed trials in parts of its loop. New research metrics must retain all attempts with failure reasons and explicit denominators.

## Reproduction scope

Run `.venv/bin/python -m warehouse_agent baseline`. The wrapper refuses a different or dirty reference commit, runs all three original smoke tests, and runs the original static three-parcel instance with recorded settings. It independently checks the expected complete stop set, pickup/drop order, and finite route distance.

The saved evidence is in `docs/evidence/baseline.json`; runtime output goes to `artifacts/baseline/static.json`. Timings describe a single local run, not a benchmark. No prior percentage-improvement claims, full plot collection, or full report results have been independently reproduced in this task.

The reference README states academic coursework use. Preserve its attribution and existing terms; this starter does not assign a new license to the reference code. Select release licensing and record robot/model asset terms when packaging a public release.
