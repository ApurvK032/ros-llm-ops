# WarehouseBot reference audit

Source: [WarehouseBot-Pick-and-Drop-Optimization](https://github.com/ApurvK032/WarehouseBot-Pick-and-Drop-Optimization), my earlier coursework project, at commit `f54b393847f7c4929846c0824dc5b4986bd8194b`. Its A* module is included unchanged as [warehouse_agent/astar.py](../warehouse_agent/astar.py). The full repository is only needed for the optional baseline reproduction below; `scripts/fetch_reference.sh` clones it into the ignored `references/warehousebot/`.

## What was reused

| Module | Use in this project |
| --- | --- |
| `astar.py` | Included unchanged as `warehouse_agent/astar.py` for task-level travel-cost estimates |
| `cost_matrix.py` | Not included; the planner computes and caches A* costs between the stops it needs |
| `warehouse.py` | Used only by the baseline reproduction |
| `sequencing.py` | Not included at runtime; `warehouse_agent/planner.py` implements greedy sequencing over live remaining stops with prerequisite sets. Hill climbing and simulated annealing run only in the baseline reproduction |
| `evaluation.py` | Not used |
| `smoke_tests.py` | Run unchanged by the baseline reproduction |

Inspection found that the core, smoke tests, and evaluation module use the standard library. Visualization dependencies can be deferred.

## Adaptation requirements

1. `build_stops` always constructs both pickup and drop and renumbers parcels. Build new remaining-work records from authoritative state before translating them into temporary indices.
2. `is_route_feasible` derives the expected stop set from the route's own length. It cannot verify a missing onboard drop with no pickup precedence pair; missing paired indices can instead raise `KeyError`. The application needs its own expected-obligation set and structured errors.
3. `greedy_route` includes infinite-cost candidates. It can return a route containing an unreachable edge. Check reachability while selecting candidates and before committing a plan.
4. Greedy prerequisites use a dictionary from destination to one predecessor. Feeding multiple policy edges to it overwrites predecessors. Use prerequisite sets for priority and onboard-first constraints; do not assume the legacy precedence representation enforces them.
5. The existing offline evaluation skips failed trials in parts of its loop. New research metrics must retain all attempts with failure reasons and explicit denominators.

## Reproduction scope

Run `bash scripts/fetch_reference.sh`, then `python3 -B -m warehouse_agent baseline`. The wrapper refuses a different or dirty reference commit, runs all three original smoke tests, and runs the original static three-parcel instance with recorded settings. It independently checks the expected complete stop set, pickup/drop order, and finite route distance.

The saved evidence is in `docs/evidence/baseline.json`; runtime output goes to `artifacts/baseline/static.json`. Timings describe a single local run, not a benchmark. No prior percentage-improvement claims, full plot collection, or full report results have been independently reproduced in this task.

Both projects have the same author. The included A* code is distributed under this repository's MIT [license](../LICENSE). Robot models and other simulation assets come from their installed ROS packages under their own terms.
