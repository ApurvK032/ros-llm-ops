# Results

Everything measured so far, with links to the raw evidence. These are **individual development runs**, not a reliability benchmark: each Gazebo result below is a single recorded run. A fault-injection campaign with repeated trials is on the [roadmap](../README.md#roadmap).

## Current warehouse (18 × 14 m shelf maze)

`Deliver all three parcels` from a fresh HOME pose, 8 September 2026:

| Measure | Result |
| --- | --- |
| Outcome | Complete: 3 pickups, 3 drops, 6/6 Nav2 goals succeeded, no retries |
| Time after the intent was accepted | 123.044 s |
| Model interpretation time (local Qwen3.5 4B) | 6.6 s |
| Largest approach-position error at transfer, supervisor's TF pose | 0.049 m (limit 0.20 m) |
| Largest approach-position error at transfer, **Gazebo ground truth** | 0.132 m (limit 0.20 m) |
| Largest facing error toward the parcel, ground truth | 0.177 rad / 10.1° (limit 0.25 rad) |
| Smallest conservative parcel clearance, ground truth | 0.422 m (minimum 0.20 m) |
| Largest localization error observed during the run | 0.148 m, 0.088 rad |

The ground-truth figures come from a read-only observer that compared each state change on `/warehouse/status` with Gazebo's actual robot pose; all six transfers passed the same checks the supervisor uses. The first attempt in this larger layout ended with deferred parcels after an AMCL pose jump during a turn. The localization settings were then tuned, and that failed attempt is not counted here.

Evidence: [results](evidence/maze/results.json) · [journal](evidence/maze/delivery.jsonl) · [notes](evidence/maze/README.md) · [before](evidence/maze/maze-ready.png) / [after](evidence/maze/maze-delivered.png)

### Re-run after the correctness-core refactor (30 September 2026)

The same mission ran headless (no GUI) on the refactored supervisor (explicit state machines, request IDs) to confirm the changes on the real stack, not only against the test double:

| Measure | Result |
| --- | --- |
| Outcome | Complete: 3 pickups, 3 drops, 6/6 Nav2 goals, no retries; launcher exit 0 |
| Time after the intent was accepted | 124.1 s |
| Model interpretation time | 5.6 s |
| Largest approach-position error at transfer, supervisor's TF pose | 0.088 m (limit 0.20 m) |
| Largest facing error | 0.057 rad (limit 0.25 rad) |
| Smallest conservative parcel clearance | 0.486 m (minimum 0.20 m) |
| Journal checker | 0 violations, 0 warnings; request R1 requested → interpreted → applied |

The first launch attempt that day failed before any mission started: Nav2 aborted its own bringup, most likely because the WSL2 clock was stepped during startup. The launcher now retries automatically (see the [runbook](runbook.md#wsl2-clock-jumps-can-stop-nav2)). It is reported here rather than omitted. Evidence: [journal](evidence/maze/post-m1-delivery.jsonl).

## Original 10 × 8 m warehouse

These runs used the earlier, smaller layout and a position-only arrival check (0.35 m tolerance), so they are not directly comparable with the current layout.

| Run | Result | Evidence |
| --- | --- | --- |
| First end-to-end delivery (7 Sep), starting from a previous test's end pose | Complete, 6/6 goals, 83.039 s; model 0.78 s | [results](evidence/mvp-results.json) · [journal](evidence/gazebo-deliveries.jsonl) |
| Combined launcher, fresh HOME | Complete, 6/6 goals, 63.080 s; largest arrival error 0.228 m; launcher exited 0 with all simulator processes closed | [results](evidence/mvp-results.json) · [journal](evidence/gazebo-clean-start.jsonl) |
| Live visuals, fresh HOME (8 Sep) | Complete, 6/6 goals, 63.779 s; largest arrival error 0.238 m; every parcel went waiting → onboard → delivered on screen | [results](evidence/stage1/results.json) · [journal](evidence/stage1/delivery.jsonl) · [notes](evidence/stage1/README.md) |
| Operated through the browser desktop over SSH | Complete, 6/6 goals, 63.171 s; no browser errors; stop command cleaned up every process | [results](evidence/browser/browser-test.json) · [journal](evidence/browser/delivery.jsonl) |
| Mid-mission updates through the model | Pause acknowledged with no pickup; a cancel during navigation left the parcel on its shelf; priority, status and onboard-first applied; the remaining two parcels were delivered. Warm model interpretation: 0.62–0.89 s | [journal](evidence/gazebo-live-updates.jsonl) |
| Headless launch | Nav2 and localization ready with every window disabled; a late subscriber still received all markers | [results](evidence/stage1/results.json) |

A cold model load took about a minute on the development machine; later requests took about a second.

## Offline planner baseline

The original WarehouseBot planner on its static 8 × 8 three-parcel grid still reproduces greedy 30, hill climbing 27, and simulated annealing 27 grid cells, each route checked independently for a complete stop set, pickup-before-drop order, and finite cost. This is separate from the Gazebo warehouse. Evidence: [baseline](evidence/baseline.json).

## Automated tests

76 unit tests plus property-based tests run without ROS, a GPU, or a model: mission behavior against a backend double, the state machines (every transition from every state), CLI control flow, visual state, warehouse geometry, start-cell snapping, A*, the request lifecycle, and the journal checker (8 scripted scenarios, 21 tampered journals). The Hypothesis property tests run 400 random sessions of up to 80 steps and 150 liveness runs. CI runs everything on Python 3.12 (ROS 2 Jazzy's interpreter) and 3.14 for every pull request, along with lint. Gazebo runs are not part of CI yet.

**Journal checker on real runs:** all 21 recorded Gazebo journals pass (the 7 committed here plus 14 local development runs, including 3 runs with retries and 8 deferrals).

**Mutation check:** 8 of 8 planted supervisor bugs are caught by the property tests, each by the rule it breaks (`scripts/mutation_check.py`).

## Known issue

On the WSL development setup, Gazebo, RViz and the Nav2 container print native errors while shutting down. The launcher still exits 0 and leaves no simulator processes, and all results above were recorded before shutdown. See the [runbook](runbook.md#remaining-shutdown-issue).
