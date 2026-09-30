# MVP status — 8 September 2026

> **Archived planning document.** Kept for history; it describes plans from before the system was built, not the current behavior. See the [design](../design.md) and [results](../results.md) for the current system.

The project prioritizes a complete local-model simulation pipeline before broader robustness and research extensions.

**The maze update is implemented:** an 18 × 14 m warehouse with seven shelf sections, separate parcel and approach positions, and position/heading/facing/clearance checks for pickup and drop. A full run completed all six goals in 123.044 seconds. All six transfers also passed checks against Gazebo's actual robot pose; the smallest conservative actual clearance was 0.422 m. The 22 tests pass on Python 3.12 and 3.14. See the [layout guide](../warehouse-layout.md) and [new run evidence](../evidence/maze/README.md).

The first expanded-scene run exposed an AMCL pose jump after a turn. The current simulation configuration uses a narrower motion-noise model, more laser beams/particles, and more frequent localization updates. The successful rerun's maximum observed localization position error was 0.148 m. These are development observations rather than a reliability claim.

Earlier MVP history follows; its timings refer to the original smaller warehouse.

**Stage 1 is implemented:** station labels, live parcel boxes and current-goal highlights in Gazebo/RViz, plus a read-only mission panel with cargo state and remaining stops. The fresh-HOME acceptance run delivered all three parcels in 63.779 seconds, with six successful navigation goals and a maximum arrival error of 0.2379 m. All 15 core, CLI and visual-state tests passed on Python 3.14 and the ROS environment's Python 3.12. See [stage 1 guide](../visuals.md), [results](../evidence/stage1/results.json), [scene](../evidence/stage1/gazebo-delivered.png) and [mission panel](../evidence/stage1/panel-onboard.png).

Next is stage 2: an operator interface for submitting instructions and pausing/resuming the mission. Startup/shutdown cleanup and repeatable scenarios follow. The stage 1 panel remains read-only; the CLI provides the existing controls.

**Remote access is ready:** a noVNC desktop now exposes the actual simulation, mission panel, RViz and CLI through localhost port 6080 and an SSH tunnel. A real browser login and keyboard-entered delivery passed all six navigation goals in 63.171 seconds. See the [connection guide](../browser-access.md) and [browser evidence](../evidence/browser/browser-test.json).

Original MVP implementation and verification:

- Separate Ubuntu-24.04 WSL distribution with ROS 2 Jazzy, Nav2, Gazebo Harmonic, RViz and Ollama 0.33.3.
- Local Qwen3.5 4B model; CUDA inference and NVIDIA D3D12 rendering verified.
- Custom warehouse geometry with matching occupancy/planning maps, automatic AMCL initialization and a stock TurtleBot3.
- Natural-language intent interpretation, original WarehouseBot A* reuse, remaining-work greedy planning, authoritative cargo state, asynchronous Nav2 execution and event journals.
- A full Gazebo mission: 3 pickups, 3 deliveries, 6 successful navigation actions. The first successful run took 83.039 seconds from the prior test's end pose; a subsequent fresh-HOME run took 63.080 seconds with a maximum arrival error of 0.228 m.
- A live update integration check: pause acknowledgement, resume, model-driven cancellation during navigation, priority, status, onboard-first, and delivery of the remaining two parcels. The cancelled parcel stayed uncollected.
- Eleven core and CLI tests passed on Python 3.14 and the ROS environment's Python 3.12, including unsuccessful arrival validation, cargo ordering, cancellation races, priority and bounded navigation failure handling.

- A second full delivery run from a fresh HOME pose passed through the combined launcher, which exited with code 0 and closed its Gazebo/RViz processes. See [clean-start journal](../evidence/gazebo-clean-start.jsonl).

Evidence: [results](../evidence/mvp-results.json), [delivery journal](../evidence/gazebo-deliveries.jsonl), [live-update journal](../evidence/gazebo-live-updates.jsonl). These are successful development checks, not a reliability benchmark.

The source workspace remains in Ubuntu-26.04. `scripts/wsl.sh` copies it into the ROS distribution before running. Use the [README](../../README.md) and [MVP runbook](../runbook.md) for current commands. Earlier planning documents remain future design references.

Not implemented: physical grasping/perception, multi-robot control, full proposal transactions, event replay tooling, crash resume, returns, arbitrary destination updates and formal research evaluation. Navigation failure behavior has core-test coverage; systematic Gazebo failure injection remains later work.

The original offline WarehouseBot baseline still reproduces greedy 30 cells, hill climbing 27 and simulated annealing 27 on its unchanged static grid. This is separate from the metric Gazebo warehouse. The reference is pinned at `f54b393847f7c4929846c0824dc5b4986bd8194b`.

Known remaining issue: Gazebo/RViz/Nav2 report native errors during shutdown on this WSL setup. The launcher returns and removes its processes, but native shutdown is not clean. Successful mission and cargo results were recorded before shutdown. The follow-up startup/quit check exits 0 with no simulator processes left.
