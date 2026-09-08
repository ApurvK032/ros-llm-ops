# Warehouse Agent: End-to-End Workflow

Planning outline — 7 September 2026. No implementation or machine setup has been performed as part of this document.

## 1. Project definition

Build a single-robot warehouse task supervisor that interprets natural-language requests, uses conventional planning algorithms, executes deliveries through ROS 2/Nav2 in Gazebo, and handles instructions that change during execution.

Research question: **Do execution-state checks improve the correctness of language-driven task updates, and what latency, interruption, and retry costs do they introduce?**

The LLM interprets intent and coordinates tools. The optimizer computes task sequences. Nav2 controls navigation. A deterministic state manager records what actually happened.

### Scope to freeze

- One existing differential-drive robot model and one small warehouse world.
- Three parcels initially; expand to six for evaluation.
- Known pickup/drop locations and a prebuilt map; no SLAM development.
- Logical pickup/drop operations at verified waypoints; no arm, grasping, or perception training.
- Text interface and terminal status; Gazebo/RViz provide the visual demo.
- Supported requests: create a delivery mission, prioritize an order, deliver onboard parcels first, cancel an uncollected order, and request status.
- Supported exceptions: an unreachable stop, an instruction arriving during execution, and an ambiguous cancellation involving onboard cargo.
- Out of scope: fleets, battery optimization, deadlines, capacity optimization, voice, a custom dashboard, fine-tuning, and new low-level navigation algorithms.

### Final demo

Start three deliveries. While the robot moves, request a priority change. Cancel an order near its pickup completion. The supervisor checks the latest state, preserves onboard cargo, requests clarification if needed, and executes an accepted revision. Introduce one unreachable destination; the system reports the failure without inventing a delivery. End with a report checked against the event log.

## 2. Where to build

**Development target: Ubuntu 24.04, directly or through WSL 2.**

Run the small simulation and language model locally. Measure rendering and inference requirements with the demo before choosing hardware for larger experiments. A remote GPU server is optional; the project does not depend on access to one.

### Recommended environment

- Add a separate Ubuntu 24.04 WSL 2 distribution for this project; keep the existing Ubuntu 26.04 distribution intact.
- Use ROS 2 Jazzy, Gazebo Harmonic, and matching Nav2 packages. This is a documented compatible combination. Newer ROS releases exist; this choice deliberately follows the established Jazzy examples. Do not install Jazzy's Ubuntu 24.04 packages into Ubuntu 26.04. [Gazebo compatibility guide](https://gazebosim.org/docs/harmonic/ros_installation/), [ROS supported release platforms](https://www.ros.org/blog/getting-started/).
- Keep the repository and build files inside WSL's Linux filesystem, such as `~/ros-llm-ops`, rather than `/mnt/c/`. [Microsoft filesystem guidance](https://learn.microsoft.com/en-us/windows/wsl/filesystems).
- Run ROS, Gazebo, the executor, and agent in the same WSL distribution initially. Do not start with cross-machine ROS networking.
- Use the ROS distribution's compatible system Python for ROS-facing code. Isolate extra Python dependencies without replacing that interpreter or using `sudo pip`.
- Start with one tool-capable hosted LLM behind a replaceable client interface. Use bounded requests and a spending limit. Local inference is a later deployment option, not an MVP prerequisite.
- Record resolved package versions, Python version, model identifier, prompts, and generation settings once the first demo works.

WSLg supports Linux GUI applications and accelerated OpenGL, but this does not guarantee every Gazebo rendering configuration works. Check both the GUI and a headless run early. [Microsoft WSLg documentation](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).

For local CUDA inference, use the NVIDIA Windows driver exposed into WSL; do not install a Linux NVIDIA display driver inside WSL. [NVIDIA CUDA on WSL guide](https://docs.nvidia.com/cuda/wsl-user-guide/index.html).

### When to use the server

Use a suitable GPU server for an optional larger-model comparison or approved batch runs. Keep simulation local unless a remote/headless workflow is demonstrably useful. If serving a model remotely, use a private authenticated connection or SSH tunnel; do not expose an unauthenticated endpoint. Respect other users' jobs and obtain an agreed resource allocation.

Shared-GPU contention and network delay are experimental confounders. Record them and report end-to-end deployment latency separately from model inference latency. Do not silently switch model providers during a benchmark.

If WSL rendering remains unreliable after a bounded troubleshooting session, continue core work in the grid backend and evaluate a supported native-Linux setup. Do not erase or repartition the Windows installation as a troubleshooting shortcut.

## 3. System boundaries

| Module | Owns | Must not own |
| --- | --- | --- |
| LLM supervisor | Intent interpretation, tool selection, clarification, explanations | Wheel commands or authoritative physical state |
| State manager | Parcel state, instruction revisions, active action, accepted plan, event log | Invented success inferred from an LLM response |
| Planner | Travel estimates and valid remaining-task sequences | Silent changes to user obligations |
| Validator | IDs, task invariants, proposal freshness, update acceptance | Claims of universal physical safety |
| Executor | Ordered actions, acknowledgements, retries, pause/cancel handling | Unrestricted execution of generated code |
| Grid backend | Fast repeatable logical motion and task transitions | Claims of physics-level realism |
| ROS/Nav2 backend | Navigation requests and actual action feedback | High-level policy decisions |
| Evaluator | Independent scoring from scenarios and execution events | Trusting the agent's final self-assessment |

Both execution backends should expose the same small interface: navigate, poll action, cancel action, perform logical pickup/drop, and read status.

## 4. Phased implementation

### Phase 0 — Define behavior before coding

- [ ] Write five example missions and five mid-mission changes.
- [ ] Define parcel IDs, named locations, and the exact meaning of each supported request.
- [ ] Define conflicting priorities: deliver-onboard-first overrides a new pickup priority; ask if the user gives incompatible hard requirements.
- [ ] Define cancellation: uncollected orders can be cancelled; onboard parcels require clarification and an explicitly accepted disposition. Never delete their physical state.
- [ ] Define an unreachable-stop policy: bounded navigation recovery, then defer or ask according to the accepted mission policy. A timeout is not proof that a location is permanently unreachable.
- [ ] Define when a revision becomes effective: after acknowledgement at an executor-controlled action boundary, not merely when the LLM emits text.

**Finish check:** each example has an unambiguous expected outcome or expected clarification.

### Phase 1 — Establish the environment and reproduce the old project

- [ ] Configure the dedicated WSL environment described above.
- [ ] Run an unmodified, version-matched Nav2 robot simulation; send a goal, observe arrival, and test cancellation.
- [ ] Verify Gazebo/RViz rendering, simulation clock, localization, and a headless run. Record any rendering fallback rather than assuming CUDA visibility proves graphics acceleration.
- [ ] Clone the existing WarehouseBot repository on the development PC. Preserve the original project and record its starting commit before extending it.
- [ ] Run the existing smoke tests and a small reference experiment. Save outputs; do not assume prior README results were reproduced.
- [ ] Separate reusable core functions from experiment/plotting entry points.

The reusable source is [WarehouseBot](https://github.com/ApurvK032/WarehouseBot-Pick-and-Drop-Optimization): A* costs, stop construction, greedy sequencing, hill climbing, simulated annealing, and experiment utilities. Its current implementation is an offline single-robot planner, not an execution engine.

Follow the **Jazzy-and-newer** section of the [Nav2 quickstart](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/); the page also contains older Gazebo Classic instructions.

**Finish check:** the stock robot navigates successfully, and the old grid planner runs from the documented environment. Do not build a custom world yet.

### Phase 2 — Build authoritative state and a grid executor

- [ ] Represent each parcel's physical lifecycle: awaiting pickup, onboard, delivered. Keep cancellation/deferment as explicit mission status rather than erasing physical state.
- [ ] Track current pose, active action, accepted instruction revision, active plan, and completed actions.
- [ ] Assign unique IDs to instructions, proposals, plans, actions, and events.
- [ ] Log state transitions with both logical/simulation time and wall-clock time.
- [ ] Add a step-based grid executor with action duration, arrival, pickup, drop, cancellation, and injected navigation failures.
- [ ] Record completion only after an executor acknowledgement. Handle duplicate events idempotently.
- [ ] Ensure resets fully separate test episodes.

Use a monotonic **semantic state revision** for relevant changes: pickup/drop, accepted instruction, active-action transition, or map/reachability update. Do not invalidate every proposal just because odometry ticks or time advances.

**Finish check:** without any LLM, a scripted mission completes and the event log can reconstruct its final state. Duplicate acknowledgements do not duplicate pickups or deliveries.

### Phase 3 — Adapt planning to unfinished work

- [ ] Build the remaining stop list from current state, rather than rebuilding the original mission blindly.
- [ ] Remove completed stops; preserve drop obligations for onboard parcels; exclude accepted cancellations.
- [ ] Add explicitly supported ordering constraints for priority and onboard-first requests. The existing distance objective does not implement these policies automatically.
- [ ] Start with greedy sequencing; enable hill climbing after correctness passes. Keep simulated annealing as an optional comparison, not another implementation prerequisite.
- [ ] Recompute costs when relevant map information changes. Reject infinite-cost paths rather than passing them to execution.
- [ ] Create a validator that checks against the independent expected remaining-stop set, not only the length or contents of the proposed route.
- [ ] Produce a plan ID, state revision, ordered actions, estimated distance, and explicit unresolved/deferred jobs.
- [ ] Keep proposing a plan separate from committing it.

Check pickup-before-drop, already-loaded exceptions, no duplicate completion, valid IDs/locations, unchanged obligations, accepted priorities, and reachability assumptions. Recheck relevant state atomically before commit. If incompatible, return a structured rejection and refreshed state; allow only bounded retry.

**Finish check:** scripted priority changes, cancellations, loaded-parcel cases, and stale proposals either produce a correct plan or a specific rejection. No LLM is needed to pass these tests.

### Phase 4 — Add the tool-using LLM

Implement these application tools; these names describe proposed code, not existing library functions:

| Tool | Purpose |
| --- | --- |
| `get_state()` | Obtain a compact authoritative snapshot and instruction revision |
| `propose_plan(change_request, expected_revision)` | Run the planner and return a candidate or structured errors |
| `commit_plan(plan_id, expected_revision)` | Revalidate and queue an accepted update through the executor |
| `get_execution_status()` | Inspect progress, failures, and unresolved obligations |
| `ask_operator(question)` | Resolve ambiguity without making up an answer |

- [ ] Describe supported IDs, tools, policies, and output schemas to the model.
- [ ] Validate arguments, bound tool-call count/retries/time, and reject unknown operations.
- [ ] Use a fake model client for deterministic tool-loop tests before using a paid endpoint.
- [ ] Add the real model client, with explicit timeout and failure behavior.
- [ ] Keep an explicit stop/pause control outside the LLM. An API outage must not disable it.
- [ ] If the LLM fails, stop accepting revisions and retain only already-authorized behavior under the executor's documented policy.
- [ ] Make the agent react to instructions and meaningful events, not every navigation feedback tick.
- [ ] Generate status reports from recorded evidence; do not treat free-form explanation as ground truth.

**Finish check:** natural-language missions and updates work end to end in the grid backend, including one clarification and one failure. Invalid model output cannot bypass the executor.

**Milestone A: working agent prototype.**

### Phase 5 — Connect the same system to ROS 2/Gazebo

- [ ] Keep the stock robot and add only a simple warehouse layout and named pickup/drop waypoints.
- [ ] Make the 2D planning map and Gazebo map consistent, including scale, frame IDs, and obstacle inflation for the robot footprint.
- [ ] Implement the backend using Nav2 navigation goals and action results. Keep the existing A* layer as a high-level distance estimator; Nav2 remains responsible for actual paths and motion.
- [ ] Verify frame transforms, localization, and `use_sim_time` throughout the ROS nodes.
- [ ] Perform simulated pickup/drop only after navigation success and a configured positional tolerance check. State clearly that cargo handling is logical, not simulated grasping.
- [ ] Allow at most one active navigation action. When changing it, wait for cancellation/termination acknowledgement before issuing the replacement.
- [ ] Keep ROS event handling responsive while an LLM request is outstanding.
- [ ] Introduce one obstacle/failure and feed the actual navigation outcome back into the supervisor. Do not leak future injected events or evaluator-only world state to the agent.
- [ ] Save application traces, relevant ROS topics, and a screen recording.

Nav2 provides non-blocking navigation, feedback, results, and cancellation through its [Simple Commander API](https://docs.nav2.org/jazzy/configuration_and_development/simple_commander_api/simple_commander_api/). The application still owns task-state synchronization.

**Finish check:** normal delivery, a mid-mission update, and an unreachable-stop scenario run in Gazebo without source-code edits between runs. The final report agrees with recorded events.

**Milestone B: complete ROS/Gazebo MVP. Freeze features here before adding research experiments.**

### Phase 6 — Run the research study

- [ ] Define and freeze the pilot scenarios and scoring rules before tuning on test results.
- [ ] Start with 30 scenarios, for example five cases in each of six categories: normal, priority update, pre-pickup cancellation, onboard cancellation, navigation failure, and delayed/stale update.
- [ ] Separate development cases from held-out layouts and instruction wording. Record the split.
- [ ] Implement three modes in one codebase: scripted recovery after LLM parsing, feedback-driven full replanning, and state-checked incremental plan updates.
- [ ] Share the same navigation stack, scenario information, optimizer, model settings, and mandatory low-level execution guards across modes.
- [ ] Define exactly which supervision checks differ; distinguish task-validation ablations from physical stop/navigation protections, which stay enabled.
- [ ] Repeat held-out runs to expose model variability; report counts and uncertainty, not just a best run. Expand beyond the pilot if conclusions remain uncertain.
- [ ] Test state freshness with controlled response delays, recording injected and naturally observed delays separately.
- [ ] During a delay, permit only the already-approved action to progress. Never advance into an unvalidated revised action queue.
- [ ] Compare the same mode with and without the proposed freshness mechanism to isolate its effect.
- [ ] Use the grid backend for the larger matrix. Run a predeclared representative Gazebo subset, including failure cases, rather than only successful demos.

Primary outcomes: correctness against the latest accepted instruction and preservation of task obligations. Secondary outcomes: invalid proposals rejected, invalid updates executed, recovery success, needless clarifications, route distance, waiting time, retries, tool calls, tokens, and wall-clock latency.

Use an evaluator independent of the agent and, where practical, separate from the runtime validator. Validate against scenario-defined expected obligations and executor events. Break results down by scenario category. Compare distance against the same conventional planner with correctly specified constraints; do not label hill-climbing results globally optimal.

Control simulation speed when studying latency. Report simulation time, wall time, real-time factor, hardware, runtime versions, and provider/network conditions. Fast grid runs and Gazebo runs are separate evaluation settings, not interchangeable latency measurements.

Existing research includes [ROS-LLM](https://arxiv.org/abs/2406.19741), [Inner Monologue](https://innermonologue.github.io/), and [VerifyLLM](https://arxiv.org/abs/2507.05118). Review these before claiming novelty. The proposed contribution is a specific implementation and evaluation of execution-aware updates, not the first LLM-controlled robot.

**Finish check:** one documented benchmark command produces traceable results, plots, and a failure analysis for every comparison mode, including negative findings.

### Phase 7 — Package the result

- [ ] Write a README with prerequisites, exact setup, supported commands, demo steps, and limitations.
- [ ] Include a compact architecture explanation, example tool traces, and instructions to reproduce the study.
- [ ] Provide a small scenario set, model/prompt configuration, machine-readable results, and selected replayable traces without credentials.
- [ ] Add automated core tests; document Gazebo integration tests separately. Do not promise GPU/GUI CI on an ordinary runner.
- [ ] Test setup from a clean project environment without changing the working installation.
- [ ] Produce a short demo and a paper-style report: question, related work, method, experiment design, results, and limitations.
- [ ] Attribute reused WarehouseBot, Nav2, robot models, and other dependencies accurately.
- [ ] Update the resume only with measured outcomes, separating task-planning work from Nav2's navigation capabilities.

**Milestone C: research-oriented portfolio release.**

## 5. Optional deployment study — only after Milestone B

Serve a suitably sized quantized tool-capable LLM on the RTX 4070 and test it with the exact same tool schemas. Leave GPU headroom for rendering; model fit depends on context, quantization, runtime, and concurrent workloads. Do not choose a model merely because its weights nominally fit in 12 GB.

Measure application success, argument validity, memory, and latency before considering larger hardware. Use the server for a larger-model comparison only if useful. Keep model/inference failures visible; no hidden fallback during evaluation. Running inference on an NVIDIA GPU is deployment experience, not evidence of custom CUDA or distributed GPU programming.

## 6. Planned code organization

These are suggested modules to add later, not files created by this outline:

- `domain`: parcel/task schemas, state transitions, revisions, event log.
- `planning`: adapters around existing search and sequencing algorithms.
- `validation`: preconditions, preserved obligations, update acceptance.
- `agent`: model adapter, tool definitions, bounded orchestration loop.
- `execution`: common executor plus grid and ROS/Nav2 backends.
- `simulation`: maps, named waypoints, worlds, and launch configurations.
- `evaluation`: scenario loading, perturbations, independent scoring, reports.
- `tests`: unit, grid integration, and separately marked Gazebo tests.
- `docs`: environment setup, demo script, experimental protocol, limitations.

Prefer this small Python application plus ROS packages over microservices. Keep the core independent of ROS so most tests run without a simulator.

## 7. Planning budget and immediate next actions

Rough planning allowance, not a delivery guarantee: 40–65 focused hours for the working MVP and another 25–40 hours for a research-oriented release, assuming reuse of the old planner and stock robot. At 10–12 hours per week, allow approximately 6–10 weeks; environment problems or a more rigorous study may extend this. Re-estimate after Phase 1.

If time is limited, freeze at three parcels, one world, one model endpoint, one navigation-failure case, and a small pilot study. Defer local inference comparisons, more algorithms, and visual polish.

### First two work sessions

1. Establish the separate Ubuntu 24.04/Jazzy/Harmonic environment and pass the stock Nav2 navigation/cancellation smoke test.
2. Reproduce WarehouseBot, record its baseline, write the mission/cancellation policies, and implement the parcel-state tests before adding an LLM.

### Completion checklist

- [ ] Fresh setup instructions work.
- [ ] A single robot completes deliveries in Gazebo.
- [ ] Natural-language task revisions cause valid, observed execution changes.
- [ ] Completed work and onboard cargo survive replanning.
- [ ] Stale/incompatible updates are rejected or revised before commit.
- [ ] Unreachable and ambiguous tasks produce honest outcomes.
- [ ] The independent evaluator checks the final report and obligations.
- [ ] Comparison runs and limitations are documented.
- [ ] The demo and repository run without the shared server.

This roadmap replaces the earlier GPU-benchmark-agent concept for planning purposes. The older document has not been modified or removed.
