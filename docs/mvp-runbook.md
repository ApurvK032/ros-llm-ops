# MVP runbook

## Prepared environment

The development setup can keep its source checkout in one WSL distribution and run ROS in a separate `Ubuntu-24.04` distribution. The runtime contains ROS 2 Jazzy, Nav2, Gazebo Harmonic, RViz and Ollama 0.33.3. The wrapper uses the current shell's username by default and discovers that user's runtime home directory; see [configuration](environment.md#wsl-wrapper-configuration) for overrides. Local account names and paths are not part of the project configuration.

`bash scripts/wsl.sh sync` copies current sources and the pinned original planner to the runtime copy. It does not copy `.git`, virtual environments or artifacts. Edit in the original workspace. Package installation and models persist in Ubuntu-24.04 across restarts. No ROS packages were installed into Ubuntu-26.04.

Qwen3.5 4B is downloaded under the runtime user's `.ollama` directory. The observed model digest is `2a654d98e6fba55d452b7043684e9b57a947e393bbffa62485a7aac05ee4eefd`, quantization Q4_K_M. CUDA inference was verified. Gazebo/RViz use WSLg's D3D12 renderer, explicitly selected in `scripts/run_sim.sh`.

ROS runs on domain 42 with local discovery. Ollama listens on `127.0.0.1:11434`. No externally hosted model or credential is needed.

## Everyday operation

For SSH access from another computer, use the [browser desktop](browser-access.md). It runs the actual GUI applications on a private virtual display and streams them through forwarded port 6080.

From the original workspace, with no warehouse simulation already running:

```bash
bash scripts/wsl.sh demo
```

Wait for `Nav2 and localization ready`, then type `Deliver all three parcels`. The first model request after a cold start can take about a minute; later requests in the observed run took about a second. `/status` reports application state; `/pause` stops navigation through Nav2 cancellation without waiting for the model; `/resume` continues. `/quit` exits and the combined launcher stops its simulation processes.

To execute and automatically exit after a delivery mission:

```bash
bash scripts/wsl.sh demo --command 'Deliver all three parcels'
bash scripts/wsl.sh logs
```

For separate terminals:

```bash
bash scripts/wsl.sh model
bash scripts/wsl.sh sim
bash scripts/wsl.sh agent
```

Run each command in its own terminal. Reuse an existing model server or simulator. Exactly one mission supervisor should control the robot. In this MVP each supervisor process starts a fresh logical mission; there is no crash recovery. Starting a new supervisor leaves existing localization intact. Restart the simulator to return the physical robot to HOME.

To work directly in the runtime distribution, use Windows Terminal with:

```powershell
wsl -d Ubuntu-24.04 --cd "~"
```

Change into your runtime checkout (`cd ~/ros-llm-ops` for the default wrapper target), then use `bash scripts/demo.sh`, or the individual `run_model.sh`, `run_sim.sh`, `run_agent.sh` scripts.

## Model and planner boundaries

The local model emits one typed intent: create, prioritize, onboard_first, cancel, status, pause, resume or clarify. The application validates parcel IDs and cargo state. Model-generated explanatory text is not execution evidence. The model does not choose coordinates, emit velocity commands or generate executable code.

The task planner reuses A* from the pinned WarehouseBot repository. It builds a fresh remaining-stop list from current cargo state and applies a greedy sequence with prerequisites. Its grid has 0.2 m cells and a 0.35 m obstacle margin. Nav2 does the actual motion planning using a 0.05 m occupancy map and its own costmaps. Both maps come from the same metric wall/shelf geometry.

The current maze uses separate parcel storage positions and robot approach poses. Logical pickup/drop requires a successful NavigateToPose result, position within 0.20 m of the approach, heading error at most 0.22 rad, facing error toward the parcel at most 0.25 rad, and conservative parcel clearance of at least 0.20 m. See the [layout guide](warehouse-layout.md). Pause/cancel requests wait for the current action's terminal result before replacement. Cargo is held while language interpretation is pending. Navigation gets one application retry, then the parcel becomes deferred and its cargo state stays visible.

The automated tests cover core behavior with a backend double as well as static warehouse geometry. The delivery and live-update journals in `docs/evidence/` came from actual Nav2/Gazebo execution. These are distinct verification levels. Earlier `stage1` runs used the original smaller layout and looser position tolerance. The [stage 1 guide](stage1-visuals.md) describes the live displays and their validation.

## Capturing and inspecting a run

Episodes are append-only JSONL files under the runtime copy's `artifacts/episodes`. Each records the initial world, input text, model result/latency, selected routes, navigation results, cargo changes and final state. `bash scripts/wsl.sh logs` copies them into `artifacts/ros/artifacts/` in the original workspace.

From the runtime directory with the simulator GUI running:

```bash
source /opt/ros/jazzy/setup.bash
python3 -B -m warehouse_agent.capture_scene
```

This frames the warehouse and saves a real Gazebo screenshot into `artifacts/captures`. The ROS status topic is `/warehouse/status` (`std_msgs/String` containing JSON). To inspect it, use the same domain and discovery settings as the launch scripts.

## Resolved setup issues

- Nav2 originally timed out before the agent supplied an initial pose. The generated Nav2 parameters now set HOME during AMCL startup, and the adapter checks the navigator lifecycle is active before accepting work.
- A slow callback loop initially read stale TF poses. The adapter now drains high-rate callbacks on each tick; successful arrival measurements are recorded in the evidence.
- WSLg initially selected software rendering. The launcher supplies `/usr/lib/wsl/lib` and the D3D12/NVIDIA Mesa settings verified on this machine.
- Stopping the new WSL distribution can remove the shared Windows executable handler. The wrapper now falls back to WSL's `/init` interop launcher when the registration is absent. No distribution reset or reinstall is required.

The first map message can emit an RViz GLSL sampler diagnostic, also tracked in [RViz issue 463](https://github.com/ros2/rviz/issues/463). It did not block the verified navigation missions.

If ROS discovery commands show only `/rosout`, check `ROS_DOMAIN_ID=42` and use `ros2 node list --no-daemon --spin-time 3`; a stale CLI daemon can show an old graph. If goals are rejected, check the navigation lifecycle and simulation log before rerunning a mission. The adapter no longer treats a merely discoverable action server as ready.

## Remaining shutdown issue

The launcher now resets inherited background interrupt handling, signals the ROS launch parent, and removes any remaining GUI subprocess. Startup followed by `/quit` exits with code 0 and leaves no simulator processes. Gazebo, RViz and the Nav2 container still report native shutdown errors on this WSL setup. This is a known unresolved integration issue; it occurs after the successful mission results. The [recorded launcher checks](evidence/mvp-results.json) summarize the outcome. Raw logs with local environment details are kept out of the public repository.

## Scope after this MVP

The user chose simulation first and research later. Stage 1 adds station labels, live cargo visualization and a read-only mission panel. The next stage is an operator interface for submitting instructions and controlling the mission, followed by startup/shutdown cleanup and repeatable scenarios. Full proposal transactions, event replay, crash resume, dynamic order insertion, configurable destinations, returns, manipulation, perception and multi-robot control remain unimplemented. The earlier research documents are proposals, not claims of features already built.

Installation references: [ROS Jazzy on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), [Nav2 simulation](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/), [Ollama Linux installation](https://docs.ollama.com/linux), [Qwen3.5 4B package](https://ollama.com/library/qwen3.5:4b), [WSL systemd integration](https://wsl.dev/technical-documentation/systemd/).
