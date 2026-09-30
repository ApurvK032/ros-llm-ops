# Visual mission feedback

The simulation shows logical cargo transfers in Gazebo and RViz, alongside a native **Warehouse mission** status window. Start it with `bash scripts/demo.sh` (or `bash scripts/wsl.sh demo` in the two-distribution WSL setup) and enter `Deliver all three parcels`.

The current [maze layout](warehouse-layout.md) places parcels on shelves and delivery pedestals. Ground circles and arrows show separate robot approach poses and facing directions. The historical screenshots and timings below describe the original smaller layout.

## What to watch

| Display | Meaning |
| --- | --- |
| Station captions | HOME and all pickup/drop station names, plus parcel IDs and physical state where present |
| Amber parcel | Waiting at its pickup station |
| Blue parcel | Confirmed onboard; follows the robot's reported pose |
| Green parcel | Confirmed delivered at its drop station |
| Gray / red parcel | Cancelled / deferred; its physical location is retained |
| Cyan ring and caption | Current navigation goal |
| Mission window | Current goal, parcel states/locations, remaining stops, pause and connection status |

Cargo moves only after Nav2 reports success and the supervisor verifies the approach position, heading, facing direction, and clearance. Driving near a parcel does not transfer it. There is no arm, attachment joint, parcel collision geometry or perception step: the parcels are visual representations of application cargo state. Shelves and delivery pedestals have collision geometry.

The remaining-stop list comes from the executor's selected plan and excludes the current goal. A priority or mission change hides the superseded queue until the next action boundary refreshes it. The viewer does not calculate a competing route or send commands to the robot.

Closing the mission window only hides the read-only display. Use the agent CLI's `/pause` and `/quit` for operation. The stock Nav2 launch shuts the simulation down when RViz closes; keep RViz open during a mission. A disconnected viewer marks the state as last known and removes the current-goal highlight rather than reporting a new delivery.

## Launch options and recording

From the original source workspace:

```bash
bash scripts/wsl.sh sim show_panel:=False
```

This hides the mission panel while keeping Gazebo and RViz. For a fully headless simulator:

```bash
bash scripts/wsl.sh sim headless:=True use_rviz:=False show_panel:=False
```

Run the model and agent in separate terminals when using these simulator-only commands. The viewer still publishes `/warehouse/markers` for an RViz client in headless mode, and does not start the Gazebo marker transport.

To record panel screenshots and authoritative snapshots, open a shell in **Ubuntu-24.04**, then run:

```bash
cd /path/to/ros-llm-ops
WAREHOUSE_VISUAL_RECORD_DIR=artifacts/visuals bash scripts/demo.sh
```

Each cargo, goal, pause or session-close transition creates matching `.json` and `.png` files. The JSON includes the received mission snapshot, projected markers and Gazebo connection state. This is a development evidence recorder, not a video or crash-recovery mechanism. Use `bash scripts/wsl.sh logs` from the original workspace to copy artifacts back.

For a real Gazebo screenshot while the GUI is running, use `python3 -B -m warehouse_agent.capture_scene` in the runtime shell after sourcing `/opt/ros/jazzy/setup.bash`.

## Implementation notes

`/warehouse/status` remains the supervisor's authoritative JSON snapshot. The separate viewer consumes it and publishes transient-local `/warehouse/markers` messages for RViz, sends markers through a persistent native Gazebo transport helper, and updates the Qt panel at 10 Hz. A bounded latest-frame queue keeps slow GUI requests outside the navigation loop.

The native helper uses the Gazebo C++ libraries already shipped with ROS Jazzy. `scripts/run_sim.sh` builds it when missing or stale. The installer includes CMake, a C++ compiler and PyQt5; these are installed in the prepared runtime distribution.

Gazebo's Ogre2 renderer does not implement TEXT markers, so its labels use triangle geometry facing the initial demo camera. They may be less readable after orbiting behind them; the capture command restores the demo view. RViz uses its normal camera-facing text markers. The goal ring uses a geometry ribbon in Gazebo so its width remains visible. These renderer choices follow the [Ogre2 marker implementation](https://github.com/gazebosim/gz-rendering/blob/gz-rendering8/ogre2/src/Ogre2Marker.cc).

The RViz config loads a generated copy of the installed TurtleBot URDF with corrected mesh paths. It uses RViz's supported [file description source](https://github.com/ros2/rviz/blob/jazzy/rviz_default_plugins/src/rviz_default_plugins/displays/robot_model/robot_model_display.cpp); installed ROS assets and robot control geometry are unchanged.

## Validation

Four visual behavior tests cover acknowledgement-gated pickup, carried/delivered placement, cancelled parcel retention, deferred onboard retention, and remaining-stop consistency after priority changes. The original stage 1 suite contained 15 tests; the current suite adds approach, orientation, clearance, and maze-geometry checks.

The fully headless launch also reached Nav2/localization readiness. A late subscriber received the three parcel boxes and seven station captions from the transient-local ROS marker topic with no GUI required.

The fresh-HOME acceptance run delivered all three parcels across six successful navigation goals in 63.779 seconds. Snapshot checks confirmed each parcel's waiting → onboard → delivered sequence and an empty final goal/queue. Results and screenshots are recorded in [stage 1 evidence](evidence/stage1/README.md). Native Gazebo/RViz/Nav2 shutdown errors on this WSL setup remain a separate known issue; see the [runbook](runbook.md#remaining-shutdown-issue).
