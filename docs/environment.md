# Development environment

The simulation uses **Ubuntu 24.04, ROS 2 Jazzy, Nav2, Gazebo Harmonic, and Python 3.12**. Core tests also run on Python 3.14 without ROS. The project runs Qwen3.5 4B locally through Ollama. See the [README](../README.md#quick-start) for the main setup sequence.

This guide describes reproducible software setup. Personal account names, workstation inventories, and raw environment probes are excluded from the public repository.

## Core development

From a checkout with Python 3.12+ and Git:

```bash
bash scripts/fetch_reference.sh
python3 -B -m unittest discover -s tests -v
python3 -B -m warehouse_agent baseline
python3 -B -m warehouse_agent doctor
```

The optional `bash scripts/bootstrap_core.sh` creates a dependency-free core virtual environment. ROS launch scripts should use Ubuntu 24.04's system Python so that apt-installed ROS packages remain importable.

The doctor command reports the local environment. Its output can contain local paths and device information; keep reports in ignored `artifacts/` when collecting diagnostics.

## Ubuntu 24.04 and ROS

Use an Ubuntu 24.04 installation or an Ubuntu 24.04 WSL2 distribution. If adding the latter, run in Windows PowerShell:

```powershell
wsl --list --verbose
wsl --install --distribution Ubuntu-24.04
```

Complete the distribution's Linux user setup. Clone the repository into its Linux filesystem and run the installer from that checkout:

```bash
bash scripts/install_ros_jazzy.sh
bash scripts/install_ros_jazzy.sh --apply
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix nav2_bringup
ros2 pkg prefix nav2_minimal_tb3_sim
```

The installer checks for Ubuntu 24.04, configures the official ROS apt source, and installs Jazzy desktop, Nav2, the stock TurtleBot3 simulation, ROS/Gazebo integration, Qt, and development tools. Install Ollama using its [Linux guide](https://docs.ollama.com/linux), then follow the README to launch the demo. Browser mode additionally requires `bash scripts/install_browser.sh --apply` and a downloaded model.

References: [ROS installation source](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst), [Gazebo compatibility](https://gazebosim.org/docs/harmonic/ros_installation/), [Microsoft WSL commands](https://learn.microsoft.com/en-us/windows/wsl/basic-commands).

## WSL wrapper configuration

`scripts/wsl.sh` is useful when the source checkout and ROS runtime live in different WSL distributions. It copies source changes into the runtime checkout before launching. Edit the source checkout and treat the runtime checkout as a generated working copy.

| Environment variable | Default | Purpose |
| --- | --- | --- |
| `WAREHOUSE_WSL_DISTRO` | `Ubuntu-24.04` | Installed ROS distribution |
| `WAREHOUSE_WSL_USER` | Current shell's username | Linux account in the runtime distribution |
| `WAREHOUSE_WSL_TARGET` | `ros-llm-ops` under the runtime user's home | Absolute runtime checkout path |
| `WAREHOUSE_WSL_BIN` | Standard Windows `wsl.exe` path | Optional override for the WSL executable |

The wrapper queries the runtime user's home directory rather than storing a personal path in source. The runtime account must already exist and the ROS/model dependencies must be installed there. Source sync creates the target directory if necessary.

For different account names or paths, set overrides in your own shell configuration. For example, replace the placeholders locally:

```bash
export WAREHOUSE_WSL_DISTRO=Ubuntu-24.04
export WAREHOUSE_WSL_USER=YOUR_RUNTIME_USER
export WAREHOUSE_WSL_TARGET=/path/to/ros-llm-ops
bash scripts/wsl.sh sync
bash scripts/wsl.sh demo
```

Available actions: `sync`, `model`, `sim`, `agent`, `demo`, `browser`, `browser-info`, `browser-stop`, and `logs`. Sync excludes Git metadata, virtual environments, agent configuration, and runtime artifacts. The log-copy action also excludes browser passwords, session metadata, and temporary browser-test dependencies.

For a single Ubuntu 24.04 checkout, use `bash scripts/demo.sh` or `bash scripts/run_browser.sh` directly; the WSL wrapper is optional.

## Display and runtime checks

A native/WSLg display is needed for the normal GUI launcher. The [browser desktop](browser-access.md) provides a virtual display for SSH access. GPU visibility alone does not prove Gazebo rendering works; inspect the actual scene and verify a navigation goal.

The simulation scripts use ROS domain 42 and localhost discovery. For diagnostic ROS commands in a separate runtime terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic echo /clock --once
ros2 action info /navigate_to_pose
ros2 param get /amcl use_sim_time
```

Use the [MVP runbook](mvp-runbook.md) for localization, separate-terminal operation, and known shutdown diagnostics. Store personal diagnostic output under ignored `artifacts/`; publish only reviewed results and application captures.
