#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec 9>"/tmp/warehouse-sim-${UID}-${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
  echo 'A warehouse simulation is already running on this ROS domain.' >&2
  exit 2
fi
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
export QT_QPA_PLATFORM=xcb
if [[ -e /dev/dxg ]]; then
  export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"
  export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-d3d12}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
fi
python3 -B -m warehouse_agent.sim_config
if [[ ! -x build/marker_bridge/warehouse_marker_bridge || sim/marker_bridge/main.cpp -nt build/marker_bridge/warehouse_marker_bridge || sim/marker_bridge/CMakeLists.txt -nt build/marker_bridge/warehouse_marker_bridge ]]; then
  bash scripts/build_visuals.sh
fi
exec ros2 launch "$PWD/sim/warehouse.launch.py" \
  world:="$PWD/sim/generated/warehouse.sdf" map:="$PWD/sim/generated/warehouse.yaml" \
  params_file:="$PWD/sim/generated/nav2_params.yaml" \
  headless:="${HEADLESS:-False}" use_rviz:="${USE_RVIZ:-True}" \
  show_panel:="${SHOW_MISSION_PANEL:-True}" "$@"
