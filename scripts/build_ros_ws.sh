#!/usr/bin/env bash
# Build the colcon workspace when its sources changed. Several launchers may call this at once.
set -eo pipefail
cd "$(dirname "$0")/../ros_ws"
source /opt/ros/jazzy/setup.bash
exec 8>.build.lock
flock 8
stamp=install/.warehouse_build_stamp
if [[ -f "$stamp" && -z "$(find src -newer "$stamp" -print -quit)" ]]; then
  exit 0
fi
echo "Building the ROS workspace (ros_ws)..."
colcon build --event-handlers console_direct- --cmake-args -DCMAKE_BUILD_TYPE=Release > build.log 2>&1 || {
  tail -30 build.log >&2
  exit 1
}
touch "$stamp"
