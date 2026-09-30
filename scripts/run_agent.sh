#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
bash scripts/build_ros_ws.sh
source ros_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec 9>"/tmp/warehouse-agent-${UID}-${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
  echo 'A warehouse agent is already running on this ROS domain.' >&2
  exit 2
fi
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec python3 -B -m warehouse_agent run "$@"
