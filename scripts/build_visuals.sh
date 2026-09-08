#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
# ROS's Gazebo vendor packages keep their CMake configs under individual prefixes.
visual_prefix=/opt/ros/jazzy
for path in /opt/ros/jazzy/opt/*_vendor; do
  visual_prefix="$visual_prefix;$path"
done
cmake -S sim/marker_bridge -B build/marker_bridge \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$visual_prefix"
cmake --build build/marker_bridge --parallel 2
