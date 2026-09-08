#!/usr/bin/env bash
# Official ROS repositories; only the dedicated Ubuntu 24.04 environment.
set -euo pipefail
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  echo 'Requires Ubuntu 24.04. No packages changed.' >&2
  exit 2
fi
if [[ "${1:-}" != --apply ]]; then
  echo 'Run --apply to configure the official ROS apt source and install Jazzy, Nav2, Gazebo and development tools.'
  exit 0
fi
if (( EUID != 0 )); then exec sudo bash "$0" --apply; fi
export DEBIAN_FRONTEND=noninteractive
if [[ -e /proc/sys/fs/binfmt_misc/WSLInterop ]]; then
  mkdir -p /etc/systemd/system/systemd-binfmt.service.d
  printf '[Unit]\nConditionVirtualization=!wsl\n' > /etc/systemd/system/systemd-binfmt.service.d/warehouse-wsl.conf
  systemctl daemon-reload
fi
apt-get update
apt-get install -y ca-certificates curl locales software-properties-common python3
locale-gen en_US.UTF-8
add-apt-repository -y universe
if ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
  release=$(curl --fail --location --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
  [[ "$release" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]
  curl --fail --location --retry 3 -o /tmp/warehouse-ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${release}/ros2-apt-source_${release}.noble_all.deb"
  dpkg -i /tmp/warehouse-ros2-apt-source.deb
fi
apt-get update
apt-get install -y ros-jazzy-desktop ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-minimal-tb3-sim \
  ros-jazzy-ros-gz ros-dev-tools python3-venv python3-pip python3-pyqt5 \
  build-essential cmake git zstd mesa-utils
echo 'Installed. Source /opt/ros/jazzy/setup.bash before running the simulation.'
