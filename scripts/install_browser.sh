#!/usr/bin/env bash
set -euo pipefail
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
  echo 'Install the browser desktop inside the ROS Ubuntu-24.04 distribution.' >&2
  exit 2
fi
packages=(tigervnc-standalone-server tigervnc-tools novnc websockify openbox tint2 xterm wmctrl xdotool xauth)
if [[ "${1:-}" != --apply ]]; then
  echo "Installs: ${packages[*]}"
  echo 'Run with --apply in Ubuntu-24.04 to install.'
  exit 0
fi
privilege=()
if [[ "$EUID" != 0 ]]; then privilege=(sudo); fi
"${privilege[@]}" apt-get update
"${privilege[@]}" apt-get install -y --no-install-recommends "${packages[@]}"
