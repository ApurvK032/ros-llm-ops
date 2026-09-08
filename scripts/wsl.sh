#!/usr/bin/env bash
# Sync this checkout into a configured Ubuntu ROS distribution, then run there.
set -euo pipefail
cd "$(dirname "$0")/.."
wsl_bin="${WAREHOUSE_WSL_BIN:-/mnt/c/Windows/System32/wsl.exe}"
wsl_cmd=("$wsl_bin")
# WSL can lose binfmt registration when another distribution stops.
if [[ -x /init && ! -e /proc/sys/fs/binfmt_misc/WSLInterop && ! -e /proc/sys/fs/binfmt_misc/WSLInterop-late ]]; then
  wsl_cmd=(/init "$wsl_bin" "$wsl_bin")
fi
distro="${WAREHOUSE_WSL_DISTRO:-Ubuntu-24.04}"
runtime_user="${WAREHOUSE_WSL_USER:-$(id -un)}"
target="${WAREHOUSE_WSL_TARGET:-}"
if [[ -z "$target" ]]; then
  target=$("${wsl_cmd[@]}" -d "$distro" -u "$runtime_user" --cd / --exec sh -c 'printf "%s/ros-llm-ops\n" "$HOME"')
  target=${target%$'\r'}
fi
if [[ "$target" != /* || "$target" == *$'\n'* || "$target" == *$'\r'* ]]; then
  echo 'WAREHOUSE_WSL_TARGET must be an absolute Linux directory path.' >&2
  exit 2
fi
case "${1:-}" in
  sync)
    "${wsl_cmd[@]}" -d "$distro" -u "$runtime_user" --cd / --exec mkdir -p "$target"
    tar --exclude='./.git' --exclude='./.venv' --exclude='./artifacts' \
      --exclude='./.agents' --exclude='./.codex' --exclude='__pycache__' -cf - . |
      "${wsl_cmd[@]}" -d "$distro" -u "$runtime_user" --cd "$target" --exec tar -xf -
    ;;
  logs)
    mkdir -p artifacts/ros
    "${wsl_cmd[@]}" -d "$distro" -u "$runtime_user" --cd "$target" --exec tar \
      --exclude=artifacts/browser-check --exclude=artifacts/browser/password.txt \
      --exclude=artifacts/browser/session.json -cf - artifacts |
      tar -xf - -C artifacts/ros
    ;;
  browser-info|browser-stop)
    action=${1#browser-}
    exec "${wsl_cmd[@]}" -d "$distro" -u "$runtime_user" --cd "$target" --exec bash scripts/run_browser.sh "$action"
    ;;
  sim|agent|model|demo|browser)
    action=$1
    shift
    bash scripts/wsl.sh sync
    script="scripts/run_${action}.sh"
    if [[ "$action" == demo ]]; then script=scripts/demo.sh; fi
    exec "${wsl_cmd[@]}" -d "$distro" -u "$runtime_user" --cd "$target" --exec bash "$script" "$@"
    ;;
  *)
    echo 'Usage: bash scripts/wsl.sh {sync|sim|agent|model|demo|browser|browser-info|browser-stop|logs}' >&2
    exit 2
    ;;
esac
