#!/usr/bin/env bash
# Run in Ubuntu-24.04, normally through scripts/wsl.sh browser.
set -eo pipefail
cd "$(dirname "$0")/.."
exec python3 -B -m warehouse_agent.remote_desktop "$@"
