#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_root"
python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 12) else "Python 3.12+ required")'
if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv --without-pip .venv
fi
"$task_root/.venv/bin/python" -m warehouse_agent doctor
echo 'Core environment ready. No third-party dependencies or pip are needed yet.'
