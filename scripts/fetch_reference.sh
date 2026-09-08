#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
task_reference="$task_root/references/warehousebot"
task_commit=f54b393847f7c4929846c0824dc5b4986bd8194b
if [[ -e "$task_reference" ]]; then
  task_head="$(git -C "$task_reference" rev-parse HEAD)"
  [[ "$task_head" == "$task_commit" ]] || { echo 'Reference already exists at a different commit; left untouched.' >&2; exit 1; }
  [[ -z "$(git -C "$task_reference" status --porcelain --untracked-files=all)" ]] || { echo 'Reference has local changes; left untouched.' >&2; exit 1; }
else
  mkdir -p "$task_root/references"
  git clone https://github.com/ApurvK032/WarehouseBot-Pick-and-Drop-Optimization.git "$task_reference"
  git -C "$task_reference" checkout --detach "$task_commit"
fi
echo "Reference available at $task_commit"
