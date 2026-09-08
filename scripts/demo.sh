#!/usr/bin/env bash
# Run inside Ubuntu 24.04. The WSL wrapper syncs the source before starting this.
set -eo pipefail
cd "$(dirname "$0")/.."
mkdir -p artifacts
sim_pid=
model_pid=
cleanup() {
  if [[ -n "$sim_pid" ]]; then
    # Let ros2 launch coordinate shutdown; signalling every child as well
    # causes duplicate interrupts in rclpy, RViz and Gazebo.
    kill -INT "$sim_pid" 2>/dev/null || true
    wait "$sim_pid" 2>/dev/null || true
    kill -TERM -- "-$sim_pid" 2>/dev/null || true
  fi
  if [[ -n "$model_pid" ]]; then kill "$model_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
if ! curl --max-time 2 --silent --fail http://127.0.0.1:11434/api/tags >/dev/null; then
  bash scripts/run_model.sh > artifacts/ollama.log 2>&1 &
  model_pid=$!
  for _ in {1..40}; do
    if curl --max-time 2 --silent --fail http://127.0.0.1:11434/api/tags >/dev/null; then break; fi
    sleep 0.25
  done
fi
if ! ollama show qwen3.5:4b >/dev/null 2>&1; then ollama pull qwen3.5:4b; fi
# A process group ensures Gazebo's GUI subprocess exits with the demo.
setsid env --default-signal=INT,QUIT bash scripts/run_sim.sh > artifacts/simulation.log 2>&1 &
sim_pid=$!
sleep 1
if ! kill -0 "$sim_pid" 2>/dev/null; then
  cat artifacts/simulation.log >&2
  exit 1
fi
bash scripts/run_agent.sh "$@"
