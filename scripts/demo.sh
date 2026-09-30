#!/usr/bin/env bash
# Run inside Ubuntu 24.04. The WSL wrapper syncs the source before starting this.
set -eo pipefail
cd "$(dirname "$0")/.."
mkdir -p artifacts
sim_pid=
model_pid=
start_sim() {
  # A process group ensures Gazebo's GUI subprocess exits with the demo.
  setsid env --default-signal=INT,QUIT bash scripts/run_sim.sh > artifacts/simulation.log 2>&1 &
  sim_pid=$!
}
stop_sim() {
  if [[ -n "$sim_pid" ]]; then
    # Let ros2 launch coordinate shutdown; signalling every child as well
    # causes duplicate interrupts in rclpy, RViz and Gazebo.
    kill -INT "$sim_pid" 2>/dev/null || true
    wait "$sim_pid" 2>/dev/null || true
    kill -TERM -- "-$sim_pid" 2>/dev/null || true
    sim_pid=
  fi
}
# 0: Nav2's navigation stack is active; 1: Nav2 aborted its own bringup; 2: the simulator exited; 3: timed out.
wait_for_nav2() {
  local deadline=$((SECONDS + 180))
  while (( SECONDS < deadline )); do
    if grep -q 'lifecycle_manager_navigation.*Managed nodes are active' artifacts/simulation.log; then return 0; fi
    if grep -q 'Aborting bringup' artifacts/simulation.log; then return 1; fi
    if ! kill -0 "$sim_pid" 2>/dev/null; then return 2; fi
    sleep 0.5
  done
  return 3
}
cleanup() {
  stop_sim
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
# Nav2 can abort its own bringup when a lifecycle service call times out; on WSL2 kernels before 6.18.35.2 this
# follows ~30 s wall-clock jumps (see docs/runbook.md). Nothing has happened yet, so restart the simulation.
for attempt in 1 2 3; do
  start_sim
  status=0
  wait_for_nav2 || status=$?
  if (( status == 0 )); then
    echo "Nav2 navigation stack active (simulation attempt $attempt)."
    break
  fi
  cp artifacts/simulation.log "artifacts/simulation.attempt$attempt.log"
  stop_sim
  if (( status != 1 || attempt == 3 )); then
    reason=([2]="the simulator exited" [3]="Nav2 was not ready within 180 s" [1]="Nav2 aborted its bringup")
    echo "Simulation not ready: ${reason[$status]} (attempt $attempt). Log: artifacts/simulation.attempt$attempt.log" >&2
    tail -20 "artifacts/simulation.attempt$attempt.log" >&2
    exit 1
  fi
  echo "Nav2 aborted its bringup (attempt $attempt); restarting the simulation." >&2
done
bash scripts/run_agent.sh "$@"
