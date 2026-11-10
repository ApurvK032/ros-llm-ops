# Fault-injection scenarios

A scenario runs one headless delivery mission, injects one fault at a set point in the mission, records the run,
and scores how it ended. It is the test bench for diagnosing and recovering from faults: every run has a known
root cause and a measured outcome.

```bash
python3 -B -m warehouse_agent scenario scenarios/blocked_station.json
python3 -B -m warehouse_agent scenario scenarios/blocked_station.json --no-fault   # same mission, no fault
```

Each run starts its own simulation, runs the supervisor with the scenario's command, and writes a directory
under `artifacts/scenarios/<name>/<time>/`:

| File | Contents |
| --- | --- |
| `result.json` | The scenario, what the fault actually changed and when, the outcome, and the checker's verdict |
| `journal.jsonl` | The supervisor's journal, as for any mission |
| `bag/` | MCAP recording: TF, raw and relayed lidar, odometry, plans, costmaps, Nav2 health, `/rosout`, mission status |
| `simulation.log`, `agent.log` | Console output of the simulator and the supervisor |

Requirements are the same as the demo (ROS 2 Jazzy, Gazebo Harmonic, Ollama with `qwen3.5:4b`); no display is
needed. The scenario file format is validated on load; see [`scenarios/`](../scenarios/) for examples.

## How a fault is triggered

A trigger names a journal event, so a fault lands at the same point in the mission on every run rather than at a
wall-clock time:

```json
"trigger": {"event": "cargo_pickup", "count": 1, "delay_s": 5.0}
```

means five seconds after the first pickup. `where` can narrow the match on dotted journal keys, for example
`{"stop.parcel": "P2"}`. Injectors take two to four seconds to act; `result.json` records when the fault was
actually in place (`fault.monotonic`) and how late that was (`fault.seconds_late`).

## Faults

| Type | What it does | How |
| --- | --- | --- |
| `blocked_path` | Drops a static box at a waypoint or position | Gazebo `create` service; the box has a visual, so the rendered lidar sees it |
| `node_crash` | Kills one Nav2 server with SIGKILL | The simulation runs Nav2 as separate processes (`use_composition:=False`) |
| `sensor_dropout` | Stops (`drop`) or freezes (`freeze`) the lidar scans Nav2 receives | A relay node between `/scan` and `/scan_nav` |
| `localization_jump` | Shifts AMCL's pose estimate by an offset from the true pose | A confident `/initialpose` at the shifted pose |
| `bad_parameter` | Sets one parameter on a running node to a harmful value | Parameter service; the previous value is recorded |

Scenario runs differ from the plain demo in two ways, both applied to every scenario run including the control:
Nav2 runs as separate processes, and Nav2 reads the lidar through the relay (`SCAN_RELAY=1`). The plain demo
(`scripts/demo.sh`) is unchanged.

## Baseline: no diagnosis or recovery

One run of each scenario on 9 November 2026 (RTX 4070, Ubuntu 24.04), with the supervisor as it is today and
nothing responding to the fault. The command is always
`Deliver all three parcels`; every fault lands about 20 s into the mission, after P1 is onboard and while the
robot drives to collect P2.

| Scenario | Fault | Result | Delivered | Mission time | What happened |
| --- | --- | --- | --- | --- | --- |
| [`nominal`](evidence/scenarios/nominal/result.json) | none | complete | 3 of 3 | 123.3 s | Control run |
| [`blocked_station`](evidence/scenarios/blocked_station/result.json) | Box on the PICK_B approach pose | partial | 2 of 3 | 493.5 s | Two attempts on P2 cost 431 s (one hit the 240 s navigation timeout) before P2 was deferred |
| [`controller_crash`](evidence/scenarios/controller_crash/result.json) | `controller_server` killed | partial | 0 of 3 | 19.9 s | Every goal failed at once; all three parcels were deferred within a second of the crash, P1 still onboard |
| [`lidar_dropout`](evidence/scenarios/lidar_dropout/result.json) | No scans reach Nav2 | partial | 0 of 3 | 22.6 s | Goals failed immediately; all three parcels deferred within 1.5 s of the dropout, P1 still onboard |
| [`localization_lost`](evidence/scenarios/localization_lost/result.json) | Pose estimate shifted 1.5 m and 0.6 rad | no route | 0 of 3 | 203.1 s | The goal failed after 180 s; the supervisor then found no route from the pose it believed and held, P1 still onboard |
| [`bad_parameter`](evidence/scenarios/bad_parameter/result.json) | `velocity_smoother.max_velocity` set to zero | timeout | 0 of 3 | 717.3 s | Each goal took about 180 s to fail; the run hit its 720 s limit with P2 deferred and P1 still onboard |

All six journals pass `warehouse_agent verify` with 0 violations: the supervisor never broke one of its own
rules, but only the control delivered everything.

What the baseline shows:

- **Symptoms overlap.** Five different root causes all surface to the supervisor as "navigation failed". The
  journal alone cannot tell a blocked station from a dead lidar or a zeroed speed limit; the recording can.
- **Retries have no delay.** After the controller crash and the lidar dropout the supervisor used every retry
  and deferred the whole mission in under two seconds. A fault that might clear in ten seconds ends the mission.
- **Slow faults are silent for minutes.** With lost localization or a zero speed limit, the first sign in the
  journal comes about 180 s after the fault.

These are single runs. The control took 123.1 s through the harness without the relay, 123.3 s with it, and
123.2 s through the demo launcher; the fault scenarios have not yet been repeated, so the times above are
examples, not a distribution.

## Limitations

- One fault per run, one robot, one warehouse layout.
- Faults are injected in simulation; the lidar relay stands in for a sensor or driver fault.
- A localization fault can let the supervisor accept a pickup or drop at the wrong physical place, because
  arrival checks use the localization estimate. The harness does not yet compare transfers with Gazebo's
  ground-truth pose.
- Recordings are not committed (5 to 160 MB per run); results and journals are, under
  [`docs/evidence/scenarios/`](evidence/scenarios/).
