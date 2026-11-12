# Fault-injection scenarios

A scenario runs one headless delivery mission, injects one or more faults at set points in the mission, records
the run, and scores how it ended. It is the test bench for diagnosing and recovering from faults: every run has
a known root cause and a measured outcome.

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
| `bag/` | MCAP recording: TF, raw and relayed lidar, odometry, plans, costmaps, Nav2 health, `/rosout`, mission status. It runs on for 10 s after the mission ends, to catch the system's reaction |
| `truth.jsonl` | The robot's true pose from the simulator, ten times a second, used to check every pickup and delivery |
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

A scenario can carry further faults in `additional`, each with its own trigger. They are injected the same way
and recorded in `result.json` under `additional_faults`:

```json
"trigger": {"event": "cargo_pickup", "count": 2, "delay_s": 5.0},
"fault": {"type": "sensor_dropout", "sensor": "lidar", "mode": "drop"},
"additional": [
  {"trigger": {"event": "cargo_pickup", "count": 1, "delay_s": 3.0},
   "fault": {"type": "bad_parameter", "node": "/velocity_smoother", "name": "max_velocity", "value": [0.3, 0.0, 2.0]}}
]
```

## What a correct response looks like

Each scenario states what a correct response achieves, in `expected`. It is used only to judge runs afterwards;
nothing that makes decisions during a mission reads it.

```json
"expected": {"delivered": 3, "actions": [["restart_process", "amcl"]], "acceptable": [["relocalize", ""]]}
```

`delivered` is the number of parcels that can be delivered at all (two when a station is blocked). `actions` are
the recovery actions the fault needs, and `acceptable` those that may reasonably accompany them. Any other action
that is carried out counts as unneeded, apart from clearing the costmaps and asking for a person, which change
nothing about the robot's configuration or the mission plan.

## Checking deliveries against the true pose

The supervisor accepts a pickup or a delivery when the robot's estimated pose passes the arrival check. After a
localization or lidar fault that estimate is exactly what cannot be trusted. Every scenario run therefore records
the robot's true pose from Gazebo (`truth.jsonl`), and `result.json` repeats the same arrival check on the true
pose at the moment of each transfer (`pose_check`). A parcel counts as delivered only if the robot really was at
its pickup station and at its drop station; `outcome.delivered_verified` lists those parcels.

That this matters is not hypothetical: with the lidar stuck on one scan and nothing responding, the supervisor
reported deliveries in two of three runs while the robot was 9 to 20 m from the stations
([evaluation](evaluation.md)).

## Faults

| Type | What it does | How |
| --- | --- | --- |
| `blocked_path` | Drops a static box at a waypoint or position | Gazebo `create` service; the box has a visual, so the rendered lidar sees it |
| `node_crash` | Kills one Nav2 server with SIGKILL | The simulation runs Nav2 as separate processes (`use_composition:=False`) |
| `sensor_dropout` | Stops (`drop`) or freezes (`freeze`) the lidar scans Nav2 receives | A relay node between `/scan` and `/scan_nav` |
| `localization_jump` | Shifts AMCL's pose estimate by an offset from the true pose | One confident `/initialpose` at the shifted pose; the injector then checks the estimate moved and reports an error if it did not |
| `bad_parameter` | Sets one parameter on a running node to a harmful value | Parameter service; the previous value is recorded |

Scenario runs differ from the plain demo in two ways, both applied to every scenario run including the control:
Nav2 runs as separate processes, and Nav2 reads the lidar through the relay (`SCAN_RELAY=1`). The plain demo
(`scripts/demo.sh`) is unchanged.

## Baseline: no diagnosis or recovery

Each scenario recorded twice on 9 November 2026 (RTX 4070, Ubuntu 24.04), with the supervisor as it is today and
nothing responding to the fault. The command is always `Deliver all three parcels`; every fault lands about 20 s
into the mission, after P1 is onboard and while the robot drives to collect P2. The table shows the later
recording of each; the earlier one ended the same way in every scenario.

| Scenario | Fault | Result | Delivered | Mission time | What happened |
| --- | --- | --- | --- | --- | --- |
| [`nominal`](evidence/scenarios/nominal/20261110T035152Z/result.json) | none | complete | 3 of 3 | 123.3 s | Control run |
| [`blocked_station`](evidence/scenarios/blocked_station/20261110T035925Z/result.json) | Box on the PICK_B stopping pose | partial | 2 of 3 | 462.4 s | Two attempts on P2 cost 397 s (the first hit the 240 s navigation timeout) before P2 was deferred |
| [`controller_crash`](evidence/scenarios/controller_crash/20261110T035421Z/result.json) | `controller_server` killed | partial | 0 of 3 | 20.1 s | Every goal failed at once; all three parcels were deferred within a second of the crash, P1 still onboard |
| [`lidar_dropout`](evidence/scenarios/lidar_dropout/20261110T035517Z/result.json) | No scans reach Nav2 | partial | 0 of 3 | 22.2 s | Goals failed immediately; all three parcels deferred within 1.5 s of the dropout, P1 still onboard |
| [`localization_lost`](evidence/scenarios/localization_lost/20261110T035612Z/result.json) | Pose estimate shifted 1.5 m and 0.6 rad | no route | 0 of 3 | 170.8 s | The goal failed after about 150 s; the supervisor then found no route from the pose it believed and held, P1 still onboard |
| [`bad_parameter`](evidence/scenarios/bad_parameter/20261110T040727Z/result.json) | `velocity_smoother.max_velocity` set to zero | timeout | 0 of 3 | 719.7 s | Each goal took about 180 s to fail; the run hit its 720 s limit with P2 deferred and P1 still onboard |

All twelve journals pass `warehouse_agent verify` with 0 violations: the supervisor never broke one of its own
rules, but only the control delivered everything.

What the baseline shows:

- **Symptoms overlap.** Five different root causes all surface to the supervisor as "navigation failed". The
  journal alone cannot tell a blocked station from a dead lidar or a zeroed speed limit; the recording can
  (see [diagnosis](diagnosis.md)).
- **Retries have no delay.** After the controller crash and the lidar dropout the supervisor used every retry
  and deferred the whole mission in under two seconds. A fault that might clear in ten seconds ends the mission.
- **Slow faults are silent for minutes.** With lost localization or a zero speed limit, the first sign in the
  journal comes 150 to 180 s after the fault.

Repeatability, from two recordings per scenario: the result, and which parcels were delivered, deferred or left
onboard, were identical both times in all six scenarios. Mission times matched within a second except
`localization_lost` (203.1 s and 170.8 s). Two recordings are a check, not a distribution.

What an agent that responds to these faults achieves is in [recovery](recovery.md).

## The thirteen scenarios

The six above are what the diagnosis and recovery tools were developed on. Seven more were written afterwards to
find out where they stop working, and were not used to change the agent before being measured
([evaluation](evaluation.md)). Every fault lands about 20 s into the mission unless stated.

| Scenario | What happens | What a correct response is |
| --- | --- | --- |
| **Developed on** | | |
| `bad_parameter` | The speed limit is set to zero | Restore the parameter |
| `controller_crash` | The controller server is killed | Restart it |
| `lidar_dropout` | No scans reach Nav2 | Restart the lidar driver |
| `localization_lost` | The pose estimate is shifted 1.5 m and 0.6 rad | Relocalize |
| `blocked_station` | A box stands on the PICK_B stopping pose | Give up on P2, deliver the other two, tell a person |
| **Written afterwards** | | |
| `planner_crash` | The planner server is killed | Restart it |
| `amcl_crash` | The localization node is killed; restarted, it starts from its configured initial pose | Restart it, then restore the pose |
| `lidar_frozen` | The lidar keeps sending its last scan with fresh timestamps | Restart the lidar driver |
| `double_fault` | The speed limit is set to zero and the lidar stops, one to three seconds apart | Restore the parameter and restart the lidar driver |
| `distractor_lidar` | The speed limit is lowered to 0.3 m/s, which is harmless; fifty seconds later the lidar stops | Restart the lidar driver, leave the speed limit alone |
| **Nothing to fix** | | |
| `nominal` | Nothing | Nothing |
| `speed_limit` | The speed limit is lowered from 0.5 to 0.3 m/s on purpose | Nothing: the mission is slower and completes |
| `blocked_aisle` | A crate closes the passage south of the centre shelf; another aisle leads to the same place | Nothing: Nav2 takes the other route |

With nothing responding, three runs each on 10 and 11 November 2026 (the same machine), deliveries counted only
when the true pose confirms them:

| Scenario | Delivered | Mission time | What happened |
| --- | --- | --- | --- |
| `planner_crash` | 0 of 3, three times | 20 s | Every goal failed at once; all parcels deferred within a second, P1 onboard |
| `amcl_crash` | 0 of 3, three times | 21 s | The same, about a second after the crash |
| `lidar_frozen` | 0 of 3, three times | 32 to 229 s | Once the supervisor held with no route. Twice it drove on and reported 2 and 3 parcels delivered; the true pose confirms none of them |
| `double_fault` | 0 of 3, three times | 25 s | All parcels deferred within two seconds of the lidar stopping |
| `distractor_lidar` | 0 of 3, three times | 73 to 76 s | Unaffected by the speed limit; all parcels deferred a second after the lidar stopped, P1 and P2 onboard |
| `speed_limit` | 3 of 3, three times | 159 to 160 s | Completed, 36 s slower than at full speed |
| `blocked_aisle` | 3 of 3, three times | 145 to 147 s | Completed by the other aisle, about 23 s longer |

## Limitations

- One robot, one warehouse layout, at most two faults in a run.
- Faults are injected in simulation; the lidar relay stands in for a sensor or driver fault.
- The true-pose check compares the pose recorded nearest in time to each journal event (within a second, ten
  poses a second); it is not a synchronized trace of the physics.
- The scenarios were written by the same person as the agent they test. The seven later ones were fixed before
  the agent was run on them, but they are not an independent test set.
- Recordings are not committed (5 to 160 MB per run). Each run's result, journal and extracted evidence are,
  under [`docs/evidence/`](evidence/).
