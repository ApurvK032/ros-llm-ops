# Mission examples

These walkthroughs use the three configured parcels: P1 from PICK_A to DROP_A, P2 from PICK_B to DROP_B, and P3 from PICK_C to DROP_C. Commands in `text` blocks go into the **agent terminal**, not your shell. Submit one instruction at a time and wait for its interpretation.

Start each independent walkthrough with a fresh demo. Exit the previous combined launcher with `/quit`, wait for cleanup, and restart it to return the robot to HOME and reset logical parcel state.

Use `bash scripts/demo.sh` in the Ubuntu 24.04 checkout, or `bash scripts/wsl.sh demo` on the prepared two-distribution WSL machine. For remote viewing, launch the [browser desktop](../README.md#watch-and-control-it-in-a-browser) instead. Wait for `Nav2 and localization ready` before entering a mission.

## 1. Deliver everything

```text
Deliver all three parcels
```

The interpreted operation should be `create` for P1, P2, and P3. The planner selects a feasible sequence from the robot's current position. Each parcel must be picked up before its own drop; the order between different parcels can depend on the current pose and policy.

Watch for:

1. A cyan indicator at the current navigation goal.
2. An amber parcel becoming blue and following the robot after confirmed pickup.
3. A blue parcel becoming green at its drop station after confirmed delivery.
4. All three parcels marked delivered and zero remaining stops at completion.

A representative **single-parcel excerpt** from the CLI looks like this. Other parcels' events can occur between these lines:

```text
→ PICK_A · pickup P1
✓ P1 onboard at PICK_A
→ DROP_A · drop P1
✓ P1 delivered at DROP_A
```

For an unattended shell command that exits after completion:

```bash
bash scripts/demo.sh --command 'Deliver all three parcels'
```

Recorded full-run evidence: [delivery journal](../docs/evidence/stage1/delivery.jsonl) and [completed screenshot](../docs/evidence/stage1/gazebo-delivered.png).

## 2. Deliver a subset

Start a fresh demo and enter:

```text
Deliver P1 and P2
```

Only P1 and P2 become active orders. P3 stays at its pickup station without a scheduled pickup or drop. After completion, enter `/status` to inspect the application state and confirm which parcels were delivered.

## 3. Change priority while the robot is working

Start all three deliveries:

```text
Deliver all three parcels
```

While P3 is still active and unfinished, enter:

```text
/pause
```

Wait for the robot to stop, then enter:

```text
Make P3 the highest priority
```

Wait until the terminal prints the interpreted `prioritize` request, then continue:

```text
/resume
```

After resuming, inspect the refreshed remaining stops in the mission panel or with `/status`. Remaining work should favor P3 while preserving cargo obligations. If P3 is still waiting, its pickup must precede its drop. This walkthrough assumes you have not enabled onboard-first, which takes precedence when cargo is already carried.

## 4. Finish onboard deliveries before new pickups

Start all three deliveries. After the panel first shows a blue onboard parcel, enter `/pause` and wait for navigation to stop. Then enter:

```text
Deliver the parcels already onboard first
```

Wait for the interpreted `onboard_first` operation and enter `/resume`, then inspect the refreshed remaining stop list. The robot should finish currently carried deliveries ahead of collecting more parcels. This policy is most visible when at least one other active parcel is still awaiting pickup.

## 5. Cancel an uncollected order

Start all three deliveries, enter `/pause` while P2 is still waiting, and wait for navigation to stop. Enter:

```text
Cancel order P2
```

After the interpreted cancellation, P2 should remain uncollected and appear gray. Enter `/resume` to finish the remaining active orders. If you cancel P2 while a goal is active, the supervisor waits for its terminal cancellation acknowledgement before dispatching replacement work.

If P2 has already been picked up, the behavior is different: cancellation pauses for clarification and preserves the onboard cargo. `/resume` continues P2's original delivery. The MVP has no return station or return-to-sender operation.

## 6. Inspect state or try an unknown request

```text
What are you carrying?
```

This uses the model to identify a status request, then prints the application's confirmed mission state. `/status` returns state directly without a model call.

```text
Deliver P9
```

P9 is unknown. The model is instructed to ask for clarification; the supervisor also rejects unknown IDs if they reach validation. No order for P9 should execute. A rejected request can pause an existing mission, which you can inspect with `/status` before resuming.

Arbitrary new destinations, compound instructions, and return-to-sender requests are outside the current intent interface.

## Inspect the run afterward

Each agent launch creates a JSONL journal under `artifacts/episodes/` in its runtime checkout. It records operator requests, model interpretation, plan changes, navigation results, and cargo events. On the prepared WSL setup, `bash scripts/wsl.sh logs` copies runtime journals back into the source workspace.

The complete Gazebo/browser delivery and the live-update checks have [recorded evidence](../docs/evidence/). The walkthroughs above describe supported interactions; they are not an automated timing-controlled scenario suite.
