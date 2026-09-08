# Implementation backlog

## Current incremental demo stages — 8 September 2026

- [x] Stage 1: labeled stations, cargo visuals, current-goal highlight and read-only mission panel. [Implementation and evidence](stage1-visuals.md).
- [ ] Stage 2: operator interface for submitting instructions, pausing/resuming and viewing command feedback.
- [ ] Stage 3: startup/shutdown cleanup and dependable everyday launch behavior.
- [ ] Stage 4: repeatable delivery/update/failure scenarios and measured evaluation.

The stages below preserve the earlier research-oriented plan; they do not replace this agreed incremental order.

> Update, 7 September 2026: the original simulation MVP is now implemented with local Qwen/Ollama. See [current status](status.md) and [MVP runbook](mvp-runbook.md). This document preserves the earlier, broader design/setup plan; unimplemented contracts remain future work.

Confirmed priority: working robotics portfolio demo, then research. There is no deadline. Use milestone finish checks; estimates describe focused engineering effort and should be revised after the first two gates.

## Preparation completed

- [x] Preserve the supplied outline and write an actionable scope.
- [x] Inspect the machine and verify the documented ROS/Gazebo pairing.
- [x] Initialize local Git and a project-local Python environment.
- [x] Pin and inspect WarehouseBot; run its original smoke tests and a static baseline.
- [x] Define five missions, five updates, and revision/cancellation policy.
- [x] Provide local bootstrap, diagnostics, reference reproduction, and a guarded ROS package installer.
- [x] Stock Nav2 simulation readiness in the separate Ubuntu 24.04 environment.

## Next two work sessions

**Research follow-up before selecting the integration framework:** inspect the PlanSys2 repair implementation and DFKI tool/event contracts in [the related-work survey](related-work-survey.md). Record what they enforce at revision activation and whether PlanSys2 reuse reduces our implementation effort. This investigation can run alongside CORE-01; no new framework has been selected or installed.

**Session 1 — core behavior, approximately 2–3 hours.** Read the behavior contract. Implement CORE-01's parcel transitions and event records first. Write tests for acknowledged pickup/drop, duplicate acknowledgements, cancellation just before/after pickup, and episode isolation. Finish with event replay of one scripted pickup/drop. No LLM required.

**Session 2 — simulator feasibility, approximately 2–3 hours plus downloads.** Add Ubuntu 24.04 using the environment guide. Install Jazzy/Nav2 and launch the stock example. Demonstrate arrival and cancellation with recorded terminal outcomes, then run with both GUIs disabled. If setup exceeds the session, record the specific failure and continue CORE-02 independently.

## Milestone A1 — deterministic grid system

| ID | Work | Depends on | Finish check |
| --- | --- | --- | --- |
| CORE-01 | Parcel lifecycle, request/event IDs, coordinator, JSONL journal and replay | Behavior contract | No state change without an acknowledgement; replay matches state; duplicates and old-episode events cannot transfer cargo twice |
| CORE-02 | Step-based grid backend and executor with separate navigation/handling actions | CORE-01 | Three parcels delivered; pause works; one active action; injected failure never produces a delivery |
| PLAN-01 | Build remaining obligations, adapt A* costs and greedy eligibility | CORE-01 | Onboard cargo has a drop only; cancellations/completions disappear appropriately; missing stops, cycles, and infinite costs are rejected |
| UPDATE-01 | Request lifecycle, dispatch fence, atomic boundary activation | CORE-02, PLAN-01 | Both pickup/cancellation race outcomes pass; stale and superseded proposals cannot activate |
| EVAL-01 | Independent event/obligation checks for development fixtures | UPDATE-01 | Every request has an outcome; final reports agree with events; omitted jobs are caught |

Tests should attack invariants rather than mirror implementation. Include a pickup acknowledged twice with different event IDs, a completion after cancel was requested, an unknown parcel, a dropped onboard obligation, an infinite route edge, an old episode result, map changes between proposal and activation, and two updates completing in the opposite order from receipt. Keep executor failure injection out of the model's observable state until it occurs.

**A1 gate:** a single documented command runs scripted normal delivery, priority, cancellation, onboard clarification, stale update, and navigation-failure episodes and saves replayable evidence. This command is not implemented yet.

## Milestone A2 — language interface

| ID | Work | Depends on | Finish check |
| --- | --- | --- | --- |
| AGENT-01 | Typed tools, bounded orchestration, fake client | A1 | Malformed/unknown tool calls cannot mutate state; requests and clarification replies are correlated |
| AGENT-02 | One hosted model adapter, recorded settings and usage cap | AGENT-01, provider/budget decision | Natural-language missions and changes pass the grid demo; timeout/pause policy remains responsive |
| DEMO-01 | Terminal workflow and event-backed status report | AGENT-02 | Run the storyboard without source edits and explain every final parcel state |

Do not send robot feedback to the model every tick. Initially cap a request at eight tool calls, two replanning retries, and a 30-second wall-time model timeout, then measure and freeze settings. These are proposed defaults and may need tuning. Model outage during a pending update leaves the executor held at its action boundary; outage with no pending change does not revoke already-authorized work.

## Milestone B — ROS/Gazebo MVP

| ID | Work | Depends on | Finish check |
| --- | --- | --- | --- |
| ENV-01 | Ubuntu 24.04/Jazzy stock TB3 navigation and cancellation | Environment guide | GUI and headless checks recorded, simulation clock and transforms valid |
| ROS-01 | Nav2 backend adapter, responsive callbacks and cancellation | ENV-01, A1 | At most one goal; replacement waits for terminal state; success and failure correctly acknowledged |
| ROS-02 | Small warehouse map, named waypoints, grid/world conversion | ROS-01 | Map geometry agrees; localization and positional tolerance pass at each logical handling location |
| ROS-03 | Full demo, failures and traces | A2, ROS-02 | Normal, updated, and deferred missions run without code edits; terminal report matches recorded cargo |

Freeze features at B. Delay local inference, hill-climbing comparisons, extra robot models and visual polish until this gate.

## Milestone C — research release

| ID | Work | Depends on | Finish check |
| --- | --- | --- | --- |
| STUDY-01 | Read related work in full; freeze pilot and evaluation rules | B | Specific contribution stated without first-of-kind claims; all outcome denominators specified |
| STUDY-02 | Paired freshness ablation, controlled delays, shared guards | STUDY-01 | Same episodes/settings across variants; rejected and failed episodes retained |
| STUDY-03 | Held-out repetitions and representative Gazebo subset | STUDY-02 | Counts and uncertainty reported by category; negative findings retained |
| RELEASE-01 | Clean setup, provenance, demo video, technical report | STUDY-03 | Reproduction command and selected artifacts work from a clean environment |

Future issue titles can use these IDs directly. No remote repository, issues, or external project board has been created.
