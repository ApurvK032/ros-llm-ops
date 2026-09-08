# Project brief

> The simulation-first MVP is now implemented with local Qwen/Ollama. See [current status](status.md) and the [runbook](mvp-runbook.md); this brief preserves the earlier planning assumptions.

## Assessment

Proceed with this project. It connects the existing path/sequence planner to an observable execution loop, and it has a focused demo: instructions change while a robot carries out deliveries. The central engineering challenge is keeping accepted obligations consistent with what the robot has actually done.

The strongest potential research question is: **Does checking execution-state freshness at plan activation improve mid-mission update correctness, and what waiting, retry, and interruption costs does it introduce?** This is a hypothesis to test. A system integration alone does not establish publication novelty.

The expanded [related-work survey](related-work-survey.md) found substantial overlap, particularly a [2026 PlanSys2 plan-repair system](https://www.mdpi.com/2218-6581/15/4/80). Assess that implementation before claiming a new execution-aware planning method. The portfolio-first objective remains the same.

The original scope is disciplined. Four changes make the plan easier to execute:

- Start core implementation immediately in a dependency-free grid backend while checking the simulator environment early in a separate Ubuntu 24.04 distribution.
- Specify a request's lifecycle through receipt, proposal, queueing, and effectiveness. Queue acceptance alone cannot certify that a revision is still valid when activated.
- Narrow the first research comparison to the same mechanism with freshness checking enabled/disabled. Add architectural comparisons afterward.
- Treat the 40–65-hour MVP estimate as optimistic until the first grid slice and stock Nav2 tests pass.

## Product concept

Working name: **Warehouse Agent**. Repository name remains `ros-llm-ops`.

An operator can create a mission, inspect progress, prioritize a parcel, request delivery of onboard cargo first, and cancel an uncollected order. The terminal shows received requests, effective revisions, current actions, cargo, deferred work, and evidence-based outcomes.

Use one differential-drive robot, one stock simulation initially, three parcels, a known map, and named pickup/drop locations. Start with greedy sequencing. Add hill climbing only when the same constraints and validator remain valid.

There is no need for microservices or a general agent framework at this stage. A Python application with a single state coordinator and replaceable backend is sufficient. Event records and replay are the foundation of observability; a dashboard can wait.

## Demo storyboard

1. Request delivery of P1, P2, and P3. Display the accepted plan.
2. While navigating, prioritize P3. Display receipt and the later effective revision separately.
3. Cancel P2 near pickup completion. If cancellation wins, P2 remains at pickup and leaves the mission. If pickup wins, cargo stays onboard and the operator is asked to continue delivery or pause. Run both race outcomes in separate rehearsals.
4. Make one destination fail navigation. After the bounded recovery policy, record a deferred obligation and an honest partial outcome.
5. Show delivered, cancelled, deferred, and onboard parcels alongside event IDs supporting the report.

## Milestones and planning allowance

| Milestone | Deliverable | Working allowance |
| --- | --- | --- |
| Preparation | Behavior contracts, baseline, local workspace, environment procedure | Prepared in this task; simulator check pending |
| A1 | Deterministic grid state, planner, executor, revision tests | 20–30 focused hours |
| A2 | Fake and hosted language tool loop, grid demo | 10–18 hours |
| B | Stock environment, ROS adapter, warehouse demo and traces | 25–40 hours |
| C | Pilot, held-out evaluation, failure analysis, report | 30–50 hours after B |

These are estimates, not measured delivery times: roughly 55–88 hours to the MVP plus contingency, and 30–50 additional hours for a research release. ROS experience, WSL graphics, and experimental rigor can move them substantially. Re-estimate after A1 and the stock Nav2 check. Use milestone completion checks to track progress.

## Defaults and open decisions

Project direction: a working robotics demonstration first, then research experiments. Track progress through milestone completion checks.

Working technical decisions: Python core; Ubuntu 24.04/Jazzy/Harmonic for simulation; stock TurtleBot3 example; hosted model behind a replaceable interface; no shared-server dependency. Choose model/provider and a spending cap at AGENT-02. Choose robot tolerances and the warehouse scale through measurement at ROS-02. Decide any public release licensing when reused source and asset provenance are assembled.

We can start CORE-01 without those later decisions. Avoid adding deadlines, battery, carrying capacity, manipulation, local models, or new optimization algorithms until Milestone B passes.
