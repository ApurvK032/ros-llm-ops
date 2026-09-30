# Research protocol — proposal, not a frozen benchmark

> **Archived planning document.** Kept for history; it describes plans from before the system was built, not the current behavior. See the [design](../design.md) and [results](../results.md) for the current system.

## Question and primary comparison

Does an execution-state freshness check at plan activation improve valid mid-mission update handling? Measure both correctness and the cost of holding, rejection, replanning, and clarification.

Primary ablation: the same update policy and planner with the semantic revision check enabled versus disabled. Keep action-boundary synchronization, request supersession, basic IDs, waypoint/cargo preconditions, and physical navigation protections enabled in both. Record exactly which task-consistency rules remain shared: if these already prevent all stale errors, freshness may only alter rejection timing or efficiency. That is a valid negative result.

This check is distinct from full semantic validation. Do not quietly weaken physical/executor guards to manufacture a benefit. Run intentionally weakened task-validation experiments only in the grid backend and identify them separately.

Secondary comparison, after the primary ablation is understood:

| Mode | Behavior |
| --- | --- |
| Scripted recovery after common intent parsing | Fixed supported handlers update the remaining work |
| Feedback-driven full replanning | Rebuild the entire remaining stop sequence on meaningful events |
| State-checked incremental revision | Preserve compatible work and replace the affected remaining sequence |

Use one codebase, shared scenario information, optimizer, accepted policies, model/settings, and low-level guards. Incremental revision and full replanning differ in more than freshness, so they cannot alone isolate the freshness mechanism.

## Pilot and held-out data

The ten current JSON examples are development contracts and may be used to write tests. They are not held-out evidence. After Milestone B, construct a separate pilot of 30 scenarios: five each for normal delivery, priority, pre-pickup cancellation, onboard cancellation, navigation failure, and delayed/stale update.

Define held-out layout and wording splits before tuning. A pilot is for identifying failures and estimating variability; decide the larger study size from that evidence. As an initial variability check, repeat held-out model runs at least three times, while acknowledging that this is not a statistical power guarantee. Preserve every attempt and its configuration.

## Timing and fairness

For pickup/cancellation races, inject requests relative to a specified event boundary (before dispatch, navigation completion, or pickup acknowledgement) and log the actual physical state at receipt. Equal elapsed time across modes can correspond to different cargo states. Test fixed-time disturbances separately as a deployment experiment.

Pair scenario IDs, seeds, and perturbations across modes. Record injected model response delay separately from natural provider/network delay. During a pending update, allow only the already-authorized current action to finish; use the same dispatch fence in both ablation arms. Do not feed future injected failures, ground-truth expected outcomes, or evaluator-only map changes to the agent.

Control grid step rate and Gazebo real-time factor when studying latency. Report simulation time and monotonic wall duration independently. Paused simulation must not create an infinite model timeout. Grid and Gazebo are separate measurement settings.

## Independent outcomes

Score from scenario obligations plus executor events, with evaluator code independent of the runtime validator where practical. Never score the model's own narrative as ground truth.

| Measure | Denominator or definition |
| --- | --- |
| Valid-request fulfillment | All valid received requests; include rejected, superseded, timed-out, and never-effective requests with reason codes |
| Appropriate resolution | Whether the scenario warranted fulfillment, clarification, explicit deferral, or rejection; judge supersession against the actual request sequence |
| Obligation preservation | Expected cargo and outstanding jobs versus event-derived state |
| Invalid revision execution | Task-invalid revisions that became effective and dispatched work |
| End-state correctness | Expected final disposition of every parcel and request, including unresolved cargo |
| Latency | Receipt → queued, receipt → effective, and receipt → terminal outcome, separately |
| Efficiency | Distance, wait time, interrupted actions, retries, tool calls, tokens and cost |

Accepted-instruction correctness alone rewards refusing every difficult update. Report it alongside fulfillment and unnecessary clarification/rejection. Requests awaiting an answer at the episode limit are unresolved outcomes, not successes. Failed episodes remain in counts; compare route distance on paired successes and display completion rates alongside it.

Report per-category counts, paired differences, and uncertainty. Cluster repeated runs by scenario when estimating uncertainty; do not present correlated repetitions as independent layouts. Optimizer outputs are heuristic routes unless independently proven optimal.

Predeclare a representative Gazebo subset, including failure and cancellation races. Save a run manifest with commit, configuration, prompt hash, model identifier, generation settings, provider errors, seed, dependency versions, timings and selected trace references. Record hardware and provider changes; do not silently fall back to another model in a benchmark.

## Related work to read fully

The initial three-paper scan has been expanded in [the related-work survey](related-work-survey.md). It found closer overlap than these foundational references, so this is not a completed novelty assessment.

Prioritize [LLM-Assisted Plan Execution](https://www.mdpi.com/2218-6581/15/4/80), [Agentic AI for Robot Control](https://arxiv.org/abs/2602.13081), [CoMuRoS](https://arxiv.org/abs/2511.22354), and [SwitchVLA](https://arxiv.org/abs/2506.03574). The first already preserves compatible executing actions during repair; the others cover operator interventions, event-triggered replanning, or execution-aware instruction switching. Our experiments must test a more specific protocol difference.

The PlanSys2 paper's limitations section also describes timestamp-based rejection of stale candidate plans. Its implementation/evaluation status was not established in the source review. Freshness rejection itself therefore cannot be assumed novel; investigate atomic activation and cargo-cancellation behavior against this existing proposal.

| Source | Relevant overlap | What to investigate |
| --- | --- | --- |
| [ROS-LLM](https://arxiv.org/abs/2406.19741) | Language interfaces and feedback-driven ROS task execution | Existing behavior execution and state/update handling |
| [Inner Monologue](https://innermonologue.github.io/) | Feedback used in language-driven embodied planning | How execution feedback changes plans and how it is evaluated |
| [VerifyLLM](https://arxiv.org/abs/2507.05118) | Pre-execution task-plan verification using language models and temporal logic | Differences between plan verification and concurrent execution-state freshness |

Our proposed focus is a deterministic boundary protocol for changing warehouse obligations during execution, with a controlled study of its costs. Whether that constitutes a publishable contribution remains open until a broader literature review and results exist.
