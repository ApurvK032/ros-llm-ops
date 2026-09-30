# Possible extensions after the related-work review

> **Archived planning document.** Kept for history; it describes plans from before the system was built, not the current behavior. See the [design](../design.md) and [results](../results.md) for the current system.

7 September 2026. These are proposed directions for discussion, not implemented behavior or a replacement for the current MVP contract. User priority remains a working robotics portfolio demo, then research, with no deadline.

## Recommendation

Focus on **reliable warehouse mission updates and recovery**, and accompany the implementation with a reproducible benchmark of failures around action boundaries. Extend the existing cancellation contract with one operator-authorized return workflow once basic state/execution works. Treat reduced pausing, richer clarification, and local models as later experiments.

The broad components are established. The closest [PlanSys2 repair paper](https://www.mdpi.com/2218-6581/15/4/80) preserves compatible running actions and describes timestamp rejection as a remedy for stale plans; implementation/evaluation of that remedy remains unverified in our review. [DFKI's agentic controller](https://arxiv.org/abs/2602.13081) exposes state/event checks and operator interventions, while reporting instruction-following failures. These are starting points and comparison targets, not proof that any proposed extension below is novel.

## 1. Cancellation that accounts for physical cargo

**Concrete example:** The operator says “Cancel P2,” but P2 becomes onboard while that instruction is being interpreted. Resolve the request against acknowledged state, preserve the parcel record, and ask whether to continue delivery, hold, or return to an allowed location. A return requires explicit authorization and becomes a new obligation. The cancelled customer's delivery and the still-required physical handling are separate records.

Compared with our current draft, the new feature is the explicit return workflow. The current contract offers continue or pause; it does not implement returns. Start with returning to the original pickup waypoint and logical unloading only. Add a physical `returned` outcome and separate return acknowledgement; never mark it as customer delivery. If the return destination fails, cargo remains onboard and return remains unresolved.

Implement serialized state transitions, correlated action acknowledgements, an activation barrier, and idempotent cargo operations. Classify work as completed, still required, cancelled, or requiring an authorized recovery action. Returning a parcel is a new task with its own preconditions; it is not reversing motor commands or deleting history. Action reversibility and compensation also have [general-agent prior work](https://arxiv.org/abs/2604.23283).

**Measure:** correct request resolution, lost/duplicated obligations, duplicate handling events, successful authorized returns, unresolved cargo, and receipt-to-effective latency. This is the strongest product/portfolio extension. Publication novelty would require establishing an execution-protocol or evaluation gap against earlier systems.

## 2. Accept compatible updates with less waiting

**Example:** A status observation unrelated to P2 should not force a P2 plan to be regenerated; a new pickup acknowledgement for P2 must. Track which parcel, map, action, and instruction facts each candidate depends on, then recheck those dependencies when activating it. Derive dependencies from action semantics in application code; do not trust an LLM to list every relevant dependency.

Begin with the existing conservative semantic-revision check. Later compare it against dependency-scoped validation, retaining the same obligation rules and cancellation barrier. Existing [PlanFence work](https://arxiv.org/abs/2609.03340) already proposes dependency-scoped action validation, so the research angle would be its behavior under robot execution constraints and our acknowledged handoff, not inventing selective checking.

Avoid invalidating on odometry alone, but refresh travel estimates or check pose assumptions when the robot's changed position matters. The final validation and dispatch decision must share a serialized boundary; reading fresh data and acting later leaves another race.

**Measure:** unnecessary proposal rejection, retries, idle time, instruction fulfillment and obligation errors. If shared guards make both variants equally correct, report a systems-cost result rather than inventing a correctness gain. Difficulty is higher than option 1.

## 3. Ask for clarification only when it changes the permissible action

**Example:** “Cancel the blue order” is ambiguous if two active parcels have that alias. Ask which one. “Cancel P2” while P2 is onboard requires a disposition decision. A valid priority request for a known, uncollected parcel can proceed without another confirmation.

Represent ambiguity as explicit alternatives grounded in current records. Ask about identity, intent, or authorized disposition rather than relying on an uncalibrated model confidence score. Preserve pending clarification IDs across replies and invalidate them when superseded. Begin with the current whole-mission hold; selectively continuing unrelated jobs is a separate, more complex policy requiring proof that they cannot affect the pending decision.

**Measure:** appropriate clarification, needless questions, incorrect assumptions, operator response time and request completion. Clarification itself is established; a measured policy for warehouse revisions is the candidate extension. A real usability claim would need a user study; scripted operator replies alone establish functional behavior.

## 4. A benchmark that deliberately exercises execution races

Construct episodes that vary request timing around navigation completion, pickup dispatch and acknowledgement, cancellation results, map changes, and plan activation. Inject delayed, duplicated and out-of-order observations without showing future events to the supervisor. Save the seed and event schedule, and minimize failing sequences into small replay cases.

Include at least: cancel just before pickup; cancel just after pickup; old proposal after pickup; late navigation success after cancel requested; two overlapping priorities; duplicate pickup acknowledgement; failed onboard drop; and failed authorized return. Expected obligations must come from scenario rules and executor evidence, independently of the tested validator.

**Measure:** every attempted episode and every valid request, not only accepted instructions or successful runs. Test liveness as well as invariant preservation: a controller that holds forever must not receive a perfect result. Time-dependent tests should record actual state at request receipt, not assume equal wall time means equal task progress.

**Deliverable:** a repeatable grid matrix plus a predeclared Gazebo subset, machine-readable outcomes, and replayable failure traces. This complements option 1 and could remain valuable even if no new planning algorithm is needed. Establishing a benchmark's research contribution requires comparison with existing dynamic-task benchmarks.

## 5. Reliable supervision using a smaller local model

Once the demo works, compare a hosted model and a small local tool-capable model under the identical tool schemas, planner, state protocol, and held-out scenarios. Include a scripted intent interface as a control to separate language errors from execution errors. Keep navigation and pause responsive when inference is slow or unavailable.

**Measure:** task/request correctness, valid tool arguments, clarification, tokens, latency and peak memory with simulation running. Compare measured outcomes before and after quantization. A smaller model may reduce deployment cost but may also increase retries. This is a useful deployment study, not inherently a new robotics method, and remains outside the first MVP.

## Smallest coherent project

Build option 1 with the core of option 4. Keep one robot, three parcels, one world, one hosted model, and one authorized return destination. Preserve the existing state and grid milestones; add return behavior only after their acceptance tests pass. Use stock Nav2 for motion. Decide whether to integrate PlanSys2 after inspecting/reproducing its closest repair example.

Start by reproducing two cases in a baseline: cancellation around pickup acknowledgement, and a proposal delayed across a cargo-state transition. Compare supported behavior honestly; an added return feature is a capability extension, not an accuracy win on a task the baseline never supported. Keep mandatory execution guards shared in all robot runs.

If these cases reveal reproducible failures, define a narrow mechanism and measure its effect. If the baseline already handles them, pursue timing efficiency, broader reproducibility, or the deployment comparison. Do not weaken a baseline or remove physical protections to manufacture a positive result.

Suggested project question: **Can a warehouse supervisor honor changing requests and retain every physical cargo obligation while limiting interruptions and recovery cost?**
