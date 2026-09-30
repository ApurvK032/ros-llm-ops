# Behavior contract — draft v0.1

> **Archived planning document.** Kept for history; it describes plans from before the system was built, not the current behavior. See the [design](../design.md) and [results](../results.md) for the current system.

> Update, 7 September 2026: the original simulation MVP is now implemented with local Qwen/Ollama. See [current status](status.md) and [MVP runbook](../runbook.md). This document preserves the earlier, broader design/setup plan; unimplemented contracts remain future work.

These are proposed implementation rules. No mission behavior is implemented yet. The examples in `scenarios/development.json` are acceptance specifications, not executed scenarios.

## World and identities

One robot R1 starts at HOME. Known waypoints are HOME, PICK_A, PICK_B, PICK_C, DROP_A, DROP_B, DROP_C. P1 goes PICK_A → DROP_A; P2 goes PICK_B → DROP_B; P3 goes PICK_C → DROP_C. The development fixture initially uses an empty 8×8 grid with four-neighbor motion; obstacle fixtures follow. Its coordinates are separate from the original WarehouseBot static baseline.

Parcel IDs survive replanning. Plan-local optimizer indices do not identify physical cargo. Location coordinates and aliases come from configuration; the model cannot invent them.

Physical lifecycle: `awaiting_pickup → onboard → delivered`. Mission disposition is separate: `active`, `cancelled`, or `deferred`. A delivered parcel cannot become awaiting pickup. Cancellation is legal only while awaiting pickup; deferment retains the current physical state and explicit outstanding work. A deferred parcel can be resumed through a later supported recovery decision.

## Supported requests

| Request | Meaning |
| --- | --- |
| Create mission | When idle, authorize a known set of parcels with known endpoints. A create request during an active mission needs clarification; dynamic order insertion is deferred. |
| Prioritize P3 | After the current action boundary, complete P3's remaining pickup/drop before other pending parcels, except drops protected by an effective onboard-first request. |
| Deliver onboard first | Capture the onboard parcel set when the revision becomes effective. Deliver that set before any new pickup. This finite set avoids a policy that changes on every future pickup. |
| Cancel P2 | Prevent P2's pickup if cancellation becomes effective before pickup acknowledgement. Resolve against current state, not state when the sentence was typed. |
| Status | Read-only snapshot with physical state, mission disposition, action, accepted revision, pending requests, and unresolved obligations. |

Repeated create messages with the same request ID are idempotent. A new request ID asking to start already active/delivered parcels produces clarification or `ALREADY_ACTIVE`/`ALREADY_DELIVERED`; it cannot reset cargo.

Only one priority parcel is active at once. A later accepted priority replaces the earlier priority policy, retaining every other delivery obligation. Onboard-first takes precedence over ordinary priority. Explicit incompatible hard requirements produce clarification. Unknown IDs, ambiguous aliases, and unsupported requests never mutate the mission.

## Cancellation and pause

Navigation and logical cargo handling are separate actions. A cancellation request immediately fences dispatch of a new action while the coordinator resolves it. Only the already-authorized active action may settle. Pause/cancel controls exist outside the LLM.

- Awaiting pickup, no cargo action active: terminate/cancel the navigation action if necessary; wait for terminal acknowledgement; accept cancellation at the boundary and remove that parcel's two remaining stops.
- Navigation succeeds before cancellation: arrival alone is not pickup. The dispatch fence still prevents logical pickup until the cancellation decision is applied.
- Pickup acknowledgement arrives first: record onboard cargo; do not report successful cancellation. Ask “P2 is onboard. Continue its original delivery, or pause with it onboard?” Hold all new action dispatch while awaiting the answer in the MVP.
- Logical pickup already dispatched: finish/acknowledge this short atomic action; do not pretend to roll it back. Apply the onboard rule if it succeeds.
- Already delivered: report that delivery completed and link the event. Cancellation is no longer applicable.
- Duplicate/late backend result: deduplicate by episode and action/operation ID, verify that it matches the recorded action, and retain it for diagnosis. An old episode event cannot affect the current episode.

Pausing does not cancel any parcel or erase any obligation. “Continue delivery” resumes the retained mission. “Return it” is unsupported until a return workflow is deliberately added; ask for a supported disposition. A physical robot emergency stop is outside this simulated MVP.

## Navigation failure

Default mission policy: initial navigation attempt plus at most one application-level retry. Nav2 may have its own internal recovery; record these separately. Per-attempt timeout is provisional (60 simulation seconds for the first experiment), plus a separately configured wall-time watchdog for a stalled simulator. Tune and freeze these values during the stock smoke test.

After both attempts fail, mark the affected job deferred with a reason and retain its cargo and outstanding stops. Continue other validated reachable work if the accepted policy permits it. An onboard drop failure retains onboard cargo. Report `partial/deferred`, not mission success. A timeout means the attempt failed; it does not establish permanent unreachability.

## Request and revision ordering

Serialize state changes in one coordinator. A request has a unique ID and receipt sequence; its lifecycle is `received → proposed → queued → effective`, or `clarification_required`, `rejected`, `superseded`, or `failed`. The accepted instruction revision advances only on effectiveness.

For the MVP, hold at the next action boundary while a mutating request is pending. Newer mutating requests supersede uncommitted work; an earlier effective revision stays in the ledger. A later “status” request does not supersede anything. If a clarification is pending, associate the reply with its request ID; a superseding request invalidates that clarification. Every superseded request gets an explicit outcome.

Record both monotonic semantic state revision and accepted instruction revision. Semantic changes include pickup/drop, action transitions, accepted instruction, effective plan, and map/reachability updates. Odometry ticks alone do not invalidate proposals. At final activation, require the proposal's relevant revision to match, validate obligations, and atomically swap the remaining queue. Do not increment the revision before comparing it against itself. A mismatched proposal is rejected and may be regenerated at most twice; then hold and report the issue.

For MVP strict revision equality may reject harmless changes. Measure this cost before adding selective dependency checks. Record proposal age and both rejection and retry counts.
