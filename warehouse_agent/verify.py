"""Independent journal checker: rebuild mission state from the events alone and check the supervisor's rules.

It shares no code with mission.py or planner.py, so a supervisor bug cannot hide itself here. Limits come from
the config recorded in each journal's episode_started event, which also lets it check journals written by older
versions of the supervisor.
"""

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

PHYSICAL = ("awaiting_pickup", "onboard", "delivered")
DISPOSITIONS = ("inactive", "active", "cancelled", "deferred")
# Measurement recorded on a cargo event -> (config limit, whether the value must be at most or at least the limit).
TRANSFER_LIMITS = {"position_error": ("arrival_tolerance", "max"), "heading_error": ("heading_tolerance", "max"),
                   "facing_error": ("facing_tolerance", "max"),
                   "parcel_clearance": ("minimum_parcel_clearance", "min")}
KNOWN_EVENTS = {"episode_started", "episode_closed", "language_requested", "language_interpreted",
                "instruction_applied", "clarification_required", "request_failed", "request_superseded",
                "plan_selected", "navigation_started", "navigation_cancel_requested", "navigation_finished",
                "cargo_pickup", "cargo_drop", "navigation_retry", "parcel_deferred", "planning_failed",
                "mission_finished", "check_passed"}


@dataclass
class Finding:
    code: str
    sequence: int | None
    message: str


@dataclass
class Report:
    path: str
    events: int = 0
    violations: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    parcels: dict = field(default_factory=dict)

    @property
    def ok(self):
        return not self.violations

    def as_dict(self):
        return {"path": self.path, "ok": self.ok, "events": self.events, "parcels": self.parcels,
                "violations": [asdict(v) for v in self.violations], "warnings": [asdict(w) for w in self.warnings]}


def obligations(parcels):
    """Stops still owed: pickup and drop for active parcels on their shelf, drop only for active onboard parcels."""
    owed = set()
    for pid, p in parcels.items():
        if p["disposition"] != "active" or p["state"] == "delivered":
            continue
        if p["state"] == "awaiting_pickup":
            owed.add((pid, "pickup"))
        owed.add((pid, "drop"))
    return owed


def check_snapshot(snapshot):
    """Rules that must hold for any published snapshot. Returns a list of Findings (empty when consistent)."""
    found = []

    def bad(code, message):
        found.append(Finding(code, None, message))

    parcels = snapshot["parcels"]
    for pid, p in parcels.items():
        if p["state"] not in PHYSICAL or p["disposition"] not in DISPOSITIONS:
            bad("UNKNOWN_STATE", f"{pid} has state {p['state']!r} and disposition {p['disposition']!r}")
        elif p["disposition"] in {"inactive", "cancelled"} and p["state"] != "awaiting_pickup":
            bad("ILLEGAL_STATE", f"{pid} is {p['disposition']} but {p['state']}; only shelf parcels can be that")
    active = snapshot.get("active")
    if active:
        p = parcels.get(active["parcel"])
        expected = "awaiting_pickup" if active["kind"] == "pickup" else "onboard"
        if p is None or p["state"] != expected:
            bad("ACTIVE_STOP_INCONSISTENT", f"active {active['kind']} for {active['parcel']} but parcel is "
                f"{p and p['state']}")
        elif "cancel_requested" in snapshot and p["disposition"] != "active" and not snapshot["cancel_requested"]:
            bad("ACTIVE_STOP_INCONSISTENT", f"goal toward {p['disposition']} parcel {active['parcel']} without a cancel")
    # Older journals did not record the planned stops; only judge what a snapshot actually contains.
    if "remaining_stops" in snapshot and "plan_pending" in snapshot and not snapshot["plan_pending"] \
            and not snapshot.get("closed"):
        stops = ([active] if active else []) + list(snapshot.get("remaining_stops", []))
        planned = [(s["parcel"], s["kind"]) for s in stops]
        if len(planned) != len(set(planned)) or set(planned) != obligations(parcels):
            bad("OBLIGATIONS_MISMATCH", f"planned stops {sorted(planned)} != owed {sorted(obligations(parcels))}")
    if "finished" in snapshot:
        finished = not active and not any(p["disposition"] == "active" and p["state"] != "delivered"
                                          for p in parcels.values())
        if snapshot["finished"] != finished:
            bad("FINISHED_FLAG_WRONG", f"finished={snapshot['finished']} but work remaining={not finished}")
    if snapshot.get("hold_reason") and not snapshot.get("paused"):
        bad("HOLD_NOT_PAUSED", "a planning hold must keep the mission paused")
    return found


class Replay:
    """Fold journal events into state, recording every rule the event sequence breaks."""

    def __init__(self, path):
        self.report = Report(str(path))
        self.config = None
        self.parcels = {}
        self.paused = False
        self.goal = None              # stop dict of the dispatched Nav2 goal
        self.cancel_requested = False
        self.arrival = None           # last navigation_finished not yet consumed by a transfer or failure
        self.plan = None              # stops of the last plan_selected
        self.request = None           # None, "requested" or "interpreted"
        self.retries = set()          # (parcel, kind) stops that already had a recorded retry
        self.closed = False
        self.previous = None

    def violation(self, event, code, message):
        self.report.violations.append(Finding(code, event.get("sequence") if event else None, message))

    def warning(self, event, code, message):
        self.report.warnings.append(Finding(code, event.get("sequence") if event else None, message))

    def run(self, events):
        for event in events:
            self.step(event)
        self.finish()
        return self.report

    def step(self, e):
        self.report.events += 1
        kind = e.get("type")
        prev = self.previous
        if prev is None and kind != "episode_started":
            self.violation(e, "MISSING_START", "journal must begin with episode_started")
        if prev is not None:
            if e.get("sequence") != prev.get("sequence", 0) + 1:
                self.violation(e, "SEQUENCE_GAP", f"sequence {e.get('sequence')} follows {prev.get('sequence')}")
            if e.get("monotonic", math.inf) < prev.get("monotonic", -math.inf):
                self.violation(e, "TIME_BACKWARDS", "monotonic time decreased")
            if e.get("revision", math.inf) < prev.get("revision", -math.inf):
                self.violation(e, "REVISION_DECREASED", "state revision decreased")
        if self.closed:
            self.violation(e, "EVENT_AFTER_CLOSE", f"{kind} recorded after episode_closed")
        self.previous = e
        handler = getattr(self, "on_" + kind, None) if kind in KNOWN_EVENTS else None
        if handler is None:
            self.warning(e, "UNKNOWN_EVENT", f"event type {kind!r} is not checked")
        elif kind == "episode_started" or self.config is not None:
            try:
                handler(e)
            except (KeyError, TypeError, AttributeError, ValueError) as exc:
                self.violation(e, "MALFORMED_EVENT", f"{kind} is missing or has invalid fields ({exc!r})")
        if "state" in e and isinstance(e["state"], dict) and self.config is not None:
            try:
                self.compare(e, e["state"])
            except (KeyError, TypeError, AttributeError, ValueError) as exc:
                self.violation(e, "MALFORMED_EVENT", f"{kind} has an invalid state snapshot ({exc!r})")

    def compare(self, e, snapshot):
        """The supervisor's own snapshot must agree with the independently replayed state."""
        for pid, mine in self.parcels.items():
            theirs = snapshot.get("parcels", {}).get(pid)
            if not theirs or (theirs["state"], theirs["disposition"]) != (mine["state"], mine["disposition"]):
                self.violation(e, "STATE_MISMATCH", f"{pid}: journal says {theirs and (theirs['state'], theirs['disposition'])}"
                                                    f", replay says {(mine['state'], mine['disposition'])}")
        if "paused" in snapshot and snapshot["paused"] != self.paused:
            self.violation(e, "STATE_MISMATCH", f"paused: journal says {snapshot['paused']}, replay says {self.paused}")
        for finding in check_snapshot(snapshot):
            self.violation(e, finding.code, finding.message)

    # Episode and requests -------------------------------------------------------------------------------
    def on_episode_started(self, e):
        if self.config is not None:
            self.violation(e, "DUPLICATE_START", "second episode_started in one journal")
            return
        self.config = e["config"]
        self.parcels = {pid: {"state": "awaiting_pickup", "disposition": "inactive", **p}
                        for pid, p in self.config["parcels"].items()}

    def on_language_requested(self, e):
        if self.request:
            self.violation(e, "OVERLAPPING_REQUESTS", "a new request started before the previous one resolved")
        self.request = "requested"

    def on_language_interpreted(self, e):
        if self.request != "requested":
            self.violation(e, "ORPHAN_INTERPRETATION", "interpretation without a pending request")
        # A status request is read-only: interpreting it is its final outcome.
        self.request = None if e.get("intent", {}).get("operation") == "status" else "interpreted"

    def resolve(self):
        if self.request == "interpreted":
            self.request = None

    def on_request_superseded(self, e):
        if not self.request:
            self.violation(e, "ORPHAN_OUTCOME", "request_superseded without a pending request")
        self.request = None

    def on_request_failed(self, e):
        self.request = None
        self.paused = True

    def on_clarification_required(self, e):
        self.resolve()
        self.paused = True

    def on_instruction_applied(self, e):
        self.resolve()
        intent = e.get("intent", {})
        # A repeated ID in one instruction still refers to one parcel.
        op, ids = intent.get("operation"), list(dict.fromkeys(intent.get("parcels", [])))
        for pid in ids:
            if pid not in self.parcels:
                self.violation(e, "UNKNOWN_PARCEL", f"{op} applied to unknown parcel {pid}")
                return
        if op == "create":
            for pid in ids:
                p = self.parcels[pid]
                if (p["state"], p["disposition"]) != ("awaiting_pickup", "inactive"):
                    self.violation(e, "ILLEGAL_CREATE", f"{pid} activated while {p['state']}/{p['disposition']}")
                p["disposition"] = "active"
            self.paused = False
        elif op == "cancel":
            for pid in ids:
                p = self.parcels[pid]
                if (p["state"], p["disposition"]) != ("awaiting_pickup", "active"):
                    self.violation(e, "ILLEGAL_CANCEL", f"{pid} cancelled while {p['state']}/{p['disposition']}")
                p["disposition"] = "cancelled"
        elif op == "pause":
            self.paused = True
        elif op == "resume":
            self.paused = False
        elif op == "prioritize" and len(ids) != 1:
            self.violation(e, "ILLEGAL_PRIORITY", f"prioritize needs exactly one parcel, got {ids}")

    # Planning and navigation ----------------------------------------------------------------------------
    def dispatch_allowed(self, e, what):
        if self.paused:
            self.violation(e, "DISPATCH_WHILE_PAUSED", f"{what} while the mission is paused")
        if self.request:
            self.violation(e, "DISPATCH_DURING_REQUEST", f"{what} while a request is being interpreted")
        if self.goal:
            self.violation(e, "CONCURRENT_GOALS", f"{what} while a Nav2 goal is still active")

    def on_plan_selected(self, e):
        self.dispatch_allowed(e, "planning")
        stops = e.get("stops", [])
        owed = obligations(self.parcels)
        planned = [(s["parcel"], s["kind"]) for s in stops]
        if len(planned) != len(set(planned)) or set(planned) != owed:
            self.violation(e, "PLAN_OBLIGATION_MISMATCH", f"plan {planned} != owed {sorted(owed)}")
        for s in stops:
            p = self.parcels.get(s["parcel"])
            if p and s["waypoint"] != p["pickup" if s["kind"] == "pickup" else "drop"]:
                self.violation(e, "PLAN_WRONG_WAYPOINT", f"{s['kind']} {s['parcel']} planned at {s['waypoint']}")
        for pid in {s["parcel"] for s in stops}:
            if {(pid, "pickup"), (pid, "drop")} <= set(planned) and \
                    planned.index((pid, "pickup")) > planned.index((pid, "drop")):
                self.violation(e, "PLAN_PRECEDENCE", f"{pid} dropped before it is picked up")
        self.plan = stops

    def on_navigation_started(self, e):
        self.dispatch_allowed(e, "navigation_started")
        if not self.plan or e["stop"] != self.plan[0]:
            self.violation(e, "DISPATCH_NOT_PLANNED", f"dispatched {e['stop']} but the plan starts with "
                                                      f"{self.plan[0] if self.plan else None}")
        self.plan = None
        self.goal, self.cancel_requested, self.arrival = e["stop"], False, None

    def on_navigation_cancel_requested(self, e):
        if not self.goal or e["stop"] != self.goal:
            self.violation(e, "CANCEL_WITHOUT_GOAL", "cancel requested for a goal that is not active")
        self.cancel_requested = True

    def on_navigation_finished(self, e):
        if not self.goal or e["stop"] != self.goal:
            self.violation(e, "FINISH_WITHOUT_GOAL", "navigation finished for a goal that is not active")
        elif self.request and not self.cancel_requested:
            self.violation(e, "FENCE_BROKEN", "goal result consumed while a request was being interpreted")
        if e.get("cancellation_requested", False) != self.cancel_requested:
            self.violation(e, "CANCEL_FLAG_MISMATCH", "cancellation flag disagrees with the recorded cancel")
        self.arrival, self.goal = e, None

    def take_arrival(self, e, stop):
        """Consume the arrival that justifies this event; each arrival justifies at most one outcome."""
        arrival, self.arrival = self.arrival, None
        if arrival is None or arrival["stop"]["parcel"] != stop[0] or arrival["stop"]["kind"] != stop[1]:
            return None
        return arrival

    def transfer(self, e, kind):
        pid = e.get("parcel")
        p = self.parcels.get(pid)
        if p is None:
            self.violation(e, "UNKNOWN_PARCEL", f"transfer of unknown parcel {pid}")
            return
        arrival = self.take_arrival(e, (pid, kind))
        if arrival is None:
            self.violation(e, "TRANSFER_WITHOUT_ARRIVAL", f"{kind} of {pid} without a matching Nav2 arrival")
        elif arrival.get("outcome") != "succeeded" or arrival.get("cancellation_requested"):
            self.violation(e, "TRANSFER_WITHOUT_ARRIVAL", f"{kind} of {pid} after outcome "
                                                          f"{arrival.get('outcome')!r}, cancel={arrival.get('cancellation_requested')}")
        if self.paused:
            self.violation(e, "TRANSFER_WHILE_PAUSED", f"{kind} of {pid} while paused")
        if p["disposition"] != "active":
            self.violation(e, "TRANSFER_INACTIVE_PARCEL", f"{kind} of {p['disposition']} parcel {pid}")
        waypoint = p["pickup" if kind == "pickup" else "drop"]
        if e.get("waypoint") != waypoint:
            self.violation(e, "TRANSFER_WRONG_PLACE", f"{kind} of {pid} at {e.get('waypoint')}, expected {waypoint}")
        for metric, (limit, sense) in TRANSFER_LIMITS.items():
            if metric in e and limit in self.config:
                value, bound = e[metric], self.config[limit]
                if (value > bound) if sense == "max" else (value < bound):
                    self.violation(e, "TRANSFER_OUT_OF_TOLERANCE", f"{kind} of {pid}: {metric}={value:.3f}, "
                                                                   f"{'max' if sense == 'max' else 'min'} {bound}")
        if "accepted" in e and e["accepted"] is not True:
            self.violation(e, "TRANSFER_OUT_OF_TOLERANCE", f"{kind} of {pid} recorded as not accepted")
        before, after = ("awaiting_pickup", "onboard") if kind == "pickup" else ("onboard", "delivered")
        if p["state"] != before:
            self.violation(e, "ILLEGAL_TRANSITION", f"{kind} of {pid} while {p['state']}")
        if e.get("state") != after:
            self.violation(e, "ILLEGAL_TRANSITION", f"{kind} of {pid} recorded state {e.get('state')!r}")
        p["state"] = after

    def on_cargo_pickup(self, e):
        self.transfer(e, "pickup")

    def on_cargo_drop(self, e):
        self.transfer(e, "drop")

    def on_navigation_retry(self, e):
        stop = e.get("stop", {})
        if self.take_arrival(e, (stop.get("parcel"), stop.get("kind"))) is None:
            self.violation(e, "RETRY_WITHOUT_ARRIVAL", f"retry of {stop} without a matching Nav2 result")
        self.retries.add((stop.get("parcel"), stop.get("kind")))

    def on_parcel_deferred(self, e):
        pid = e.get("parcel")
        p = self.parcels.get(pid)
        if p is None or p["disposition"] != "active" or p["state"] == "delivered":
            self.violation(e, "ILLEGAL_DEFER", f"deferred {pid} while {p and (p['state'], p['disposition'])}")
            return
        arrival = self.arrival
        if arrival is None or arrival["stop"]["parcel"] != pid:
            self.violation(e, "DEFER_WITHOUT_ARRIVAL", f"deferred {pid} without a matching Nav2 result")
        elif (pid, arrival["stop"]["kind"]) not in self.retries:
            self.violation(e, "RETRY_POLICY", f"{pid} deferred on its first failed {arrival['stop']['kind']}; "
                                              "one retry is required")
        self.arrival = None
        p["disposition"] = "deferred"

    def on_planning_failed(self, e):
        self.paused = True

    def on_mission_finished(self, e):
        remaining = obligations(self.parcels)
        if remaining or self.goal:
            self.violation(e, "FINISHED_WITH_WORK_LEFT", f"mission finished with {sorted(remaining)} still owed")
        partial = any(p["disposition"] == "deferred" for p in self.parcels.values())
        if e.get("result") != ("partial" if partial else "complete"):
            self.violation(e, "RESULT_MISMATCH", f"result {e.get('result')!r} but deferred parcels={partial}")

    def on_episode_closed(self, e):
        if self.request:
            self.warning(e, "UNRESOLVED_REQUEST", "episode closed while a request was pending")
        self.closed = True

    def on_check_passed(self, e):
        pass  # Recorded by the live integration check; informational only.

    def finish(self):
        if self.config is None:
            self.violation(None, "MISSING_START", "no episode_started event")
        if not self.closed:
            self.warning(None, "NOT_CLOSED", "journal ends without episode_closed (truncated or still running)")
            if self.request:
                self.warning(None, "UNRESOLVED_REQUEST", "journal ends with a request pending")
        self.report.parcels = {pid: {"state": p["state"], "disposition": p["disposition"]}
                               for pid, p in self.parcels.items()}


def verify_events(events, path="<events>"):
    return Replay(path).run(events)


def verify_file(path):
    events = []
    for number, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        if line.strip():
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                report = Report(str(path))
                report.violations.append(Finding("MALFORMED_LINE", None, f"line {number} is not valid JSON"))
                return report
    return verify_events(events, str(path))
