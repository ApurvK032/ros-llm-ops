"""Single-owner mission loop. Cargo changes only after verified navigation."""

import json
import time
from datetime import UTC, datetime
from pathlib import Path

from .planner import Planner, PlanningError
from .state import Disposition, Goal, Hold, HoldKind, Parcel, Physical


class Mission:
    def __init__(self, world, backend, journal, emit=None):
        self.world, self.backend = world, backend
        self.planner = Planner(world)
        self.parcels = {pid: Parcel(pid, p["pickup"], p["drop"]) for pid, p in world.config["parcels"].items()}
        self.goal = Goal()
        self.hold = None
        self.priority = None
        self.onboard_first = set()
        self.language_pending = False
        self.pending_request = None   # {"id": "R<n>", "superseded": bool} while the model interprets
        self.requests = 0
        self.commands = 0
        self.remaining_stops = []
        self.plan_pending = False
        self.closed = False
        self.attempts = {}
        self.revision = 0
        self.sequence = 0
        self.completed_reported = False
        self.emit = emit or (lambda event: None)
        Path(journal).parent.mkdir(parents=True, exist_ok=True)
        self.journal = Path(journal).open("x", encoding="utf-8")
        self.record("episode_started", config=world.config, backend=backend.name)

    @property
    def active(self):
        return self.goal.stop

    @property
    def cancel_requested(self):
        return self.goal.cancelling

    @property
    def paused(self):
        return self.hold is not None

    @property
    def finished(self):
        return not self.active and not any(p.open for p in self.parcels.values())

    def record(self, kind, **data):
        self.sequence += 1
        event = {"sequence": self.sequence, "time": datetime.now(UTC).isoformat(),
                 "monotonic": time.monotonic(), "revision": self.revision,
                 "type": kind, **data}
        self.journal.write(json.dumps(event, allow_nan=False)+"\n")
        self.journal.flush()
        self.emit(event)

    def snapshot(self):
        return {"revision": self.revision, "paused": self.paused,
                "hold": self.hold.as_dict() if self.hold else None,
                "language_pending": self.language_pending,
                "pending_request_id": self.pending_request["id"] if self.pending_request else None,
                "pose": list(self.backend.pose), "priority": self.priority,
                "onboard_first": sorted(self.onboard_first),
                "active": self.active.as_dict() if self.active else None,
                "remaining_stops": [s.as_dict() for s in self.remaining_stops if s != self.active]
                                   if not self.plan_pending else [],
                "plan_pending": self.plan_pending, "closed": self.closed,
                "cancel_requested": self.cancel_requested,
                "parcels": {pid: p.as_dict() for pid, p in self.parcels.items()}, "finished": self.finished}

    def cancel_motion(self):
        if self.goal.request_cancel():
            self.backend.cancel()
            self.record("navigation_cancel_requested", stop=self.active.as_dict())

    def hold_for(self, kind, message):
        """Pause with a reason; any active goal is cancelled and nothing new is dispatched until resume."""
        self.hold = Hold(kind, message)
        self.cancel_motion()

    def fail_request(self, error, request_id=None):
        """A request could not be interpreted or accepted: hold rather than act on it."""
        self.hold_for(HoldKind.REQUEST_FAILED, error)
        self.record("request_failed", request_id=request_id, error=error)

    # Request lifecycle: every request gets an ID and exactly one recorded outcome.
    def submit(self, text, model):
        """Start interpreting an operator request. Cargo and dispatch are fenced until resolve()."""
        if self.pending_request:
            raise RuntimeError(f"{self.pending_request['id']} is still being interpreted")
        self.requests += 1
        self.pending_request = {"id": f"R{self.requests}", "superseded": False}
        self.language_pending = True
        self.record("language_requested", request_id=self.pending_request["id"], text=text, model=model)
        return self.pending_request["id"]

    def resolve(self, request_id, intent=None, error=None, **metrics):
        """Finish the pending request: superseded, failed, or interpreted and applied.

        Returns "superseded", "failed", "status", "clarification" or "applied".
        """
        if not self.pending_request or self.pending_request["id"] != request_id:
            raise RuntimeError(f"{request_id} is not the pending request")
        superseded, self.pending_request = self.pending_request["superseded"], None
        try:
            if superseded:
                self.record("request_superseded", request_id=request_id,
                            reason="Direct pause took precedence over the pending model request")
                return "superseded"
            if error is not None:
                self.fail_request(error, request_id)
                return "failed"
            self.record("language_interpreted", request_id=request_id, intent=intent, **metrics)
            try:
                return self.apply(intent, request_id)
            except ValueError as exc:
                self.fail_request(str(exc), request_id)
                return "failed"
        finally:
            self.language_pending = False

    def command(self, operation):
        """Direct operator control (/pause, /resume, /status); bypasses the model entirely."""
        self.commands += 1
        if operation == "pause" and self.pending_request:
            self.pending_request["superseded"] = True
        return self.apply({"operation": operation, "parcels": [], "message": ""}, f"D{self.commands}")

    def apply(self, intent, request_id=None):
        """Apply validated intent to current cargo, never to a model's copy.

        Returns "status", "clarification" or "applied"; raises ValueError when the request is not allowed now.
        """
        if not isinstance(intent, dict):
            raise ValueError("The model did not return a request")
        operation = intent.get("operation")
        ids = intent.get("parcels", [])
        if not isinstance(ids, list) or any(not isinstance(p, str) or p not in self.parcels for p in ids):
            raise ValueError(f"Unknown parcel ID; available IDs are {', '.join(self.parcels)}")
        ids = list(dict.fromkeys(ids))
        # Journal what was actually applied (repeated IDs collapsed), not the raw model output.
        normalized = {"operation": operation, "parcels": ids, "message": intent.get("message", "")}
        if operation == "status":
            # Read-only, but still journaled so every request ends in exactly one recorded outcome.
            self.record("status_reported", request_id=request_id)
            return "status"
        if operation == "clarify":
            self.hold_for(HoldKind.CLARIFICATION, intent.get("message") or "Please clarify the request")
            self.record("clarification_required", request_id=request_id, question=self.hold.message)
            return "clarification"
        if operation == "create":
            if not ids:
                raise ValueError("Specify which parcels to deliver")
            if not self.finished or not all(self.parcels[p].can("activate") for p in ids):
                raise ValueError("A mission is active or a requested parcel was already handled")
            for pid in ids:
                self.parcels[pid].apply("activate")
            self.hold = None
            self.completed_reported = False
        elif operation == "prioritize":
            if len(ids) != 1 or not self.parcels[ids[0]].open:
                raise ValueError("Prioritize one active, undelivered parcel")
            self.priority = ids[0]
        elif operation == "onboard_first":
            self.onboard_first = {pid for pid, p in self.parcels.items() if p.open and p.state is Physical.ONBOARD}
        elif operation == "cancel":
            if not ids:
                raise ValueError("Specify which parcel to cancel")
            onboard = [pid for pid in ids if self.parcels[pid].state is Physical.ONBOARD]
            if onboard:
                # Cargo on the robot cannot be un-picked; ask instead, and cancel nothing from this request.
                names = ", ".join(onboard)
                others = [pid for pid in ids if pid not in onboard]
                question = (f"{names} {'is' if len(onboard) == 1 else 'are'} onboard, so nothing was cancelled"
                            + (f" (resend the cancel for {', '.join(others)} if still wanted)" if others else "")
                            + ". Resume the original delivery, or keep the mission paused?")
                self.hold_for(HoldKind.CLARIFICATION, question)
                self.record("clarification_required", request_id=request_id, question=question)
                return "clarification"
            refused = [pid for pid in ids if not self.parcels[pid].can("cancel")]
            if refused:
                raise ValueError(f"{', '.join(refused)} {'is' if len(refused) == 1 else 'are'} not an active, "
                                 "uncollected parcel")
            for pid in ids:
                self.parcels[pid].apply("cancel")
            if self.active and self.active.parcel in ids:
                self.cancel_motion()
        elif operation == "pause":
            self.hold_for(HoldKind.OPERATOR, "Paused by the operator")
        elif operation == "resume":
            self.hold = None
        else:
            raise ValueError("Unsupported operation")
        self.revision += 1
        if operation in {"create", "prioritize", "onboard_first", "cancel", "resume"}:
            self.plan_pending = True
        self.record("instruction_applied", request_id=request_id, intent=normalized, state=self.snapshot())
        return "applied"

    def tick(self):
        self.backend.spin()
        if self.active:
            outcome = self.backend.poll()
            if outcome is None:
                return
            # While language resolves, retain terminal arrival but prohibit cargo.
            if self.language_pending and not self.cancel_requested:
                return
            stop, cancelled = self.goal.finish()
            self.remaining_stops = []
            self.plan_pending = True
            self.record("navigation_finished", stop=stop.as_dict(), outcome=outcome,
                        pose=list(self.backend.pose), cancellation_requested=cancelled)
            parcel = self.parcels[stop.parcel]
            if cancelled or self.paused or parcel.disposition is not Disposition.ACTIVE:
                return
            arrival = self.world.arrival(self.backend.pose, stop.waypoint)
            if outcome == "succeeded" and arrival["accepted"]:
                parcel.apply("pick_up" if stop.kind == "pickup" else "drop_off")
                self.revision += 1
                self.record("cargo_" + stop.kind, parcel=stop.parcel, waypoint=stop.waypoint,
                            **arrival, pose=list(self.backend.pose),
                            parcel_position=self.world.station_position(stop.waypoint), state=str(parcel.state))
            else:
                key = (stop.parcel, stop.kind)
                self.attempts[key] = self.attempts.get(key, 0)+1
                if self.attempts[key] >= 2:
                    parcel.apply("defer", reason=(f"Navigation {outcome}; approach error "
                                                  f"{arrival['position_error']:.3f} m, heading error "
                                                  f"{arrival['heading_error']:.3f} rad, facing error "
                                                  f"{arrival['facing_error']:.3f} rad, parcel clearance "
                                                  f"{arrival['parcel_clearance']:.3f} m"))
                    self.revision += 1
                    self.record("parcel_deferred", parcel=stop.parcel, reason=parcel.reason)
                else:
                    self.record("navigation_retry", stop=stop.as_dict(), arrival=arrival)
        if self.paused or self.language_pending:
            return
        try:
            route, distance = self.planner.plan(self.parcels, self.backend.pose, self.priority, self.onboard_first)
        except PlanningError as exc:
            # Hold until the operator resumes; retrying every tick would flood the journal.
            self.hold = Hold(HoldKind.NO_ROUTE, str(exc))
            self.revision += 1
            self.record("planning_failed", reason=str(exc), pose=list(self.backend.pose))
            return
        self.remaining_stops = route
        self.plan_pending = False
        if not route:
            if not self.completed_reported and any(p.disposition is not Disposition.INACTIVE
                                                   for p in self.parcels.values()):
                self.completed_reported = True
                partial = any(p.disposition is Disposition.DEFERRED for p in self.parcels.values())
                self.record("mission_finished", result="partial" if partial else "complete", state=self.snapshot())
            return
        self.goal.dispatch(route[0])
        self.record("plan_selected", stops=[s.as_dict() for s in route], estimated_distance_m=distance)
        self.backend.navigate(self.world.waypoints[self.active.waypoint])
        self.record("navigation_started", stop=self.active.as_dict())

    def close(self):
        self.cancel_motion()
        deadline = time.monotonic()+10
        while self.active and time.monotonic() < deadline:
            self.backend.spin()
            outcome = self.backend.poll()
            if outcome is not None:
                stop, _ = self.goal.finish()
                self.record("navigation_finished", stop=stop.as_dict(), outcome=outcome,
                            cancellation_requested=True, pose=list(self.backend.pose))
            time.sleep(0.05)
        self.closed = True
        self.remaining_stops = []
        self.plan_pending = False
        self.record("episode_closed", cancellation_acknowledged=self.active is None, state=self.snapshot())
        if hasattr(self.backend, "publish_status"):
            self.backend.publish_status(self.snapshot())
        self.journal.close()
        self.backend.close()
