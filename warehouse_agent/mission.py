"""Single-owner mission loop. Cargo changes only after verified navigation."""

import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from .planner import Planner


class Mission:
    def __init__(self, world, backend, journal, emit=None):
        self.world, self.backend = world, backend
        self.planner = Planner(world)
        self.parcels = {pid: {**p, "state": "awaiting_pickup", "disposition": "inactive"}
                        for pid, p in world.config["parcels"].items()}
        self.priority = None
        self.onboard_first = set()
        self.paused = False
        self.language_pending = False
        self.active = None
        self.remaining_stops = []
        self.plan_pending = False
        self.closed = False
        self.cancel_requested = False
        self.attempts = {}
        self.revision = 0
        self.sequence = 0
        self.completed_reported = False
        self.emit = emit or (lambda event: None)
        Path(journal).parent.mkdir(parents=True, exist_ok=True)
        self.journal = Path(journal).open("x", encoding="utf-8")
        self.record("episode_started", config=world.config, backend=backend.name)

    def record(self, kind, **data):
        self.sequence += 1
        event = {"sequence": self.sequence, "time": datetime.now(timezone.utc).isoformat(),
                 "monotonic": time.monotonic(), "revision": self.revision,
                 "type": kind, **data}
        self.journal.write(json.dumps(event, allow_nan=False)+"\n")
        self.journal.flush()
        self.emit(event)

    def snapshot(self):
        return {"revision": self.revision, "paused": self.paused,
                "language_pending": self.language_pending,
                "pose": list(self.backend.pose), "priority": self.priority,
                "onboard_first": sorted(self.onboard_first),
                "active": self.active.as_dict() if self.active else None,
                "remaining_stops": [s.as_dict() for s in self.remaining_stops if s != self.active]
                                   if not self.plan_pending else [],
                "plan_pending": self.plan_pending, "closed": self.closed,
                "cancel_requested": self.cancel_requested,
                "parcels": copy.deepcopy(self.parcels), "finished": self.finished}

    @property
    def finished(self):
        return not self.active and not any(p["disposition"] == "active" and p["state"] != "delivered"
                                          for p in self.parcels.values())

    def cancel_motion(self):
        if self.active and not self.cancel_requested:
            self.backend.cancel()
            self.cancel_requested = True
            self.record("navigation_cancel_requested", stop=self.active.as_dict())

    def apply(self, intent):
        """Apply validated intent to current cargo, never to a model's copy."""
        operation = intent.get("operation")
        ids = intent.get("parcels", [])
        if not isinstance(ids, list) or any(not isinstance(p, str) or p not in self.parcels for p in ids):
            raise ValueError("Unknown parcel ID; available IDs are P1, P2 and P3")
        ids = list(dict.fromkeys(ids))
        if operation == "status":
            return self.snapshot()
        if operation == "clarify":
            self.paused = True
            self.cancel_motion()
            self.record("clarification_required", question=intent.get("message", "Please clarify the request"))
            return self.snapshot()
        if operation == "create":
            if not ids:
                raise ValueError("Specify which parcels to deliver")
            if not self.finished or any(self.parcels[p]["state"] != "awaiting_pickup" or
                                        self.parcels[p]["disposition"] != "inactive" for p in ids):
                raise ValueError("A mission is active or a requested parcel was already handled")
            for pid in ids:
                self.parcels[pid]["disposition"] = "active"
            self.paused = False
            self.completed_reported = False
        elif operation == "prioritize":
            if len(ids) != 1 or self.parcels[ids[0]]["disposition"] != "active" or self.parcels[ids[0]]["state"] == "delivered":
                raise ValueError("Prioritize one active, undelivered parcel")
            self.priority = ids[0]
        elif operation == "onboard_first":
            self.onboard_first = {pid for pid, p in self.parcels.items()
                                  if p["state"] == "onboard" and p["disposition"] == "active"}
        elif operation == "cancel":
            if not ids:
                raise ValueError("Specify which parcel to cancel")
            for pid in ids:
                p = self.parcels[pid]
                if p["state"] == "onboard":
                    self.paused = True
                    self.cancel_motion()
                    self.record("clarification_required", question=f"{pid} is onboard. Resume its original delivery, or keep the mission paused?")
                    return self.snapshot()
                if p["state"] == "delivered" or p["disposition"] != "active":
                    raise ValueError(f"{pid} is not an active, uncollected parcel")
            for pid in ids:
                self.parcels[pid]["disposition"] = "cancelled"
            if self.active and self.active.parcel in ids:
                self.cancel_motion()
        elif operation == "pause":
            self.paused = True
            self.cancel_motion()
        elif operation == "resume":
            self.paused = False
        else:
            raise ValueError("Unsupported operation")
        self.revision += 1
        if operation in {"create", "prioritize", "onboard_first", "cancel", "resume"}:
            self.plan_pending = True
        self.record("instruction_applied", intent=intent, state=self.snapshot())
        return self.snapshot()

    def tick(self):
        self.backend.spin()
        if self.active:
            outcome = self.backend.poll()
            if outcome is None:
                return
            # While language resolves, retain terminal arrival but prohibit cargo.
            if self.language_pending and not self.cancel_requested:
                return
            stop = self.active
            self.active = None
            self.remaining_stops = []
            self.plan_pending = True
            cancelled = self.cancel_requested
            self.cancel_requested = False
            self.record("navigation_finished", stop=stop.as_dict(), outcome=outcome,
                        pose=list(self.backend.pose), cancellation_requested=cancelled)
            parcel = self.parcels[stop.parcel]
            if cancelled or self.paused or parcel["disposition"] != "active":
                return
            arrival = self.world.arrival(self.backend.pose, stop.waypoint)
            if outcome == "succeeded" and arrival["accepted"]:
                expected = "awaiting_pickup" if stop.kind == "pickup" else "onboard"
                if parcel["state"] != expected:
                    raise RuntimeError("Cargo state changed unexpectedly")
                parcel["state"] = "onboard" if stop.kind == "pickup" else "delivered"
                self.revision += 1
                self.record("cargo_" + stop.kind, parcel=stop.parcel, waypoint=stop.waypoint,
                            **arrival, pose=list(self.backend.pose),
                            parcel_position=self.world.station_position(stop.waypoint), state=parcel["state"])
            else:
                key = (stop.parcel, stop.kind)
                self.attempts[key] = self.attempts.get(key, 0)+1
                if self.attempts[key] >= 2:
                    parcel["disposition"] = "deferred"
                    parcel["reason"] = (f"Navigation {outcome}; approach error {arrival['position_error']:.3f} m, "
                                        f"heading error {arrival['heading_error']:.3f} rad, "
                                        f"facing error {arrival['facing_error']:.3f} rad, "
                                        f"parcel clearance {arrival['parcel_clearance']:.3f} m")
                    self.revision += 1
                    self.record("parcel_deferred", parcel=stop.parcel, reason=parcel["reason"])
                else:
                    self.record("navigation_retry", stop=stop.as_dict(), arrival=arrival)
        if self.paused or self.language_pending:
            return
        route, distance = self.planner.plan(self.parcels, self.backend.pose, self.priority, self.onboard_first)
        self.remaining_stops = route
        self.plan_pending = False
        if not route:
            if not self.completed_reported and any(p["disposition"] != "inactive" for p in self.parcels.values()):
                self.completed_reported = True
                self.record("mission_finished", result="partial" if any(p["disposition"] == "deferred" for p in self.parcels.values()) else "complete", state=self.snapshot())
            return
        self.active = route[0]
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
                self.record("navigation_finished", stop=self.active.as_dict(), outcome=outcome,
                            cancellation_requested=True, pose=list(self.backend.pose))
                self.active = None
            time.sleep(0.05)
        self.closed = True
        self.remaining_stops = []
        self.plan_pending = False
        self.record("episode_closed", cancellation_acknowledged=self.active is None, state=self.snapshot())
        if hasattr(self.backend, "publish_status"):
            self.backend.publish_status(self.snapshot())
        self.journal.close()
        self.backend.close()
