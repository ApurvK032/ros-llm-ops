"""Explicit state machines: parcels, the single navigation goal, and mission holds.

Every change goes through a named transition that checks where it starts from, so an illegal change raises
IllegalTransition instead of silently corrupting mission state. Snapshots keep the plain-string JSON format.
"""

from dataclasses import dataclass
from enum import StrEnum


class IllegalTransition(RuntimeError):
    """A change was attempted from a state that does not allow it. Escaping this is a supervisor bug."""


class Physical(StrEnum):
    AWAITING_PICKUP = "awaiting_pickup"
    ONBOARD = "onboard"
    DELIVERED = "delivered"


class Disposition(StrEnum):
    INACTIVE = "inactive"
    ACTIVE = "active"
    CANCELLED = "cancelled"
    DEFERRED = "deferred"


# transition -> (allowed (physical, disposition) sources, (new physical, new disposition)); None keeps the value.
PARCEL_TRANSITIONS = {
    "activate": ({(Physical.AWAITING_PICKUP, Disposition.INACTIVE)}, (None, Disposition.ACTIVE)),
    "cancel": ({(Physical.AWAITING_PICKUP, Disposition.ACTIVE)}, (None, Disposition.CANCELLED)),
    "pick_up": ({(Physical.AWAITING_PICKUP, Disposition.ACTIVE)}, (Physical.ONBOARD, None)),
    "drop_off": ({(Physical.ONBOARD, Disposition.ACTIVE)}, (Physical.DELIVERED, None)),
    "defer": ({(Physical.AWAITING_PICKUP, Disposition.ACTIVE), (Physical.ONBOARD, Disposition.ACTIVE)},
              (None, Disposition.DEFERRED)),
}


@dataclass
class Parcel:
    """Physical state and mission disposition are separate: cancelling or deferring never moves cargo."""
    id: str
    pickup: str
    drop: str
    state: Physical = Physical.AWAITING_PICKUP
    disposition: Disposition = Disposition.INACTIVE
    reason: str | None = None

    def can(self, transition):
        return (self.state, self.disposition) in PARCEL_TRANSITIONS[transition][0]

    def apply(self, transition, reason=None):
        if not self.can(transition):
            raise IllegalTransition(f"{transition} {self.id} is not allowed while {self.state}/{self.disposition}")
        state, disposition = PARCEL_TRANSITIONS[transition][1]
        self.state = state or self.state
        self.disposition = disposition or self.disposition
        if reason:
            self.reason = reason

    @property
    def open(self):
        """Still owed work: part of the mission and not yet delivered."""
        return self.disposition is Disposition.ACTIVE and self.state is not Physical.DELIVERED

    def as_dict(self):
        record = {"pickup": self.pickup, "drop": self.drop, "state": str(self.state),
                  "disposition": str(self.disposition)}
        if self.reason:
            record["reason"] = self.reason
        return record


class Goal:
    """At most one Nav2 goal: idle -> active -> (cancelling ->) idle."""

    def __init__(self):
        self.stop = None
        self.cancelling = False

    def dispatch(self, stop):
        if self.stop is not None:
            raise IllegalTransition(f"cannot dispatch {stop} while {self.stop} is still active")
        self.stop, self.cancelling = stop, False

    def request_cancel(self):
        """Returns True only when this call turned an active goal into a cancelling one."""
        if self.stop is None or self.cancelling:
            return False
        self.cancelling = True
        return True

    def finish(self):
        """Nav2 reported a terminal result: returns the stop and whether a cancel had been requested."""
        if self.stop is None:
            raise IllegalTransition("a goal finished while none was active")
        stop, cancelled = self.stop, self.cancelling
        self.stop, self.cancelling = None, False
        return stop, cancelled


class HoldKind(StrEnum):
    OPERATOR = "operator"
    CLARIFICATION = "clarification"
    REQUEST_FAILED = "request_failed"
    NO_ROUTE = "no_route"


@dataclass(frozen=True)
class Hold:
    """Why the mission is paused; every pause has one, and only resume or a new mission clears it."""
    kind: HoldKind
    message: str

    def as_dict(self):
        return {"kind": str(self.kind), "message": self.message}
