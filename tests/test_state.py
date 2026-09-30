"""Every parcel transition is allowed exactly where the table says, and nowhere else."""

import itertools
import json
import unittest

from warehouse_agent.state import (
    PARCEL_TRANSITIONS,
    Disposition,
    Goal,
    Hold,
    HoldKind,
    IllegalTransition,
    Parcel,
    Physical,
)

EXPECTED = {  # (transition, from physical, from disposition) -> (to physical, to disposition)
    ("activate", "awaiting_pickup", "inactive"): ("awaiting_pickup", "active"),
    ("cancel", "awaiting_pickup", "active"): ("awaiting_pickup", "cancelled"),
    ("pick_up", "awaiting_pickup", "active"): ("onboard", "active"),
    ("drop_off", "onboard", "active"): ("delivered", "active"),
    ("defer", "awaiting_pickup", "active"): ("awaiting_pickup", "deferred"),
    ("defer", "onboard", "active"): ("onboard", "deferred"),
}


class ParcelTests(unittest.TestCase):
    def test_every_transition_from_every_state(self):
        for transition, state, disposition in itertools.product(PARCEL_TRANSITIONS, Physical, Disposition):
            with self.subTest(transition=transition, state=str(state), disposition=str(disposition)):
                parcel = Parcel("P1", "PICK_A", "DROP_A", state, disposition)
                expected = EXPECTED.get((transition, state, disposition))
                self.assertEqual(parcel.can(transition), expected is not None)
                if expected:
                    parcel.apply(transition)
                    self.assertEqual((parcel.state, parcel.disposition), expected)
                else:
                    with self.assertRaises(IllegalTransition):
                        parcel.apply(transition)
                    self.assertEqual((parcel.state, parcel.disposition), (state, disposition))

    def test_delivered_parcels_never_change_again(self):
        parcel = Parcel("P1", "PICK_A", "DROP_A")
        for transition in ("activate", "pick_up", "drop_off"):
            parcel.apply(transition)
        self.assertEqual([t for t in PARCEL_TRANSITIONS if parcel.can(t)], [])

    def test_open_means_owed_work(self):
        self.assertFalse(Parcel("P1", "A", "B").open)
        self.assertTrue(Parcel("P1", "A", "B", Physical.ONBOARD, Disposition.ACTIVE).open)
        self.assertFalse(Parcel("P1", "A", "B", Physical.DELIVERED, Disposition.ACTIVE).open)
        self.assertFalse(Parcel("P1", "A", "B", Physical.ONBOARD, Disposition.DEFERRED).open)

    def test_snapshot_format_is_plain_json(self):
        parcel = Parcel("P1", "PICK_A", "DROP_A")
        parcel.apply("activate")
        parcel.apply("defer", reason="blocked aisle")
        self.assertEqual(json.loads(json.dumps(parcel.as_dict())),
                         {"pickup": "PICK_A", "drop": "DROP_A", "state": "awaiting_pickup",
                          "disposition": "deferred", "reason": "blocked aisle"})


class GoalTests(unittest.TestCase):
    def test_lifecycle(self):
        goal = Goal()
        self.assertFalse(goal.request_cancel())  # Nothing to cancel.
        goal.dispatch("PICK_A")
        with self.assertRaises(IllegalTransition):
            goal.dispatch("PICK_B")  # One goal at a time.
        self.assertTrue(goal.request_cancel())
        self.assertFalse(goal.request_cancel())  # Cancel is requested once.
        self.assertEqual(goal.finish(), ("PICK_A", True))
        self.assertIsNone(goal.stop)
        with self.assertRaises(IllegalTransition):
            goal.finish()
        goal.dispatch("PICK_B")
        self.assertEqual(goal.finish(), ("PICK_B", False))

    def test_hold_serializes_its_kind(self):
        self.assertEqual(Hold(HoldKind.NO_ROUTE, "No route").as_dict(), {"kind": "no_route", "message": "No route"})


if __name__ == "__main__":
    unittest.main()
