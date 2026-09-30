"""Typed MissionStatus messages carry exactly what the supervisor's snapshots say. Needs the built ros_ws."""

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT/"tests"))

from test_mission import Backend  # noqa: E402

from warehouse_agent.mission import Mission  # noqa: E402
from warehouse_agent.ros_messages import status_message, status_snapshot  # noqa: E402
from warehouse_agent.world import World  # noqa: E402


class StatusMessageTests(unittest.TestCase):
    def test_round_trip_through_a_mission(self):
        world = World()
        backend = Backend(world)
        with tempfile.TemporaryDirectory() as directory:
            mission = Mission(world, backend, Path(directory)/"episode.jsonl")
            snapshots = [mission.snapshot()]
            mission.submit("deliver P1 and P2", "test")
            snapshots.append(mission.snapshot())  # A request is pending.
            mission.resolve("R1", {"operation": "create", "parcels": ["P1", "P2"], "message": ""})
            mission.tick()
            snapshots.append(mission.snapshot())  # One active goal and planned stops.
            backend.arrive()
            mission.tick()
            snapshots.append(mission.snapshot())  # Cargo onboard.
            mission.command("pause")
            snapshots.append(mission.snapshot())  # Held by the operator, cancel requested.
            backend.result = "cancelled"
            mission.tick()
            mission.command("resume")
            mission.tick()
            for _ in range(3):
                backend.arrive()
                mission.tick()
            snapshots.append(mission.snapshot())  # Finished.
            mission.close()
            snapshots.append(mission.snapshot())
        self.assertTrue(snapshots[-2]["finished"])
        for index, snapshot in enumerate(snapshots):
            with self.subTest(snapshot=index):
                msg = status_message(snapshot)
                self.assertEqual(status_snapshot(msg), snapshot)
                self.assertEqual(msg.paused, bool(msg.hold))

    def test_constants_match_the_state_machines(self):
        from warehouse_interfaces.msg import Hold, Parcel

        from warehouse_agent.state import Disposition, HoldKind, Physical
        self.assertEqual({Parcel.STATE_AWAITING_PICKUP, Parcel.STATE_ONBOARD, Parcel.STATE_DELIVERED},
                         {str(s) for s in Physical})
        self.assertEqual({Parcel.DISPOSITION_INACTIVE, Parcel.DISPOSITION_ACTIVE, Parcel.DISPOSITION_CANCELLED,
                          Parcel.DISPOSITION_DEFERRED}, {str(d) for d in Disposition})
        self.assertEqual({Hold.KIND_OPERATOR, Hold.KIND_CLARIFICATION, Hold.KIND_REQUEST_FAILED, Hold.KIND_NO_ROUTE},
                         {str(k) for k in HoldKind})


if __name__ == "__main__":
    unittest.main()
