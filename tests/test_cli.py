"""Operator controls must remain authoritative while language is pending."""

import contextlib
import io
import json
import queue
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from test_mission import Backend

from warehouse_agent.cli import run
from warehouse_agent.world import ROOT, World


class Future:
    def __init__(self, intent, delay=0):
        self.intent, self.delay = intent, delay

    def done(self):
        if self.delay:
            self.delay -= 1
            return False
        return True

    def result(self):
        return self.intent, {"model": "test_model"}


class CliTests(unittest.TestCase):
    def execute(self, commands, intent, delay=0, pose=None, command=None, code=0, recovery=False,
                on_spin=None, **options):
        inbox = queue.Queue()
        for text in commands:
            inbox.put(text)
        backend = Backend(World())
        if pose:
            backend.pose = list(pose)
        backend.publish_status = lambda state: None
        backend.handler = None

        def serve_commands(handler):
            backend.handler = handler
            return True

        def spin():
            # A ROS service request arrives inside spin(); on_spin plays the caller.
            if on_spin and backend.handler:
                on_spin(backend)

        backend.serve_commands, backend.spin = serve_commands, spin
        future = Future(intent, delay)
        with tempfile.TemporaryDirectory() as directory:
            journal = Path(directory)/"episode.jsonl"
            args = SimpleNamespace(config=ROOT/"config/warehouse.json", reset_localization=False,
                                   journal=journal, model="test_model", endpoint="http://unused",
                                   command=command, timeout=20, recovery=recovery,
                                   on_fault=options.get("on_fault", "retry"),
                                   fault_hold_seconds=options.get("fault_hold_seconds", 90.0))
            output = io.StringIO()
            with patch("warehouse_agent.ros_backend.RosBackend", return_value=backend), \
                 patch("warehouse_agent.cli.queue.Queue", return_value=inbox), \
                 patch("warehouse_agent.cli.threading.Thread"), \
                 patch("warehouse_agent.cli.ThreadPoolExecutor") as executor, \
                 contextlib.redirect_stdout(output):
                executor.return_value.submit.return_value = future
                self.assertEqual(run(args), code)
            events = [json.loads(s) for s in journal.read_text().splitlines()]
        return backend, events, output.getvalue()

    def test_pause_supersedes_pending_model_create(self):
        backend, events, _ = self.execute(["Deliver all parcels", "/pause", "/quit"],
                                          {"operation": "create", "parcels": ["P1", "P2", "P3"]}, delay=1)
        self.assertEqual(backend.dispatched, 0)
        self.assertTrue(any(e["type"] == "request_superseded" for e in events))
        self.assertTrue(events[-1]["state"]["paused"])

    def test_natural_language_status_uses_application_cargo(self):
        _, _, output = self.execute(["What are you carrying?", "/quit"],
                                    {"operation": "status", "parcels": [], "message": "All parcels delivered!"})
        self.assertIn('"state": "awaiting_pickup"', output)
        self.assertNotIn("All parcels delivered!", output)

    def test_unplannable_pose_holds_instead_of_exiting(self):
        backend, events, output = self.execute(["Deliver all parcels", "/quit"],
                                               {"operation": "create", "parcels": ["P1", "P2", "P3"]},
                                               pose=[-5.0, 0.0, 0.0])
        self.assertEqual(backend.dispatched, 0)
        self.assertTrue(any(e["type"] == "planning_failed" for e in events))
        self.assertTrue(events[-1]["state"]["paused"])
        self.assertIn('"type": "planning_failed"', output)

    def test_direct_commands_from_the_service_are_applied_between_ticks_and_never_raise(self):
        replies = []

        def caller(backend):
            if backend.dispatched == 1 and not replies:
                replies.append(backend.handler("defer", ["P9"], ""))            # refused: unknown parcel
                replies.append(backend.handler("explode", [], ""))              # refused: not a command
                replies.append(backend.handler("defer", ["P1"], "PICK_A is blocked"))
            if backend.target is not None and backend.result is None:
                backend.result = "cancelled" if backend.cancelled else None
                if backend.result is None:
                    backend.arrive()

        backend, events, output = self.execute([], {"operation": "create", "parcels": ["P1", "P2"]},
                                               command="Deliver P1 and P2", code=1, recovery=True, on_spin=caller)
        self.assertEqual([accepted for accepted, _, _ in replies], [False, False, True])
        self.assertIn("Unknown parcel ID", replies[0][2])
        self.assertEqual(replies[2][1:], ("D3", "applied"))
        self.assertEqual([e["type"] for e in events].count("command_rejected"), 2)
        final = events[-1]["state"]["parcels"]
        self.assertEqual((final["P1"]["disposition"], final["P1"]["reason"]), ("deferred", "PICK_A is blocked"))
        self.assertEqual(final["P2"]["state"], "delivered")
        self.assertIn("Direct command D3: defer ['P1'] PICK_A is blocked", output)

    def test_without_recovery_no_command_service_is_offered(self):
        backend, _, _ = self.execute(["/quit"], {"operation": "status", "parcels": []})
        self.assertIsNone(backend.handler)

    def test_with_recovery_attached_an_unplannable_pose_waits_for_a_resume(self):
        def caller(backend):
            backend.pose = list(World().waypoints["HOME"])                      # recovery fixed the pose
            backend.handler("resume", [], "")
            if backend.target is not None and backend.result is None:
                backend.arrive()

        backend, events, output = self.execute([], {"operation": "create", "parcels": ["P1"]},
                                               pose=[-5.0, 0.0, 0.0], command="Deliver P1", recovery=True,
                                               on_spin=caller)
        self.assertNotIn("Mission held", output)
        self.assertEqual(events[-1]["state"]["parcels"]["P1"]["state"], "delivered")

    def test_unplannable_pose_exits_promptly_in_command_mode(self):
        backend, events, output = self.execute([], {"operation": "create", "parcels": ["P1", "P2", "P3"]},
                                               pose=[-5.0, 0.0, 0.0], command="Deliver all parcels", code=1)
        self.assertEqual(backend.dispatched, 0)
        self.assertEqual([e["type"] for e in events].count("planning_failed"), 1)
        self.assertIn("Mission held", output)
        self.assertNotIn("timed out", output)
