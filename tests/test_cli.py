"""Operator controls must remain authoritative while language is pending."""

import contextlib
import io
import json
from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from warehouse_agent.cli import run
from warehouse_agent.world import ROOT, World
from test_mission import Backend


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
    def execute(self, commands, intent, delay=0, pose=None, command=None, code=0):
        inbox = queue.Queue()
        for text in commands:
            inbox.put(text)
        backend = Backend(World())
        if pose:
            backend.pose = list(pose)
        backend.publish_status = lambda state: None
        future = Future(intent, delay)
        with tempfile.TemporaryDirectory() as directory:
            journal = Path(directory)/"episode.jsonl"
            args = SimpleNamespace(config=ROOT/"config/warehouse.json", reset_localization=False,
                                   journal=journal, model="test_model", endpoint="http://unused",
                                   command=command, timeout=20)
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

    def test_unplannable_pose_exits_promptly_in_command_mode(self):
        backend, events, output = self.execute([], {"operation": "create", "parcels": ["P1", "P2", "P3"]},
                                               pose=[-5.0, 0.0, 0.0], command="Deliver all parcels", code=1)
        self.assertEqual(backend.dispatched, 0)
        self.assertEqual([e["type"] for e in events].count("planning_failed"), 1)
        self.assertIn("Mission held", output)
        self.assertNotIn("timed out", output)
