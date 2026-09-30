"""Property-based tests: random operators, model replies and Nav2 results against the real supervisor.

After every step, the published snapshot must pass check_snapshot(); at the end of every run, the journal must pass
the independent checker. Requires Hypothesis (pip install hypothesis); skipped when it is not installed.
"""

import json
import math
import tempfile
import unittest
from pathlib import Path

try:
    from hypothesis import HealthCheck, given, settings
    from hypothesis import strategies as st
    from hypothesis.stateful import RuleBasedStateMachine, initialize, invariant, precondition, rule
except ImportError:  # pragma: no cover - exercised only without Hypothesis
    raise unittest.SkipTest("Hypothesis is not installed; run: pip install hypothesis") from None

from test_mission import Backend

from warehouse_agent.mission import Mission
from warehouse_agent.verify import check_snapshot, verify_file
from warehouse_agent.world import World

WORLD = World()
A_STAR_CACHE = {}  # Costs depend only on the shared, unchanged world, so runs can share them.
PARCELS = list(WORLD.config["parcels"])
OPERATIONS = ["create", "prioritize", "onboard_first", "cancel", "clarify", "status", "pause", "resume"]
x0, y0, x1, y1 = WORLD.bounds

valid_ids = st.lists(st.sampled_from(PARCELS), min_size=1, max_size=len(PARCELS), unique=True)


def intent(operation, parcels=()):
    return {"operation": operation, "parcels": list(parcels), "message": ""}


# Mostly well-formed requests (which may still be illegal in the current state), plus arbitrary model output.
intents = st.one_of(
    valid_ids.map(lambda ids: intent("create", ids)),
    st.sampled_from(PARCELS).map(lambda pid: intent("prioritize", [pid])),
    valid_ids.map(lambda ids: intent("cancel", ids)),
    st.sampled_from(["onboard_first", "status", "pause", "resume", "clarify"]).map(intent),
    st.fixed_dictionaries({"operation": st.sampled_from(OPERATIONS),
                           "parcels": st.lists(st.sampled_from([*PARCELS, "P9"]), max_size=3),
                           "message": st.just("")}),
)
poses = st.tuples(st.floats(x0+0.3, x1-0.3), st.floats(y0+0.3, y1-0.3), st.floats(-math.pi, math.pi))
# Mostly exact arrivals, sometimes small errors that may or may not pass the transfer checks.
arrival_errors = st.one_of(st.just((0.0, 0.0, 0.0)),
                           st.tuples(st.floats(-0.4, 0.4), st.floats(-0.4, 0.4), st.floats(-0.6, 0.6)))


class Harness:
    """A Mission with a backend double, driven through the same request API the CLI uses."""

    def __init__(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.journal = Path(self.tmp.name)/"episode.jsonl"
        self.backend = Backend(WORLD)
        self.backend.publish_status = lambda state: None
        self.mission = Mission(WORLD, self.backend, self.journal)
        self.mission.planner.cache = A_STAR_CACHE
        self.intent = None

    @property
    def pending(self):
        return self.mission.pending_request

    def start_request(self, intent):
        self.intent = intent
        return self.mission.submit(json.dumps(intent), "property-test")

    def finish_request(self, error=None):
        request_id = self.pending["id"]
        if error:
            return self.mission.resolve(request_id, error=error)
        return self.mission.resolve(request_id, self.intent, model="property-test")

    def direct(self, operation):
        self.mission.command(operation)

    def close(self):
        if self.mission.active:
            self.backend.result = "cancelled"
        self.mission.close()
        report = verify_file(self.journal)
        self.tmp.cleanup()
        return report


class SupervisorMachine(RuleBasedStateMachine):
    def __init__(self):
        super().__init__()
        self.h = Harness()

    @initialize(parcels=valid_ids)
    def mission_created(self, parcels):
        self.h.mission.apply(intent("create", parcels))

    @precondition(lambda self: self.h.pending is None)
    @rule(intent=intents)
    def operator_sends_request(self, intent):
        self.h.start_request(intent)

    @precondition(lambda self: self.h.pending is None and self.h.mission.active is not None)
    @rule()
    def operator_cancels_parcel_in_transit(self):
        """The race that matters most: cancel the parcel the robot is driving to."""
        self.h.start_request(intent("cancel", [self.h.mission.active.parcel]))

    @precondition(lambda self: self.h.mission.active is not None)
    @rule()
    def operator_pauses_mid_navigation(self):
        self.h.direct("pause")

    @precondition(lambda self: self.h.pending is not None)
    @rule()
    def model_replies(self):
        self.h.finish_request()

    @precondition(lambda self: self.h.pending is not None)
    @rule(error=st.sampled_from(["model timed out", "model unreachable", "unsupported intent"]))
    def model_fails(self, error):
        self.h.finish_request(error)

    @rule(operation=st.sampled_from(["pause", "resume", "status"]))
    def operator_direct_command(self, operation):
        self.h.direct(operation)

    @precondition(lambda self: self.h.backend.target is not None)
    @rule()
    def nav2_arrives_exactly(self):
        self.h.backend.arrive()

    @precondition(lambda self: self.h.mission.active is not None)
    @rule(error=st.tuples(st.floats(-0.4, 0.4), st.floats(-0.4, 0.4), st.floats(-0.6, 0.6)))
    def nav2_succeeds_off_target(self, error):
        """Nav2 reports success, but the measured pose is off: the supervisor must verify before moving cargo."""
        x, y, yaw = self.h.backend.target
        self.h.backend.pose = [x+error[0], y+error[1], yaw+error[2]]
        self.h.backend.result = "succeeded"
        self.h.mission.tick()

    @precondition(lambda self: self.h.mission.active is not None)
    @rule()
    def robot_completes_current_stop(self):
        """Time passes: the robot reaches its goal and the supervisor reacts. Lets runs reach deep states."""
        self.h.backend.arrive()
        self.h.mission.tick()

    @precondition(lambda self: self.h.backend.target is not None)
    @rule(outcome=st.sampled_from(["succeeded", "failed", "cancelled", "rejected"]), error=arrival_errors)
    def nav2_reports(self, outcome, error):
        backend = self.h.backend
        if outcome == "succeeded":
            x, y, yaw = backend.target
            backend.pose = [x+error[0], y+error[1], yaw+error[2]]
        backend.result = outcome

    @rule(pose=poses)
    def robot_drifts(self, pose):
        """Localization can place the robot anywhere, including inside a rack's planning margin."""
        self.h.backend.pose = list(pose)

    @rule()
    def supervisor_ticks(self):
        self.h.mission.tick()

    @invariant()
    def snapshot_obeys_the_rules(self):
        findings = check_snapshot(self.h.mission.snapshot())
        assert not findings, findings

    def teardown(self):
        report = self.h.close()
        assert report.ok, report.violations
        unexpected = [w for w in report.warnings if w.code != "UNRESOLVED_REQUEST"]
        assert not unexpected, unexpected


SupervisorMachine.TestCase.settings = settings(max_examples=400, stateful_step_count=80, deadline=None,
                                               suppress_health_check=[HealthCheck.too_slow])
TestSupervisorMachine = SupervisorMachine.TestCase


class LivenessTests(unittest.TestCase):
    @settings(max_examples=150, deadline=None, suppress_health_check=[HealthCheck.too_slow])
    @given(parcels=st.lists(st.sampled_from(PARCELS), min_size=1, unique=True),
           priority=st.one_of(st.none(), st.sampled_from(PARCELS)),
           failures=st.lists(st.booleans(), min_size=40, max_size=40))
    def test_every_parcel_ends_delivered_or_deferred(self, parcels, priority, failures):
        """With no operator interference, the mission always terminates; each parcel is delivered or deferred."""
        h = Harness()
        h.mission.apply({"operation": "create", "parcels": parcels})
        if priority in parcels:
            h.mission.apply({"operation": "prioritize", "parcels": [priority]})
        attempts = iter(failures)
        for _ in range(2*len(parcels)*2+2):
            h.mission.tick()
            if not h.mission.active:
                break
            if next(attempts):
                h.backend.result = "failed"
            else:
                h.backend.arrive()
        self.assertTrue(h.mission.finished)
        self.assertTrue(h.mission.completed_reported)
        for pid in parcels:
            p = h.mission.parcels[pid]
            self.assertTrue(p.state == "delivered" or p.disposition == "deferred", (pid, p))
        report = h.close()
        self.assertTrue(report.ok, report.violations)


if __name__ == "__main__":
    unittest.main()
