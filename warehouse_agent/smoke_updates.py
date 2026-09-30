"""Explicit integration check against a running Gazebo/Nav2/Ollama instance."""

import time
from concurrent.futures import ThreadPoolExecutor
from datetime import UTC, datetime

from .cli import print_event
from .language import LocalModel
from .mission import Mission
from .ros_backend import RosBackend
from .world import ROOT, World


def main():
    world = World()
    backend = RosBackend(world)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S.%fZ")
    mission = Mission(world, backend, ROOT / "artifacts/episodes" / f"updates-{stamp}.jsonl", print_event)
    model = LocalModel()
    executor = ThreadPoolExecutor(max_workers=1)

    def until(condition, timeout=240):
        deadline = time.monotonic()+timeout
        while not condition():
            if time.monotonic() > deadline:
                raise RuntimeError("Live update check timed out")
            mission.tick()
            backend.publish_status(mission.snapshot())
            time.sleep(0.05)

    def say(text, expected):
        mission.language_pending = True
        mission.record("language_requested", text=text, model=model.model)
        future = executor.submit(model.interpret, text, mission.snapshot())
        until(future.done, timeout=125)
        intent, metrics = future.result()
        mission.record("language_interpreted", intent=intent, **metrics)
        print(f"{text} => {intent}", flush=True)
        if intent["operation"] != expected:
            raise AssertionError(f"Expected {expected}, got {intent}")
        mission.apply(intent)
        mission.language_pending = False

    try:
        say("Deliver all three parcels", "create")
        mission.tick()
        first = mission.active.parcel
        # Pause during a real active action and wait for terminal cancellation.
        mission.apply({"operation": "pause", "parcels": []})
        until(lambda: mission.active is None, timeout=20)
        assert mission.parcels[first].state == "awaiting_pickup"
        mission.record("check_passed", check="pause_acknowledged_without_pickup")
        mission.apply({"operation": "resume", "parcels": []})
        mission.tick()
        say(f"Cancel order {first}", "cancel")
        until(lambda: mission.active is None, timeout=20)
        assert mission.parcels[first].state == "awaiting_pickup"
        assert mission.parcels[first].disposition == "cancelled"
        mission.record("check_passed", check="model_cancel_during_navigation", parcel=first)
        preferred = next(pid for pid in mission.parcels if pid != first)
        say(f"Make {preferred} the highest priority", "prioritize")
        say("What are you carrying?", "status")
        until(lambda: any(p.state == "onboard" for p in mission.parcels.values()))
        say("Deliver the parcels already onboard first", "onboard_first")
        until(lambda: mission.completed_reported)
        assert all(p.state == "delivered" for pid, p in mission.parcels.items() if pid != first)
        mission.record("check_passed", check="live_updates_complete", state=mission.snapshot())
        print("LIVE UPDATE CHECK PASSED", flush=True)
    finally:
        mission.close()
        executor.shutdown(wait=False, cancel_futures=True)


if __name__ == "__main__":
    main()
