"""Interactive local-model supervisor and repeatable ROS delivery demo."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import sys
import threading
import time

from .language import LocalModel
from .mission import Mission
from .world import ROOT, World


def print_event(event):
    kind = event["type"]
    if kind == "navigation_started":
        s = event["stop"]
        print(f"→ {s['waypoint']} · {s['kind']} {s['parcel']}", flush=True)
    elif kind.startswith("cargo_"):
        print(f"✓ {event['parcel']} {event['state']} at {event['waypoint']}", flush=True)
    elif kind in {"mission_finished", "parcel_deferred", "clarification_required", "navigation_retry"}:
        print(json.dumps(event, indent=2), flush=True)


def run(args):
    from .ros_backend import RosBackend
    world = World(args.config)
    backend = RosBackend(world, initialize_pose=args.reset_localization)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    journal = args.journal or ROOT / "artifacts/episodes" / f"{stamp}.jsonl"
    mission = Mission(world, backend, journal, print_event)
    model = LocalModel(args.model, args.endpoint)
    inbox = queue.Queue()
    executor = ThreadPoolExecutor(max_workers=1)
    pending = None
    pending_superseded = False
    started = time.monotonic()
    demo_accepted = False
    print(f"Journal: {journal}", flush=True)
    print("Commands: /status /pause /resume /quit. Otherwise type a warehouse instruction.", flush=True)

    def read_input():
        for line in sys.stdin:
            inbox.put(line.strip())
        if not args.command:
            inbox.put("/quit")

    if args.command:
        inbox.put(args.command)
    else:
        threading.Thread(target=read_input, daemon=True).start()
    try:
        while True:
            try:
                text = inbox.get_nowait()
            except queue.Empty:
                text = None
            if text == "/quit":
                break
            if text:
                if text in {"/status", "/pause", "/resume"}:
                    if text == "/pause" and pending:
                        pending_superseded = True
                    print(json.dumps(mission.apply({"operation": text[1:], "parcels": []}), indent=2), flush=True)
                elif pending:
                    print("Still interpreting the previous request. /pause and /status remain available.", flush=True)
                else:
                    mission.language_pending = True
                    mission.record("language_requested", text=text, model=args.model)
                    pending = executor.submit(model.interpret, text, mission.snapshot())
                    pending_superseded = False
                    print("Interpreting locally…", flush=True)
            if pending and pending.done():
                try:
                    if pending_superseded:
                        mission.record("request_superseded", reason="Direct pause took precedence over the pending model request")
                        print("Pending request discarded after /pause.", flush=True)
                        continue
                    intent, metrics = pending.result()
                    mission.record("language_interpreted", intent=intent, **metrics)
                    print("Request: " + json.dumps({"operation": intent["operation"], "parcels": intent["parcels"]}), flush=True)
                    state = mission.apply(intent)
                    if intent["operation"] == "status":
                        print(json.dumps(state, indent=2), flush=True)
                    demo_accepted = intent["operation"] == "create"
                    if args.command and not demo_accepted:
                        print("The command did not start a delivery mission.", flush=True)
                        return 2
                except Exception as exc:
                    mission.paused = True
                    mission.cancel_motion()
                    mission.record("request_failed", error=str(exc))
                    print(f"Request failed; mission paused: {exc}", flush=True)
                    if args.command:
                        return 2
                finally:
                    pending = None
                    mission.language_pending = False
            mission.tick()
            backend.publish_status(mission.snapshot())
            if args.command and demo_accepted and mission.completed_reported:
                return 0 if all(p["disposition"] != "deferred" for p in mission.parcels.values()) else 1
            if args.command and time.monotonic()-started > args.timeout:
                print("Demo timed out; cancelling navigation.", flush=True)
                return 1
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Stopping…", flush=True)
    finally:
        mission.close()
        executor.shutdown(wait=False, cancel_futures=True)
    return 0


def add_arguments(parser):
    parser.add_argument("--config", type=Path, default=ROOT / "config/warehouse.json")
    parser.add_argument("--model", default="qwen3.5:4b")
    parser.add_argument("--endpoint", default="http://127.0.0.1:11434")
    parser.add_argument("--journal", type=Path)
    parser.add_argument("--command", help="Interpret this text and exit when its delivery mission finishes")
    parser.add_argument("--timeout", type=float, default=600)
    parser.add_argument("--reset-localization", action="store_true", help="Explicitly reset localization to HOME; use only with the robot at HOME")
