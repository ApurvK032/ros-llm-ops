"""The robot's true pose, read from the simulator, to check what the mission claims.

The supervisor accepts a pickup or a delivery when the robot's estimated pose passes the arrival check. After a
localization fault that estimate is exactly what cannot be trusted. So scenario runs also record where Gazebo
says the robot is, and every transfer is checked again against that with the same arrival check.

The recorder needs the simulator's `gz` tool on the path and no ROS node:

    python3 -B -m warehouse_agent.ground_truth <run directory>
"""

import bisect
import json
import math
import subprocess
import sys
import time
from pathlib import Path

ROBOT = "turtlebot3_waffle"
PERIOD = 0.1        # seconds between recorded poses; the simulator publishes far more often
MATCH_WINDOW = 1.0  # seconds: how far from a journal event a recorded pose may be and still count


def parse_pose(line, model=ROBOT):
    """(x, y, yaw) of the model in one JSON message from Gazebo's pose stream, or None if it is not in it."""
    try:
        message = json.loads(line)
    except json.JSONDecodeError:
        return None
    for pose in message.get("pose", []) if isinstance(message, dict) else []:
        if pose.get("name") == model:
            # Protobuf's JSON form leaves out every field that is zero.
            p, q = pose.get("position", {}), pose.get("orientation", {})
            x, y, z, w = (float(q.get(key, 0.0)) for key in "xyzw")
            yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            return float(p.get("x", 0.0)), float(p.get("y", 0.0)), yaw
    return None


def read_truth(path):
    """Recorded rows [monotonic, x, y, yaw], oldest first; a partly written last line is ignored."""
    rows = []
    try:
        lines = Path(path).read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return rows
    for line in lines:
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            break
        if isinstance(row, list) and len(row) == 4:
            rows.append(row)
    return rows


def pose_at(rows, monotonic, window=MATCH_WINDOW):
    """The recorded pose nearest in time to a journal event, or None when nothing was recorded close to it."""
    index = bisect.bisect_left(rows, [monotonic])
    near = [row for row in rows[max(0, index-1):index+1] if abs(row[0] - monotonic) <= window]
    if not near:
        return None
    return tuple(min(near, key=lambda row: abs(row[0] - monotonic))[1:])


def check_transfers(events, rows, world):
    """Check every pickup and delivery in a journal against the true pose at that moment.

    A parcel counts as verified when the journal says it was delivered and the robot really was at the pickup
    station and at the drop station, by the same arrival check the supervisor applied to its estimate.
    """
    transfers, passed, unknown = [], {}, set()
    for event in events:
        if event.get("type") not in ("cargo_pickup", "cargo_drop"):
            continue
        entry = {"parcel": event["parcel"], "kind": event["type"].removeprefix("cargo_"),
                 "waypoint": event["waypoint"], "estimated_position_error": round(event["position_error"], 3)}
        pose = pose_at(rows, event["monotonic"])
        if pose is None:
            entry["verified"] = None
            unknown.add(event["parcel"])
        else:
            report = world.arrival(pose, event["waypoint"])
            entry.update(true_pose=[round(v, 3) for v in pose],
                         position_error=round(report["position_error"], 3),
                         heading_error=round(report["heading_error"], 3),
                         facing_error=round(report["facing_error"], 3),
                         parcel_clearance=round(report["parcel_clearance"], 3),
                         estimate_off_by=round(math.dist(pose[:2], event["pose"][:2]), 3),
                         verified=bool(report["accepted"]))
            passed.setdefault(event["parcel"], []).append(entry["verified"])
        transfers.append(entry)
    state = next((e["state"] for e in reversed(events) if isinstance(e.get("state"), dict)), {})
    delivered = sorted(p for p, v in state.get("parcels", {}).items() if v["state"] == "delivered")
    checked = [t for t in transfers if t["verified"] is not None]
    return {
        "samples": len(rows), "transfers": transfers,
        "delivered_verified": [p for p in delivered if p not in unknown and len(passed.get(p, [])) == 2
                               and all(passed[p])],
        "delivered_not_checked": [p for p in delivered if p in unknown or len(passed.get(p, [])) != 2],
        "worst_position_error": max((t["position_error"] for t in checked), default=None),
        "worst_estimate_error": max((t["estimate_off_by"] for t in checked), default=None),
    }


def main():
    if len(sys.argv) != 2:
        print(__doc__.strip().splitlines()[-1].strip(), file=sys.stderr)
        return 2
    from .faults import FaultError, world_name
    run = Path(sys.argv[1])
    try:
        topic = f"/world/{world_name()}/dynamic_pose/info"
    except FaultError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    reader = subprocess.Popen(["gz", "topic", "-e", "--json-output", "-t", topic], stdout=subprocess.PIPE,
                              text=True)
    last = -math.inf
    try:
        with open(run / "truth.jsonl", "a", encoding="utf-8") as out:
            for line in reader.stdout:
                now = time.monotonic()
                if now - last < PERIOD:
                    continue
                pose = parse_pose(line)
                if pose:
                    out.write(json.dumps([round(now, 3), *(round(v, 4) for v in pose)]) + "\n")
                    out.flush()
                    last = now
    except KeyboardInterrupt:
        pass
    finally:
        reader.terminate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
