"""Turn a scenario run's recording into compact evidence for diagnosis.

Reading an MCAP bag needs ROS; reasoning about a run should not. This module does the ROS-dependent part once and
writes evidence.json: downsampled motion and pose, message rates, warnings, Nav2 heartbeats, behaviour-tree
failures, parameter changes, pose resets, plans, and obstacles that appeared in the global costmap. Everything
is timed in seconds from the start of the mission, so it lines up with the journal.

Run inside the ROS environment:  python3 -B -m warehouse_agent.evidence <run directory>
"""

import json
import math
import re
import sys
from datetime import datetime
from pathlib import Path

BUCKET = 5.0  # seconds, for message rates and lidar summaries
RATE_TOPICS = ("/scan", "/scan_nav", "/odom", "/cmd_vel", "/amcl_pose", "/plan", "/tf")
LEVELS = {30: "WARN", 40: "ERROR", 50: "FATAL"}
LETHAL = 100       # nav2 costmap -> OccupancyGrid scales a lethal obstacle to 100
OBSTACLE_BIN = 0.5  # metres; new lethal cells are grouped on this grid
# rcl_interfaces/ParameterType value -> the ParameterValue field that holds it
PARAMETER_FIELDS = (None, "bool_value", "integer_value", "double_value", "string_value", "byte_array_value",
                    "bool_array_value", "integer_array_value", "double_array_value", "string_array_value")


def yaw_of(q):
    return 2*math.atan2(q.z, q.w)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    """Pose b expressed in a's parent frame, for planar (x, y, yaw) transforms."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    return (ax + bx*math.cos(ayaw) - by*math.sin(ayaw), ay + bx*math.sin(ayaw) + by*math.cos(ayaw),
            wrap(ayaw + byaw))


def mission_start(run):
    """Wall-clock time the mission was accepted, from the journal; None if it never started."""
    journal = Path(run) / "journal.jsonl"
    if not journal.exists():
        return None
    for line in journal.read_text(encoding="utf-8").splitlines():
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            break
        if event.get("type") == "instruction_applied":
            return datetime.fromisoformat(event["time"]).timestamp()
    return None


def process_events(run, origin):
    """Processes the launch system reported as dead, timed by the nearest earlier log line.

    Only the part of the log written before the runner began tearing the simulation down is read, when the
    result file says where that was: teardown kills processes too.
    """
    log = Path(run) / "simulation.log"
    events, last = [], None
    if not log.exists():
        return events
    limit = None
    result = Path(run) / "result.json"
    if result.exists():
        limit = json.loads(result.read_text(encoding="utf-8")).get("run", {}).get("simulation_log_bytes")
    stamp = re.compile(r"\[(\d{10}\.\d+)\]")
    died = re.compile(r"\[ERROR\] \[([\w.-]+)\]: process has died \[pid (\d+), exit code (-?\d+)")
    for line in log.read_bytes()[:limit].decode("utf-8", errors="replace").splitlines():
        found = stamp.search(line)
        if found:
            last = float(found.group(1))
        match = died.search(line)
        if match and last is not None and last >= origin - 1:
            name = re.sub(r"-\d+$", "", match.group(1))
            events.append({"t": round(last - origin, 1), "process": name, "exit_code": int(match.group(3))})
    return events


def extract(run):
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    run = Path(run)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    origin = mission_start(run)
    counts, first_stamp, last_stamp = {}, None, None
    rates = {topic: {} for topic in RATE_TOPICS}
    logs, bonds, bt_failures, bt_counts = [], {}, [], {}
    parameters, pose_resets, plans = [], [], []
    motion = {}       # second -> sums for averaging
    odom_pose, map_odom, map_odom_series, jumps = None, None, [], []
    pose_series, amcl = [], []
    scans = {"/scan": {}, "/scan_nav": {}}
    baseline, grid, obstacle_bins, snapshots = None, None, {}, 0
    last_plan_second = None

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        wall = stamp / 1e9
        first_stamp = first_stamp or wall
        last_stamp = wall
        origin = origin or first_stamp
        t = wall - origin
        counts[topic] = counts.get(topic, 0) + 1
        bucket = int(t // BUCKET)
        if topic in rates:
            rates[topic][bucket] = rates[topic].get(bucket, 0) + 1
        if topic not in ("/rosout", "/bond", "/behavior_tree_log", "/parameter_events", "/initialpose", "/plan",
                         "/odom", "/cmd_vel", "/tf", "/amcl_pose", "/scan", "/scan_nav",
                         "/global_costmap/costmap"):
            continue
        message = deserialize_message(data, types[topic])
        if topic == "/rosout":
            if message.level >= 30:
                text = " ".join(message.msg.split())
                if logs and logs[-1]["node"] == message.name and logs[-1]["text"] == text:
                    logs[-1]["count"] += 1
                    logs[-1]["t_last"] = round(t, 1)
                else:
                    logs.append({"t": round(t, 1), "t_last": round(t, 1), "level": LEVELS.get(message.level, "WARN"),
                                 "node": message.name, "text": text, "count": 1})
        elif topic == "/bond":
            # Both ends of a bond publish under the node's name: the node itself and the lifecycle manager.
            # Keep them apart, or the manager's side hides the moment the node went quiet.
            bond = bonds.setdefault(message.id, {"first": round(t, 1), "instances": {}})
            bond["active"] = bool(message.active)
            bond["instances"][message.instance_id] = t
        elif topic == "/behavior_tree_log":
            for entry in message.event_log:
                if entry.current_status == "FAILURE":
                    bt_counts[entry.node_name] = bt_counts.get(entry.node_name, 0) + 1
                    if len(bt_failures) < 400:
                        bt_failures.append({"t": round(t, 1), "node": entry.node_name})
                elif entry.current_status == "RUNNING" and entry.node_name in (
                        "Spin", "BackUp", "Wait", "DriveOnHeading", "RecoveryActions"):
                    key = "started:" + entry.node_name
                    bt_counts[key] = bt_counts.get(key, 0) + 1
        elif topic == "/parameter_events":
            changed = [{"name": p.name, "value": _parameter_value(p.value)} for p in message.changed_parameters]
            if changed:
                parameters.append({"t": round(t, 1), "node": message.node, "changed": changed})
        elif topic == "/initialpose":
            p = message.pose.pose
            pose_resets.append({"t": round(t, 1), "x": round(p.position.x, 2), "y": round(p.position.y, 2),
                                "yaw": round(yaw_of(p.orientation), 2)})
        elif topic == "/plan":
            if last_plan_second != int(t) and message.poses:
                last_plan_second = int(t)
                points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
                length = sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))
                plans.append([round(t, 1), round(length, 2), round(points[-1][0], 2), round(points[-1][1], 2)])
        elif topic == "/odom":
            second = motion.setdefault(int(t), {"v": 0.0, "w": 0.0, "n": 0, "cv": 0.0, "cw": 0.0, "cn": 0})
            second["v"] += message.twist.twist.linear.x
            second["w"] += message.twist.twist.angular.z
            second["n"] += 1
        elif topic == "/cmd_vel":
            second = motion.setdefault(int(t), {"v": 0.0, "w": 0.0, "n": 0, "cv": 0.0, "cw": 0.0, "cn": 0})
            second["cv"] += message.linear.x
            second["cw"] += message.angular.z
            second["cn"] += 1
        elif topic == "/tf":
            for transform in message.transforms:
                pose = (transform.transform.translation.x, transform.transform.translation.y,
                        yaw_of(transform.transform.rotation))
                if transform.header.frame_id == "odom" and transform.child_frame_id in ("base_footprint",
                                                                                         "base_link"):
                    odom_pose = pose
                elif transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                    if map_odom is not None:
                        shift = math.dist(pose[:2], map_odom[:2])
                        turn = abs(wrap(pose[2] - map_odom[2]))
                        if shift > 0.3 or turn > 0.2:
                            # A correction this large between two updates is a discontinuity, not drift.
                            jumps.append({"t": round(t, 1), "shift_m": round(shift, 2), "turn_rad": round(turn, 2)})
                    if not map_odom_series or t - map_odom_series[-1][0] >= 1.0:
                        map_odom_series.append([round(t, 1)] + [round(v, 3) for v in pose])
                    map_odom = pose
            if odom_pose and map_odom and (not pose_series or t - pose_series[-1][0] >= 1.0):
                pose_series.append([round(t, 1)] + [round(v, 2) for v in compose(map_odom, odom_pose)])
        elif topic == "/amcl_pose":
            p, c = message.pose.pose, message.pose.covariance
            amcl.append([round(t, 1), round(p.position.x, 2), round(p.position.y, 2), round(yaw_of(p.orientation), 2),
                         round(math.sqrt(max(c[0], 0) + max(c[7], 0)), 3)])
        elif topic in scans:
            entry = scans[topic].setdefault(bucket, {"n": 0, "valid": 0.0, "min": math.inf})
            valid = [r for r in message.ranges if math.isfinite(r) and message.range_min < r < message.range_max]
            entry["n"] += 1
            entry["valid"] += len(valid) / max(len(message.ranges), 1)
            entry["min"] = min(entry["min"], min(valid, default=math.inf))
        elif topic == "/global_costmap/costmap":
            snapshots += 1
            cells = np.frombuffer(bytes(message.data), dtype=np.int8).reshape(message.info.height,
                                                                              message.info.width)
            if baseline is None:
                baseline = cells.copy()
                grid = (message.info.origin.position.x, message.info.origin.position.y, message.info.resolution)
                continue
            if cells.shape != baseline.shape:
                continue
            rows, cols = np.nonzero((cells == LETHAL) & (baseline < 50))
            if len(rows) == 0:
                continue
            xs = grid[0] + (cols + 0.5)*grid[2]
            ys = grid[1] + (rows + 0.5)*grid[2]
            keys, numbers = np.unique(np.stack([np.floor(xs/OBSTACLE_BIN), np.floor(ys/OBSTACLE_BIN)], axis=1),
                                      axis=0, return_counts=True)
            for (bx, by), number in zip(keys.astype(int).tolist(), numbers.tolist(), strict=True):
                if number < 4:
                    continue
                inside = (np.floor(xs/OBSTACLE_BIN) == bx) & (np.floor(ys/OBSTACLE_BIN) == by)
                box = [float(xs[inside].min()), float(xs[inside].max()), float(ys[inside].min()),
                       float(ys[inside].max())]
                seen = obstacle_bins.setdefault((bx, by), {"first": t, "snapshots": 0, "cells": 0, "box": box})
                seen["last"] = t
                seen["snapshots"] += 1
                if number >= seen["cells"]:
                    seen["cells"], seen["box"] = number, box

    end = (last_stamp or 0) - (origin or 0)
    return {
        "meta": {"mission_start_wall": origin, "recording_start": round((first_stamp or 0) - (origin or 0), 1),
                 "recording_end": round(end, 1), "message_counts": counts, "bucket_s": BUCKET,
                 "costmap_snapshots": snapshots},
        "rates": {topic: _buckets(values, end) for topic, values in rates.items() if counts.get(topic)},
        "logs": logs,
        "bonds": {name: _bond(b, end) for name, b in sorted(bonds.items())},
        "behaviour_tree": {"failure_counts": dict(sorted(bt_counts.items())), "failures": bt_failures},
        "parameter_changes": parameters,
        "pose_resets": pose_resets,
        # Without a teardown marker (older runs), at least drop deaths after the recording stopped.
        "processes_died": [d for d in process_events(run, origin or 0) if d["t"] <= end],
        "motion": [[second, round(m["v"]/m["n"], 3) if m["n"] else None, round(m["w"]/m["n"], 3) if m["n"] else None,
                    round(m["cv"]/m["cn"], 3) if m["cn"] else None, round(m["cw"]/m["cn"], 3) if m["cn"] else None]
                   for second, m in sorted(motion.items())],
        "pose": pose_series,
        "map_to_odom": map_odom_series,
        "localization_jumps": jumps,
        "amcl": amcl,
        "plans": plans,
        "lidar": {topic: [[round(b*BUCKET, 1), e["n"], round(e["valid"]/e["n"], 2),
                           None if math.isinf(e["min"]) else round(e["min"], 2)] for b, e in sorted(values.items())]
                  for topic, values in scans.items() if values},
        "new_obstacles": _clusters(obstacle_bins),
    }


def _bond(bond, end):
    """When a node's heartbeat was last heard: the earlier of the bond's two ends to fall silent."""
    last = min(bond["instances"].values())
    return {"first": bond["first"], "last": round(last, 1), "silent_for_s": round(end - last, 1),
            "last_active_flag": bond["active"], "restarts": max(0, len(bond["instances"]) - 2)}


def _parameter_value(value):
    field = PARAMETER_FIELDS[value.type] if 0 < value.type < len(PARAMETER_FIELDS) else None
    if field is None:
        return None
    got = getattr(value, field)
    return got if isinstance(got, bool | int | float | str) else list(got)


def _buckets(values, end):
    """Messages per bucket from the first bucket to the end of the recording, zeros included."""
    if not values:
        return {"start": 0, "counts": []}
    first, last = min(values), max(max(values), int(end // BUCKET))
    return {"start": round(first*BUCKET, 1), "counts": [values.get(b, 0) for b in range(first, last+1)]}


def _clusters(bins):
    """Merge neighbouring bins of new lethal cells into obstacles, with when they were first and last seen."""
    remaining, clusters = dict(bins), []
    while remaining:
        key, seed = remaining.popitem()
        members, frontier = {key: seed}, [key]
        while frontier:
            bx, by = frontier.pop()
            for near in [(bx+dx, by+dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]:
                if near in remaining:
                    members[near] = remaining.pop(near)
                    frontier.append(near)
        weight = sum(m["cells"] for m in members.values())
        clusters.append({
            "x": round(sum((bx+0.5)*OBSTACLE_BIN*m["cells"] for (bx, _), m in members.items())/weight, 2),
            "y": round(sum((by+0.5)*OBSTACLE_BIN*m["cells"] for (_, by), m in members.items())/weight, 2),
            "extent_m": round(max(_span(members, 0, 1), _span(members, 2, 3)), 1),
            "first_seen": round(min(m["first"] for m in members.values()), 1),
            "last_seen": round(max(m["last"] for m in members.values()), 1),
            "snapshots": max(m["snapshots"] for m in members.values())})
    return sorted(clusters, key=lambda c: c["first_seen"])


def _span(members, low, high):
    """Width of a cluster along one axis: from its cells where known, else from its bins."""
    boxes = [m["box"] for m in members.values() if "box" in m]
    if len(boxes) == len(members):
        return max(box[high] for box in boxes) - min(box[low] for box in boxes) + 0.05
    axis = low // 2
    return OBSTACLE_BIN*(max(key[axis] for key in members) - min(key[axis] for key in members) + 1)


def main():
    if len(sys.argv) != 2:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2
    run = Path(sys.argv[1])
    evidence = extract(run)
    (run / "evidence.json").write_text(json.dumps(evidence, allow_nan=False) + "\n", encoding="utf-8")
    print(f"{run / 'evidence.json'}: {sum(evidence['meta']['message_counts'].values())} messages, "
          f"{len(evidence['logs'])} warnings, {len(evidence['new_obstacles'])} new obstacles")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
