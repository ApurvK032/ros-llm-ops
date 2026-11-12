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
from array import array
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


class EvidenceBuilder:
    """Accumulates evidence one message at a time, from a recording or from live topics.

    Everything is kept in wall-clock seconds; snapshot() converts to seconds from the start of the mission, which
    a live monitor only learns once the mission has begun.
    """

    TOPICS = ("/rosout", "/bond", "/behavior_tree_log", "/parameter_events", "/initialpose", "/plan", "/odom",
              "/cmd_vel", "/tf", "/amcl_pose", "/scan", "/scan_nav", "/global_costmap/costmap")

    def __init__(self):
        self.counts, self.first, self.last = {}, None, None
        self.stamps = {topic: [] for topic in RATE_TOPICS}
        self.logs, self.bonds, self.bt_failures, self.bt_counts = [], {}, [], {}
        self.parameters, self.pose_resets, self.plans = [], [], []
        self.motion = {}       # wall second -> sums for averaging
        self.odom_pose, self.map_odom, self.map_odom_series, self.jumps = None, None, [], []
        self.pose_series, self.amcl = [], []
        self.scans = {"/scan": [], "/scan_nav": []}
        self.last_ranges = {}
        self.baseline, self.grid, self.obstacle_bins, self.costmaps = None, None, {}, 0
        self.last_plan_second = None

    def count(self, topic, wall):
        """Note that a message arrived; enough for topics whose content is not examined."""
        self.first = self.first or wall
        self.last = wall
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if topic in self.stamps:
            self.stamps[topic].append(wall)

    def add(self, topic, message, wall):
        self.count(topic, wall)
        if topic == "/rosout":
            if message.level >= 30:
                text = " ".join(message.msg.split())
                if self.logs and self.logs[-1]["node"] == message.name and self.logs[-1]["text"] == text:
                    self.logs[-1]["count"] += 1
                    self.logs[-1]["t_last"] = wall
                else:
                    self.logs.append({"t": wall, "t_last": wall, "level": LEVELS.get(message.level, "WARN"),
                                      "node": message.name, "text": text, "count": 1})
        elif topic == "/bond":
            # Both ends of a bond publish under the node's name: the node itself and the lifecycle manager.
            # Keep them apart, or the manager's side hides the moment the node went quiet.
            bond = self.bonds.setdefault(message.id, {"first": wall, "instances": {}})
            bond["active"] = bool(message.active)
            bond["instances"][message.instance_id] = wall
        elif topic == "/behavior_tree_log":
            for entry in message.event_log:
                if entry.current_status == "FAILURE":
                    self.bt_counts[entry.node_name] = self.bt_counts.get(entry.node_name, 0) + 1
                    # GoalUpdated "fails" on every tick without a new goal; it is not a failure worth timing.
                    if entry.node_name != "GoalUpdated" and len(self.bt_failures) < 5000:
                        self.bt_failures.append({"t": wall, "node": entry.node_name})
                elif entry.current_status == "RUNNING" and entry.node_name in (
                        "Spin", "BackUp", "Wait", "DriveOnHeading", "RecoveryActions"):
                    key = "started:" + entry.node_name
                    self.bt_counts[key] = self.bt_counts.get(key, 0) + 1
        elif topic == "/parameter_events":
            changed = [{"name": p.name, "value": _parameter_value(p.value)} for p in message.changed_parameters]
            if changed:
                self.parameters.append({"t": wall, "node": message.node, "changed": changed})
        elif topic == "/initialpose":
            p = message.pose.pose
            self.pose_resets.append({"t": wall, "x": round(p.position.x, 2), "y": round(p.position.y, 2),
                                     "yaw": round(yaw_of(p.orientation), 2)})
        elif topic == "/plan":
            if self.last_plan_second != int(wall) and message.poses:
                self.last_plan_second = int(wall)
                points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
                length = sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))
                self.plans.append([wall, round(length, 2), round(points[-1][0], 2), round(points[-1][1], 2)])
        elif topic == "/odom":
            second = self.motion.setdefault(int(wall), {"v": 0.0, "w": 0.0, "n": 0, "cv": 0.0, "cw": 0.0, "cn": 0})
            second["v"] += message.twist.twist.linear.x
            second["w"] += message.twist.twist.angular.z
            second["n"] += 1
        elif topic == "/cmd_vel":
            second = self.motion.setdefault(int(wall), {"v": 0.0, "w": 0.0, "n": 0, "cv": 0.0, "cw": 0.0, "cn": 0})
            second["cv"] += message.linear.x
            second["cw"] += message.angular.z
            second["cn"] += 1
        elif topic == "/tf":
            for transform in message.transforms:
                pose = (transform.transform.translation.x, transform.transform.translation.y,
                        yaw_of(transform.transform.rotation))
                if transform.header.frame_id == "odom" and transform.child_frame_id in ("base_footprint",
                                                                                         "base_link"):
                    self.odom_pose = pose
                elif transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                    if self.map_odom is not None:
                        shift = math.dist(pose[:2], self.map_odom[:2])
                        turn = abs(wrap(pose[2] - self.map_odom[2]))
                        if shift > 0.3 or turn > 0.2:
                            # A correction this large between two updates is a discontinuity, not drift.
                            self.jumps.append({"t": wall, "shift_m": round(shift, 2), "turn_rad": round(turn, 2),
                                               "before": [round(v, 3) for v in self.map_odom]})
                    if not self.map_odom_series or wall - self.map_odom_series[-1][0] >= 1.0:
                        self.map_odom_series.append([wall] + [round(v, 3) for v in pose])
                    self.map_odom = pose
            if self.odom_pose and self.map_odom and (not self.pose_series or wall - self.pose_series[-1][0] >= 1.0):
                self.pose_series.append([wall] + [round(v, 2) for v in compose(self.map_odom, self.odom_pose)])
        elif topic == "/amcl_pose":
            p, c = message.pose.pose, message.pose.covariance
            self.amcl.append([wall, round(p.position.x, 2), round(p.position.y, 2), round(yaw_of(p.orientation), 2),
                              round(math.sqrt(max(c[0], 0) + max(c[7], 0)), 3)])
        elif topic in self.scans:
            valid = [r for r in message.ranges if math.isfinite(r) and message.range_min < r < message.range_max]
            # A lidar's ranges carry noise, so two scans in a row are never bit-for-bit the same unless the
            # second is the first sent again.
            ranges = message.ranges.tobytes() if hasattr(message.ranges, "tobytes") \
                else array("f", message.ranges).tobytes()
            repeated = self.last_ranges.get(topic) == ranges
            self.last_ranges[topic] = ranges
            self.scans[topic].append((wall, len(valid) / max(len(message.ranges), 1), min(valid, default=math.inf),
                                      repeated))
        elif topic == "/global_costmap/costmap":
            self.add_costmap(message, wall)

    def add_costmap(self, message, wall):
        import numpy as np
        self.costmaps += 1
        cells = np.frombuffer(bytes(message.data), dtype=np.int8).reshape(message.info.height, message.info.width)
        if self.baseline is None:
            self.baseline = cells.copy()
            self.grid = (message.info.origin.position.x, message.info.origin.position.y, message.info.resolution)
            return
        if cells.shape != self.baseline.shape:
            return
        rows, cols = np.nonzero((cells == LETHAL) & (self.baseline < 50))
        if len(rows) == 0:
            return
        xs = self.grid[0] + (cols + 0.5)*self.grid[2]
        ys = self.grid[1] + (rows + 0.5)*self.grid[2]
        keys, numbers = np.unique(np.stack([np.floor(xs/OBSTACLE_BIN), np.floor(ys/OBSTACLE_BIN)], axis=1),
                                  axis=0, return_counts=True)
        for (bx, by), number in zip(keys.astype(int).tolist(), numbers.tolist(), strict=True):
            if number < 4:
                continue
            inside = (np.floor(xs/OBSTACLE_BIN) == bx) & (np.floor(ys/OBSTACLE_BIN) == by)
            box = [float(xs[inside].min()), float(xs[inside].max()), float(ys[inside].min()),
                   float(ys[inside].max())]
            seen = self.obstacle_bins.setdefault((bx, by), {"first": wall, "snapshots": 0, "cells": 0, "box": box})
            seen["last"] = wall
            seen["snapshots"] += 1
            if number >= seen["cells"]:
                seen["cells"], seen["box"] = number, box

    def snapshot(self, origin=None, end=None, processes_died=()):
        """The evidence so far, timed in seconds from origin (the mission start; else the first message)."""
        origin = origin or self.first or 0.0
        end = (end or self.last or origin) - origin

        def rel(wall):
            return round(wall - origin, 1)

        def bucketed(stamps):
            values = {}
            for wall in stamps:
                bucket = int((wall - origin) // BUCKET)
                values[bucket] = values.get(bucket, 0) + 1
            return values

        lidar = {}
        for topic, scans in self.scans.items():
            buckets = {}
            for wall, valid, nearest, repeated in scans:
                entry = buckets.setdefault(int((wall - origin) // BUCKET),
                                           {"n": 0, "valid": 0.0, "min": math.inf, "repeated": 0})
                entry["n"] += 1
                entry["valid"] += valid
                entry["min"] = min(entry["min"], nearest)
                entry["repeated"] += repeated
            if buckets:
                # Per interval: start, scans, share of valid ranges, nearest return, share of repeated scans.
                lidar[topic] = [[round(b*BUCKET, 1), e["n"], round(e["valid"]/e["n"], 2),
                                 None if math.isinf(e["min"]) else round(e["min"], 2),
                                 round(e["repeated"]/e["n"], 2)]
                                for b, e in sorted(buckets.items())]
        bins = {key: {**value, "first": value["first"] - origin, "last": value["last"] - origin}
                for key, value in self.obstacle_bins.items()}
        return {
            "meta": {"mission_start_wall": origin, "recording_start": rel(self.first or origin),
                     "recording_end": round(end, 1), "message_counts": dict(self.counts), "bucket_s": BUCKET,
                     "costmap_snapshots": self.costmaps,
                     # Finer than the buckets: what is arriving right now, for a live agent checking a fix.
                     "recent_rate_hz": {topic: round(sum(1 for wall in reversed(stamps[-400:])
                                                         if wall - origin > end - 3.0) / 3.0, 1)
                                        for topic, stamps in self.stamps.items() if stamps}},
            "rates": {topic: _buckets(bucketed(stamps), end) for topic, stamps in self.stamps.items() if stamps},
            "logs": [{**log, "t": rel(log["t"]), "t_last": rel(log["t_last"])} for log in self.logs],
            "bonds": {name: _bond({"first": rel(b["first"]), "active": b["active"],
                                   "instances": {k: v - origin for k, v in b["instances"].items()}}, end)
                      for name, b in sorted(self.bonds.items())},
            "behaviour_tree": {"failure_counts": dict(sorted(self.bt_counts.items())),
                               "failures": [{**f, "t": rel(f["t"])} for f in self.bt_failures]},
            "parameter_changes": [{**c, "t": rel(c["t"])} for c in self.parameters],
            "pose_resets": [{**r, "t": rel(r["t"])} for r in self.pose_resets],
            "processes_died": [d for d in processes_died if d["t"] <= end],
            "motion": [[math.floor(second - origin), round(m["v"]/m["n"], 3) if m["n"] else None,
                        round(m["w"]/m["n"], 3) if m["n"] else None,
                        round(m["cv"]/m["cn"], 3) if m["cn"] else None,
                        round(m["cw"]/m["cn"], 3) if m["cn"] else None]
                       for second, m in sorted(self.motion.items())],
            "pose": [[rel(row[0]), *row[1:]] for row in self.pose_series],
            "map_to_odom": [[rel(row[0]), *row[1:]] for row in self.map_odom_series],
            "localization_jumps": [{**j, "t": rel(j["t"])} for j in self.jumps],
            "amcl": [[rel(row[0]), *row[1:]] for row in self.amcl],
            "plans": [[rel(row[0]), *row[1:]] for row in self.plans],
            "lidar": lidar,
            "new_obstacles": _clusters(bins),
        }


def extract(run):
    """Read a finished run's bag into evidence."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    run = Path(run)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    builder = EvidenceBuilder()
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in EvidenceBuilder.TOPICS:
            builder.add(topic, deserialize_message(data, types[topic]), stamp / 1e9)
        else:
            builder.count(topic, stamp / 1e9)
    origin = mission_start(run) or builder.first or 0.0
    # Without a teardown marker (older runs), snapshot() at least drops deaths after the recording stopped.
    return builder.snapshot(origin, builder.last, process_events(run, origin))


def _bond(bond, end):
    """When a node's heartbeat was last heard: the earlier of the bond's two ends to fall silent.

    A restarted node bonds again under new instance IDs, so only the two most recently heard ends count;
    the ends that died with the old process are history.
    """
    heard = sorted(bond["instances"].values(), reverse=True)[:2]
    last = min(heard)
    return {"first": bond["first"], "last": round(last, 1), "silent_for_s": round(end - last, 1),
            "last_active_flag": bond["active"], "restarts": math.ceil(max(0, len(bond["instances"]) - 2) / 2)}


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
