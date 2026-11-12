"""Live evidence: what a recording would give afterwards, built from the topics while the mission runs.

Every two seconds it writes <run>/live_evidence.json, in the same shape evidence.py extracts from a finished bag,
so the diagnostic checks work on a running mission unchanged. It also writes <run>/processes.json once: how each
Nav2 server and the lidar relay was started, so a dead or hung one can be started again in exactly the same way.

Run inside the ROS environment:  python3 -B -m warehouse_agent.monitor <run directory>
"""

import json
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .evidence import EvidenceBuilder, mission_start, process_events
from .faults import CRASHABLE_NODES

PERIOD = 2.0
RELAY = "scan_relay"


def process_table():
    """Running Nav2 servers and the lidar relay: name -> pid, command line and working directory."""
    table = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = [part.decode(errors="replace") for part in (entry / "cmdline").read_bytes().split(b"\0") if part]
            cwd = os.readlink(entry / "cwd")
        except OSError:
            continue
        if not argv:
            continue
        name = Path(argv[0]).name
        if name in CRASHABLE_NODES and argv[0].startswith("/opt/ros/"):
            table[name] = {"pid": int(entry.name), "argv": argv, "cwd": cwd}
        elif name == "lifecycle_manager" and argv[0].startswith("/opt/ros/"):
            # Two of them run; the node name each was given tells them apart.
            named = [part.split(":=", 1)[1] for part in argv if part.startswith("__node:=")]
            if named:
                table[named[0]] = {"pid": int(entry.name), "argv": argv, "cwd": cwd}
        elif "warehouse_agent.sensor_relay" in argv:
            table[RELAY] = {"pid": int(entry.name), "argv": argv, "cwd": cwd}
    return table


def write_atomically(path, data):
    scratch = path.with_suffix(path.suffix + ".tmp")
    scratch.write_text(json.dumps(data, allow_nan=False), encoding="utf-8")
    os.replace(scratch, path)


class Monitor(Node):
    def __init__(self, run):
        super().__init__("warehouse_monitor", parameter_overrides=[Parameter("use_sim_time", value=True)])
        from bond.msg import Status
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav2_msgs.msg import BehaviorTreeLog
        from nav_msgs.msg import OccupancyGrid, Odometry
        from nav_msgs.msg import Path as PlannedPath
        from rcl_interfaces.msg import Log, ParameterEvent
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        self.run = Path(run)
        self.status = None      # the supervisor's latest published snapshot
        self.builder = EvidenceBuilder()
        self.origin = None
        self.processes = {}
        events = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        stream = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, kind, qos in (
                ("/rosout", Log, events), ("/bond", Status, events), ("/behavior_tree_log", BehaviorTreeLog, events),
                ("/parameter_events", ParameterEvent, events), ("/initialpose", PoseWithCovarianceStamped, events),
                ("/plan", PlannedPath, events), ("/odom", Odometry, stream), ("/cmd_vel", Twist, stream),
                ("/tf", TFMessage, stream), ("/amcl_pose", PoseWithCovarianceStamped, latched),
                ("/scan", LaserScan, stream), ("/scan_nav", LaserScan, stream),
                ("/global_costmap/costmap", OccupancyGrid, latched)):
            self.create_subscription(kind, topic, lambda message, topic=topic: self.receive(topic, message), qos)
        self.create_subscription(String, "/warehouse/status", self.on_status, latched)
        # Wall-clock timer: the snapshot must keep coming even if simulated time stops.
        self.started = time.time()
        self.next_write = self.started + PERIOD

    def on_status(self, message):
        try:
            self.status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def receive(self, topic, message):
        self.builder.add(topic, message, time.time())

    def write(self):
        now = time.time()
        if now < self.next_write:
            return
        self.next_write = now + PERIOD
        if len(self.processes) < 2 or now - self.started < 20:
            # Nav2 comes up over several seconds; keep the fullest table seen, never drop a process that died.
            for name, entry in process_table().items():
                self.processes.setdefault(name, entry)
            write_atomically(self.run / "processes.json", self.processes)
        self.origin = self.origin or mission_start(self.run)
        origin = self.origin or self.builder.first or now
        snapshot = self.builder.snapshot(origin, now, process_events(self.run, origin))
        snapshot["meta"]["live"] = True
        snapshot["meta"]["mission_started"] = self.origin is not None
        snapshot["mission_status"] = self.status
        write_atomically(self.run / "live_evidence.json", snapshot)


def main():
    if len(sys.argv) != 2:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2
    rclpy.init()
    node = Monitor(sys.argv[1])
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            node.write()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
