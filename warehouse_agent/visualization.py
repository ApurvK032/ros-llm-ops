"""Independent ROS visualization node; it has no robot command publisher."""

import argparse
import json
import os
import signal
import time
from dataclasses import asdict
from pathlib import Path

from .marker_transport import GazeboMarkers, ros_message
from .visual_state import initial_state, project
from .world import World


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--panel", action="store_true")
    parser.add_argument("--no-gazebo", action="store_true")
    parser.add_argument("--record-dir", type=Path)
    args = parser.parse_args()
    record_dir = args.record_dir or (Path(os.environ["WAREHOUSE_VISUAL_RECORD_DIR"]) if os.environ.get("WAREHOUSE_VISUAL_RECORD_DIR") else None)
    rclpy.init()
    node = Node("warehouse_visualization", parameter_overrides=[Parameter("use_sim_time", value=True)])
    world = World()
    state = initial_state(world)
    received_at = None
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(MarkerArray, "warehouse/markers", qos)

    def receive(message):
        nonlocal state, received_at
        candidate = json.loads(message.data)
        # The supervisor is authoritative; no intent parsing or replanning here.
        if set(candidate["parcels"]) != set(world.config["parcels"]):
            node.get_logger().error("Ignoring a status snapshot for a different warehouse")
            return
        state, received_at = candidate, time.monotonic()

    node.create_subscription(String, "warehouse/status", receive, qos)  # The node keeps it alive.
    gazebo = None if args.no_gazebo else GazeboMarkers()
    app = panel = None
    if args.panel:
        from PyQt5.QtWidgets import QApplication

        from .mission_panel import MissionPanel
        app = QApplication([])
        panel = MissionPanel(world)
        panel.show()
    running = True

    def stop(*_):
        nonlocal running
        running = False

    # Keep ROS and Qt cleanup in this loop rather than asynchronous handlers.
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    known = set()
    next_frame = 0
    previous_record = None
    record_number = 0
    try:
        while running and rclpy.ok():
            for _ in range(15):
                rclpy.spin_once(node, timeout_sec=0.0)
            if app:
                app.processEvents()
            now = time.monotonic()
            if now >= next_frame:
                connection = "Waiting for the agent"
                display = state
                if received_at is not None:
                    age = now-received_at
                    connection = "Live mission state" if age < 3 else "Agent disconnected · last known state"
                    if state.get("closed"):
                        connection = "Session ended · final recorded state"
                    if age >= 3 or state.get("closed"):
                        display = {**state, "active": None, "disconnected": age >= 3}
                visuals = project(world, display)
                current = {v.key for v in visuals}
                publisher.publish(ros_message(visuals, known-current, node.get_clock().now().to_msg()))
                known |= current
                if gazebo:
                    gazebo.submit(visuals)
                if panel:
                    panel.update_state(display, connection, gazebo.available if gazebo else None)
                if record_dir and received_at is not None:
                    signature = (tuple((pid, p["state"], p["disposition"]) for pid, p in state["parcels"].items()),
                                 str(state["active"]), state["paused"], state.get("closed"))
                    if signature != previous_record:
                        record_dir.mkdir(parents=True, exist_ok=True)
                        record_number += 1
                        prefix = record_dir / f"{record_number:03d}"
                        prefix.with_suffix(".json").write_text(json.dumps({"state": state, "visuals": [asdict(v) for v in visuals], "gazebo_connected": gazebo.available if gazebo else False}, indent=2)+"\n")
                        if panel:
                            app.processEvents()
                            panel.grab().save(str(prefix.with_suffix(".png")))
                        previous_record = signature
                next_frame = now+0.1
            time.sleep(0.01)
    finally:
        if gazebo:
            gazebo.close()
        if panel:
            panel.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
