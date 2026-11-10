"""ROS-side helpers for fault injection, run as a short subprocess with ROS sourced.

Each command prints one JSON object describing exactly what it changed, including the value it replaced, so the
scenario result records the ground truth and a later recovery can be checked against it.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python


def spin_until(node, done, timeout):
    deadline = time.monotonic() + timeout
    while not done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    return done()


def set_parameter(node, target, name, value):
    """Set one parameter on another node and report the value it had before."""
    from rclpy.parameter_client import AsyncParameterClient
    client = AsyncParameterClient(node, target)
    if not client.wait_for_services(timeout_sec=5.0):
        raise RuntimeError(f"{target} has no parameter services; is the node running?")
    before = client.get_parameters([name])
    if not spin_until(node, before.done, 5):
        raise RuntimeError(f"{target} did not answer a parameter read")
    old = parameter_value_to_python(before.result().values[0])
    if old is None:
        raise RuntimeError(f"{target} has no parameter named {name}")
    change = client.set_parameters([Parameter(name, value=value)])
    if not spin_until(node, change.done, 5):
        raise RuntimeError(f"{target} did not answer the parameter change")
    result = change.result().results[0]
    if not result.successful:
        raise RuntimeError(f"{target} refused {name}={value!r}: {result.reason}")
    return {"node": target, "name": name, "old": list(old) if hasattr(old, "__iter__") and not isinstance(old, str)
            else old, "new": value}


def offset_pose(node, dx, dy, dyaw):
    """Tell AMCL the robot is somewhere it is not: its current estimate shifted by an offset in the map frame."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from tf2_ros import Buffer, TransformException, TransformListener
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    transform = None

    def have_pose():
        nonlocal transform
        try:
            transform = buffer.lookup_transform("map", "base_link", rclpy.time.Time()).transform
        except TransformException:
            return False
        return True

    if not spin_until(node, have_pose, 5):
        raise RuntimeError("No map to base_link transform; is localization running?")
    del listener
    x, y = transform.translation.x, transform.translation.y
    yaw = 2*math.atan2(transform.rotation.z, transform.rotation.w)
    wrong = [x + dx, y + dy, math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw))]
    publisher = node.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
    spin_until(node, lambda: publisher.get_subscription_count() > 0, 5)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.pose.position.x, message.pose.pose.position.y = wrong[0], wrong[1]
    message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(wrong[2]/2), math.cos(wrong[2]/2)
    # A confident (tight) covariance, so the filter commits to the wrong pose instead of spreading out.
    message.pose.covariance[0] = message.pose.covariance[7] = 0.04
    message.pose.covariance[35] = 0.04
    publisher.publish(message)
    spin_until(node, lambda: False, 0.5)
    return {"estimate_before": [round(v, 3) for v in (x, y, yaw)], "estimate_after": [round(v, 3) for v in wrong],
            "offset": [dx, dy, dyaw]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    change = sub.add_parser("param-set")
    change.add_argument("node")
    change.add_argument("name")
    change.add_argument("value", help="JSON value")
    shift = sub.add_parser("pose-offset")
    shift.add_argument("offset", type=float, nargs=3, metavar=("DX", "DY", "DYAW"))
    args = parser.parse_args()
    rclpy.init()
    node = Node("fault_tools", parameter_overrides=[Parameter("use_sim_time", value=True)])
    try:
        if args.command == "param-set":
            done = set_parameter(node, args.node, args.name, json.loads(args.value))
        else:
            done = offset_pose(node, *args.offset)
        print(json.dumps(done))
        return 0
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
