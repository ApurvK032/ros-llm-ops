"""Lidar pass-through that scenario runs can interrupt.

With SCAN_RELAY=1 the simulation starts this node and Nav2 reads /scan_nav instead of /scan. In mode "pass" every
scan is forwarded unchanged; "drop" forwards nothing (a dead sensor or driver); "freeze" keeps re-sending the last
scan with fresh timestamps (a driver stuck on stale data). The mode is an ordinary ROS parameter.
"""

import contextlib
import copy

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

MODES = ("pass", "drop", "freeze")
OUTPUT_TOPIC = "scan_nav"


class ScanRelay(Node):
    def __init__(self):
        super().__init__("scan_relay", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.declare_parameter("mode", "pass")
        self.add_on_set_parameters_callback(self.check)
        self.last = None
        # Reliable output serves both the reliable and the best-effort subscribers inside Nav2.
        self.publisher = self.create_publisher(LaserScan, OUTPUT_TOPIC, 10)
        self.create_subscription(LaserScan, "scan", self.forward, qos_profile_sensor_data)

    def check(self, parameters):
        for parameter in parameters:
            if parameter.name == "mode" and parameter.value not in MODES:
                return SetParametersResult(successful=False, reason=f"mode must be one of {', '.join(MODES)}")
        return SetParametersResult(successful=True)

    def forward(self, scan):
        mode = self.get_parameter("mode").value
        if mode == "drop":
            return
        if mode == "freeze" and self.last is not None:
            stale = copy.copy(self.last)
            stale.header = copy.copy(self.last.header)
            stale.header.stamp = scan.header.stamp
            self.publisher.publish(stale)
            return
        self.last = scan
        self.publisher.publish(scan)


def main():
    rclpy.init()
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A second interrupt can arrive while shutting down; there is nothing left to clean up by then.
        with contextlib.suppress(KeyboardInterrupt):
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
