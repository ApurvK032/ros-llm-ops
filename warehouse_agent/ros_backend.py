"""ROS 2 NavigateToPose adapter with asynchronous cancellation and TF poses."""

import math
import time


class RosBackend:
    name = "ros2_nav2_gazebo"

    def __init__(self, world, initialize_pose=False):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from rclpy.parameter import Parameter
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_msgs.action import NavigateToPose
        from tf2_ros import Buffer, TransformListener
        from std_msgs.msg import String
        from lifecycle_msgs.srv import GetState
        self.rclpy, self.action_type = rclpy, NavigateToPose
        rclpy.init()
        self.node = Node("warehouse_agent", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
        self.lifecycle = self.node.create_client(GetState, "bt_navigator/get_state")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        self.pose = list(world.waypoints["HOME"])
        self.pose_received = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.node.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.on_pose, qos)
        self.initial = self.node.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.status_pub = self.node.create_publisher(String, "warehouse/status", qos)
        self.timeout = world.config["navigation_timeout"]
        self.goal_future = self.result_future = self.goal_handle = None
        self.cancel_pending = self.cancel_sent = False
        self.terminal = None
        deadline, next_initial = time.monotonic()+120, 0
        state_future = None
        navigation_active = False
        print("Waiting for Nav2 and localization…", flush=True)
        while time.monotonic() < deadline:
            self.spin()
            if state_future is not None and state_future.done():
                navigation_active = state_future.result().current_state.id == 3
                state_future = None
            if state_future is None and self.lifecycle.service_is_ready():
                state_future = self.lifecycle.call_async(GetState.Request())
            if navigation_active and self.pose_received and self.client.server_is_ready():
                print("Nav2 and localization ready.", flush=True)
                return
            if initialize_pose and not self.pose_received and time.monotonic() >= next_initial:
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.pose.pose = self.pose_message(world.waypoints["HOME"]).pose
                msg.pose.covariance[0] = msg.pose.covariance[7] = 0.04
                msg.pose.covariance[35] = 0.04
                self.initial.publish(msg)
                next_initial = time.monotonic()+1
            time.sleep(0.05)
        self.close()
        raise RuntimeError("Nav2/localization not ready after 120 seconds; start scripts/run_sim.sh first")

    def on_pose(self, message):
        p = message.pose.pose
        self.pose = [p.position.x, p.position.y, 2*math.atan2(p.orientation.z, p.orientation.w)]
        self.pose_received = True

    def pose_message(self, pose):
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(pose[0]), float(pose[1])
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        return msg

    def spin(self):
        # Drain high-rate TF/clock callbacks without blocking operator controls.
        for _ in range(30):
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
        try:
            from tf2_ros import TransformException
            t = self.tf.lookup_transform("map", "base_link", self.rclpy.time.Time()).transform
            self.pose = [t.translation.x, t.translation.y, 2*math.atan2(t.rotation.z, t.rotation.w)]
        except TransformException:
            pass

    def navigate(self, pose):
        if self.goal_future and self.terminal is None:
            raise RuntimeError("A navigation goal is already active")
        self.terminal = None
        self.cancel_pending = self.cancel_sent = False
        self.goal_handle = self.result_future = None
        self.started = time.monotonic()
        goal = self.action_type.Goal()
        goal.pose = self.pose_message(pose)
        self.goal_future = self.client.send_goal_async(goal)

    def poll(self):
        if self.terminal:
            return self.terminal
        if not self.goal_future:
            return None
        if not self.goal_handle and self.goal_future.done():
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted:
                self.terminal = "rejected"
                return self.terminal
            self.result_future = self.goal_handle.get_result_async()
        if time.monotonic()-self.started > self.timeout:
            self.cancel_pending = True
        if self.cancel_pending and self.goal_handle and not self.cancel_sent:
            self.cancel_future = self.goal_handle.cancel_goal_async()
            self.cancel_sent = True
            self.cancel_started = time.monotonic()
        if self.result_future and self.result_future.done():
            result = self.result_future.result()
            self.terminal = {4: "succeeded", 5: "cancelled", 6: "failed"}.get(result.status, "failed")
            return self.terminal
        if self.cancel_sent and time.monotonic()-self.cancel_started > 20:
            raise RuntimeError("Nav2 cancellation did not reach a terminal result; dispatch stopped")
        if not self.goal_handle and time.monotonic()-self.started > 20:
            raise RuntimeError("Nav2 did not acknowledge the goal; dispatch stopped")
        return None

    def cancel(self):
        self.cancel_pending = True

    def publish_status(self, state):
        import json
        from std_msgs.msg import String
        self.status_pub.publish(String(data=json.dumps(state)))

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()
