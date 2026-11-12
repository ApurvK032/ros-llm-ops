"""A camera in the simulated warehouse, for footage of a scenario run without a display.

The simulator renders on the GPU with no window. This module drops a camera sensor into the running world, the
same way a fault drops a box, and saves what it sees twice a second with the time of each frame, so the frames
line up with the journal and the true-pose recording. Where a point in the warehouse lands in a frame is plain
pinhole geometry (`project`), which is what lets a figure script label the robot and the stations afterwards.

Recorder, inside the ROS environment:  python3 -B -m warehouse_agent.camera <run directory>
"""

import math
import subprocess
import sys
import time
from pathlib import Path

# The whole 18 x 14 m floor from above the south wall.
OVERVIEW = {"position": (0.0, -19.5, 15.5), "look_at": (0.0, -0.6, 0.0), "hfov": 0.98, "width": 1600,
            "height": 900}
NAME = "overview_camera"
PERIOD = 0.5    # seconds between saved frames


def aim(view):
    """Yaw and downward pitch that point the camera's axis from its position at the point it looks at."""
    dx, dy, dz = (view["look_at"][i] - view["position"][i] for i in range(3))
    return math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))


def project(point, view=OVERVIEW):
    """Pixel (column, row) at which a point in the world appears, or None when it is behind the camera."""
    yaw, pitch = aim(view)
    x, y, z = (point[i] - view["position"][i] for i in range(3))
    # Into the camera's frame: undo the yaw about the vertical, then the pitch about the camera's left axis.
    ahead, left = x*math.cos(yaw) + y*math.sin(yaw), -x*math.sin(yaw) + y*math.cos(yaw)
    forward, up = ahead*math.cos(pitch) - z*math.sin(pitch), ahead*math.sin(pitch) + z*math.cos(pitch)
    if forward <= 0.05:
        return None
    focal = (view["width"]/2)/math.tan(view["hfov"]/2)
    return view["width"]/2 - focal*left/forward, view["height"]/2 - focal*up/forward


def spawn(view=OVERVIEW, name=NAME, rate=10):
    """Add the camera to the running world. Returns the image topic it publishes on."""
    from .faults import FaultError, _quote, ros_shell, world_name
    yaw, pitch = aim(view)
    cy, sy, cp, sp = math.cos(yaw/2), math.sin(yaw/2), math.cos(pitch/2), math.sin(pitch/2)
    x, y, z = view["position"]
    sdf = (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static><link name='link'>"
           f"<sensor name='{name}' type='camera'><always_on>1</always_on><update_rate>{rate}</update_rate>"
           f"<topic>{name}/image</topic><camera><horizontal_fov>{view['hfov']}</horizontal_fov><image>"
           f"<width>{view['width']}</width><height>{view['height']}</height></image><clip><near>0.1</near>"
           "<far>150</far></clip></camera></sensor></link></model></sdf>")
    request = (f'sdf: "{sdf}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: '
               f'{{x: {-sy*sp}, y: {cy*sp}, z: {sy*cp}, w: {cy*cp}}}}}, name: "{name}"')
    done = ros_shell(f"gz service -s /world/{world_name()}/create --reqtype gz.msgs.EntityFactory "
                     f"--reptype gz.msgs.Boolean --timeout 5000 --req {_quote(request)}")
    if "data: true" not in done.stdout:
        raise FaultError(f"Gazebo did not create the camera: {(done.stdout + done.stderr).strip()}")
    return f"/{name}/image"


def main():
    if len(sys.argv) != 2:
        print(__doc__.strip().splitlines()[-1].strip(), file=sys.stderr)
        return 2
    import numpy as np
    import rclpy
    from PIL import Image
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image as ImageMessage
    frames = Path(sys.argv[1]) / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    topic = f"/{NAME}/image"
    bridge = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                               f"{topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node("warehouse_camera")
    last = [-math.inf]

    def save(message):
        now = time.monotonic()
        if now - last[0] < PERIOD:
            return
        last[0] = now
        pixels = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(message.height, message.step // 3, 3)
        # Named by the clock the journal uses, so a frame can be placed in the mission afterwards.
        Image.fromarray(pixels[:, :message.width]).save(frames / f"{now:.2f}.jpg", quality=93)

    node.create_subscription(ImageMessage, topic, save, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        bridge.terminate()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
