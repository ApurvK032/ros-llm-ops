"""Frame the actual Gazebo warehouse and ask its GUI to save a screenshot."""

import json
import math
import subprocess
import time

from .world import ROOT

CAMERA_POSITION = tuple(json.loads((ROOT/"config/warehouse.json").read_text())["overview_camera"])


def frame_camera():
    """Frame once when the GUI is ready; later camera movement stays user-owned."""
    x, y, z = CAMERA_POSITION
    yaw = math.atan2(-y, -x)
    pitch = math.atan2(z, math.hypot(x, y))
    qx = -math.sin(pitch/2)*math.sin(yaw/2)
    qy = math.sin(pitch/2)*math.cos(yaw/2)
    qz = math.cos(pitch/2)*math.sin(yaw/2)
    qw = math.cos(pitch/2)*math.cos(yaw/2)
    x, y, z = CAMERA_POSITION
    position = f"position {{x: {x} y: {y} z: {z}}}"
    orientation = f"orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}"
    text = "pose {"+position+" "+orientation+"}"
    try:
        result = subprocess.run(["gz", "service", "-s", "/gui/move_to/pose", "--reqtype", "gz.msgs.GUICamera",
                                 "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", text],
                                capture_output=True, text=True, timeout=3)
        return result.returncode == 0 and "data: true" in result.stdout
    except (OSError, subprocess.TimeoutExpired):
        return False


def main():
    directory = ROOT / "artifacts/captures"
    directory.mkdir(parents=True, exist_ok=True)
    if not frame_camera():
        raise RuntimeError("Gazebo GUI camera is unavailable")
    time.sleep(2)
    existing = set(directory.glob("*.png"))
    subprocess.run(["gz", "service", "-s", "/gui/screenshot", "--reqtype", "gz.msgs.StringMsg",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", f'data: "{directory}"'],
                   check=True, timeout=10)
    for _ in range(100):
        new = set(directory.glob("*.png"))-existing
        if new:
            print(sorted(new)[-1])
            return
        time.sleep(0.1)
    raise RuntimeError("Gazebo acknowledged capture but did not save a frame; ensure its GUI is rendering")


if __name__ == "__main__":
    main()
