"""Fault injectors for scenario runs.

Each injector acts on the simulator or on ROS processes from outside the supervisor, the way a real fault would,
and returns what it actually did so the result file holds the ground truth. Parameter checks are separate pure
functions so scenario files are validated without ROS.
"""

import json
import os
import signal
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Nav2 servers started as separate processes when the simulation runs with use_composition:=False.
CRASHABLE_NODES = ("controller_server", "planner_server", "bt_navigator", "behavior_server", "smoother_server",
                   "velocity_smoother", "collision_monitor", "amcl", "map_server")


class FaultError(RuntimeError):
    """The fault could not be injected; the run is invalid rather than a mission failure."""


def ros_env():
    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "42")
    env.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    return env


def ros_shell(command, timeout=15):
    """Run a command with ROS 2 Jazzy sourced, on the simulation's ROS domain."""
    return subprocess.run(["bash", "-c", f"source /opt/ros/jazzy/setup.bash && {command}"],
                          env=ros_env(), capture_output=True, text=True, timeout=timeout)


def world_name():
    """The running Gazebo world, read from its services so it cannot drift from the generated SDF."""
    listing = ros_shell("gz service -l").stdout
    names = {line.split("/")[2] for line in listing.splitlines()
             if line.startswith("/world/") and line.endswith("/create")}
    if len(names) != 1:
        raise FaultError(f"Expected one running Gazebo world, found {sorted(names) or 'none'}")
    return names.pop()


def check_blocked_path(params, waypoints):
    """Validate blocked_path parameters and return the obstacle's (x, y) and size."""
    at, position = params.get("at"), params.get("position")
    if (at is None) == (position is None):
        raise ValueError("blocked_path needs exactly one of 'at' (a waypoint name) or 'position' ([x, y])")
    if at is not None:
        if at not in waypoints:
            raise ValueError(f"blocked_path: unknown waypoint {at!r}")
        position = waypoints[at][:2]
    if (not isinstance(position, list | tuple) or len(position) != 2
            or not all(isinstance(v, int | float) for v in position)):
        raise ValueError("blocked_path: position must be [x, y]")
    size = params.get("size", [0.6, 0.6, 0.8])
    if not isinstance(size, list | tuple) or len(size) != 3 or not all(isinstance(v, int | float) and v > 0
                                                                        for v in size):
        raise ValueError("blocked_path: size must be three positive numbers [x, y, z]")
    return [float(position[0]), float(position[1])], [float(v) for v in size]


def check_node_crash(params, waypoints=None):
    node = params.get("node")
    if node not in CRASHABLE_NODES:
        raise ValueError(f"node_crash: node must be one of {', '.join(CRASHABLE_NODES)}")
    return node


def blocked_path(params, waypoints):
    """Drop a static box into the world: something left in an aisle or on a station's approach pose."""
    (x, y), (sx, sy, sz) = check_blocked_path(params, waypoints)
    name = params.get("name", "fault_obstacle")
    box = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    # The lidar is rendered (gpu_lidar), so the box needs a visual to be seen and a collision to block.
    sdf = (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static><link name='body'>"
           f"<collision name='collision'>{box}</collision><visual name='visual'>{box}<material>"
           "<ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material></visual>"
           "</link></model></sdf>")
    request = f'sdf: "{sdf}", pose: {{position: {{x: {x}, y: {y}, z: {sz/2}}}}}, name: "{name}"'
    world = world_name()
    done = ros_shell(f"gz service -s /world/{world}/create --reqtype gz.msgs.EntityFactory "
                     f"--reptype gz.msgs.Boolean --timeout 5000 --req {_quote(request)}")
    if "data: true" not in done.stdout:
        raise FaultError(f"Gazebo did not create the obstacle: {(done.stdout + done.stderr).strip()}")
    return {"entity": name, "position": [x, y], "size": [sx, sy, sz], "at": params.get("at")}


def node_crash(params, waypoints=None):
    """Kill one Nav2 server process outright (SIGKILL), as a segfault or out-of-memory kill would."""
    node = check_node_crash(params)
    pids = _pids_of(node)
    if len(pids) != 1:
        raise FaultError(f"Expected one {node} process, found {len(pids)}; "
                         "run the simulation with use_composition:=False")
    os.kill(pids[0], signal.SIGKILL)
    return {"node": node, "pid": pids[0], "signal": "SIGKILL"}


def _pids_of(node):
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = (entry / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        if argv and Path(argv[0].decode(errors="replace")).name == node and b"/opt/ros/" in argv[0]:
            found.append(int(entry.name))
    return found


def _quote(text):
    return "'" + text.replace("'", "'\"'\"'") + "'"


def fault_tool(*arguments):
    """Run a fault_tools command inside the ROS environment and return the JSON it prints."""
    quoted = " ".join(_quote(str(a)) for a in arguments)
    done = subprocess.run(
        ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash && "
         f"exec python3 -B -m warehouse_agent.fault_tools {quoted}"],
        cwd=ROOT, env=ros_env(), capture_output=True, text=True, timeout=30)
    if done.returncode != 0:
        raise FaultError((done.stderr or done.stdout).strip().splitlines()[-1] if (done.stderr or done.stdout).strip()
                         else "fault_tools failed")
    return json.loads(done.stdout.strip().splitlines()[-1])


def check_sensor_dropout(params, waypoints=None):
    if params.get("sensor", "lidar") != "lidar":
        raise ValueError("sensor_dropout: the only sensor that can be interrupted is 'lidar'")
    mode = params.get("mode", "drop")
    if mode not in ("drop", "freeze"):
        raise ValueError("sensor_dropout: mode must be 'drop' (no data) or 'freeze' (stale data)")
    return mode


def sensor_dropout(params, waypoints=None):
    """Stop or freeze the lidar scans Nav2 receives. Needs the simulation started with SCAN_RELAY=1."""
    done = fault_tool("param-set", "/scan_relay", "mode", json.dumps(check_sensor_dropout(params)))
    return {"sensor": "lidar", "mode": done["new"], "previous_mode": done["old"]}


def check_localization_jump(params, waypoints=None):
    offset = params.get("offset")
    if (not isinstance(offset, list | tuple) or len(offset) != 3
            or not all(isinstance(v, int | float) for v in offset) or not any(offset)):
        raise ValueError("localization_jump: offset must be a non-zero [dx, dy, dyaw] in metres and radians")
    return [float(v) for v in offset]


def localization_jump(params, waypoints=None):
    """Shift AMCL's pose estimate away from where the robot really is."""
    return fault_tool("pose-offset", *check_localization_jump(params))


def check_bad_parameter(params, waypoints=None):
    node, name = params.get("node"), params.get("name")
    if not isinstance(node, str) or not node.startswith("/") or not isinstance(name, str) or not name:
        raise ValueError("bad_parameter: needs 'node' (for example /velocity_smoother) and 'name'")
    if "value" not in params or isinstance(params["value"], dict) or params["value"] is None:
        raise ValueError("bad_parameter: needs a 'value' (number, string, boolean or list)")
    return node, name, params["value"]


def bad_parameter(params, waypoints=None):
    """Change one parameter on a running node to a harmful value, as a bad configuration push would."""
    node, name, value = check_bad_parameter(params)
    return fault_tool("param-set", node, name, json.dumps(value))


# fault type -> (parameter check, injector)
FAULTS = {"blocked_path": (check_blocked_path, blocked_path), "node_crash": (check_node_crash, node_crash),
          "sensor_dropout": (check_sensor_dropout, sensor_dropout),
          "localization_jump": (check_localization_jump, localization_jump),
          "bad_parameter": (check_bad_parameter, bad_parameter)}
