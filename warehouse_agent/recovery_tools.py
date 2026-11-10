"""ROS-side recovery actions, each run as a short subprocess with ROS sourced.

Every command prints one JSON object saying exactly what it did. None of them takes a free-form value from its
caller: a parameter is restored to the value in the deployed configuration, a pose is rebuilt from the last good
correction and odometry, a process is restarted with the command line it was first started with.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from .evidence import compose
from .fault_tools import planar, set_initial_pose, set_parameter, spin_until

ROOT = Path(__file__).resolve().parent.parent
PARAMETERS = ROOT / "sim/generated/nav2_params.yaml"
LOCALIZATION_NODES = {"amcl", "map_server"}
RELAY = "scan_relay"


def call(node, kind, name, request, timeout=10.0):
    client = node.create_client(kind, name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"service {name} is not available")
    future = client.call_async(request)
    if not spin_until(node, future.done, timeout):
        raise RuntimeError(f"service {name} did not answer within {timeout} s")
    return future.result()


def command(node, operation, parcels, reason):
    """Send a direct command to the mission supervisor."""
    from warehouse_interfaces.srv import OperatorCommand
    reply = call(node, OperatorCommand, "warehouse/command",
                 OperatorCommand.Request(command=operation, parcels=parcels, reason=reason))
    return {"operation": operation, "parcels": parcels, "accepted": reply.accepted,
            "request_id": reply.request_id, "message": reply.message}


def configured_value(target, name, path=PARAMETERS):
    """The value a parameter has in the deployed Nav2 configuration; KeyError if it is not configured there."""
    import yaml
    level = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    for part in target.strip("/").split("/"):
        level = level[part]
    level = level["ros__parameters"]
    for part in name.split("."):
        level = level[part]
    if isinstance(level, dict):
        raise KeyError(f"{name} is a group of parameters, not one value")
    return level


def restore_parameter(node, target, name):
    """Put a parameter back to its configured value."""
    try:
        value = configured_value(target, name)
    except KeyError as exc:
        raise RuntimeError(f"{target} {name} has no configured value to restore ({exc})") from None
    done = set_parameter(node, target, name, value)
    return {"node": target, "name": name, "was": done["old"], "restored_to": value}


def relocalize(node, before):
    """Undo a jump in the pose estimate: re-apply the map-to-odometry correction from before it.

    Odometry has no discontinuity, so the last good correction combined with where odometry says the robot is now
    gives where the robot really is, up to the drift since the jump. AMCL refines it from there.
    """
    from tf2_ros import Buffer, TransformException, TransformListener
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    odometry = None

    def have_odometry():
        nonlocal odometry
        try:
            odometry = planar(buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time()).transform)
        except TransformException:
            return False
        return True

    if not spin_until(node, have_odometry, 5):
        raise RuntimeError("No odom to base_footprint transform; cannot rebuild the pose")
    del listener
    pose = compose(tuple(before), odometry)
    # Looser than a confident fix: odometry may have drifted since the jump, and AMCL should be free to correct.
    was, now, publishes = set_initial_pose(node, pose, 0.1, 0.07)
    return {"pose_set": [round(v, 3) for v in pose], "estimate_was": [round(v, 3) for v in was],
            "estimate_now": [round(v, 3) for v in now], "moved_m": round(math.dist(was[:2], now[:2]), 2),
            "from_correction": list(before), "publishes": publishes}


def clear_costmaps(node):
    """Drop everything the costmaps have accumulated; they rebuild from the map and the next scans."""
    from nav2_msgs.srv import ClearEntireCostmap
    cleared = []
    for name in ("/global_costmap/clear_entirely_global_costmap", "/local_costmap/clear_entirely_local_costmap"):
        call(node, ClearEntireCostmap, name, ClearEntireCostmap.Request())
        cleared.append(name.split("/")[1])
    return {"cleared": cleared}


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return Path(f"/proc/{pid}/stat").read_text().split(") ")[-1][0] != "Z"


def wait_for(condition, seconds, period=0.5):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(period)
    return False


def launch_again(run, table, name):
    """Start a recorded process again with its original command line, replacing it if it is still running."""
    entry = table[name]
    was_alive = alive(entry["pid"])
    if was_alive:
        os.kill(entry["pid"], signal.SIGKILL)
        wait_for(lambda: not alive(entry["pid"]), 5, 0.1)
    with open(run / f"restart-{name}.log", "a", encoding="utf-8") as log:
        process = subprocess.Popen(entry["argv"], cwd=entry["cwd"], stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
    # The table always names the current process, so a second restart replaces this one instead of adding to it.
    table[name] = {**entry, "pid": process.pid}
    (run / "processes.json").write_text(json.dumps(table), encoding="utf-8")
    restarted = run / "restarted.json"
    pids = json.loads(restarted.read_text()) if restarted.exists() else []
    restarted.write_text(json.dumps(pids + [process.pid]))
    return {"was_running": was_alive, "old_pid": entry["pid"], "new_pid": process.pid}


def managed_nodes(entry):
    """The servers a lifecycle manager looks after, from the parameter file it was started with."""
    import yaml

    def find(level):
        if isinstance(level, dict):
            if "node_names" in level:
                return level["node_names"]
            for value in level.values():
                found = find(value)
                if found:
                    return found
        return None

    class Loader(yaml.SafeLoader):
        """The launch system writes lists as Python tuples, which the plain safe loader refuses."""

    Loader.add_constructor("tag:yaml.org,2002:python/tuple",
                           lambda loader, sequence: list(loader.construct_sequence(sequence)))
    for flag, value in zip(entry["argv"], entry["argv"][1:], strict=False):
        if flag == "--params-file":
            found = find(yaml.load(Path(value).read_text(encoding="utf-8"), Loader=Loader))  # noqa: S506
            if found:
                return list(found)
    return []


def unconfigure(node, name):
    """Take one lifecycle node back to unconfigured, from whatever state it is in. Returns the state found."""
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState, GetState
    state = call(node, GetState, f"/{name}/get_state", GetState.Request(), timeout=5.0).current_state.label
    steps = {"active": [Transition.TRANSITION_DEACTIVATE, Transition.TRANSITION_CLEANUP],
             "inactive": [Transition.TRANSITION_CLEANUP]}.get(state, [])
    for step in steps:
        call(node, ChangeState, f"/{name}/change_state",
             ChangeState.Request(transition=Transition(id=step)), timeout=15.0)
    return state


def restart(node, run, name):
    """Start a dead or hung process again with its original command line, and see its lifecycle group active.

    For a Nav2 server there are two cases. If the restart comes within a few seconds of the death, the lifecycle
    manager notices the missed heartbeat, resets its nodes, finds the server back and starts the group itself.
    If it comes later, the manager has already tried to reset the dead server and is stuck waiting for an answer
    that will never come; it serves no requests and ignores an interrupt. Then the manager is replaced as well:
    every server it managed is taken back to unconfigured, and a fresh manager starts the group from there.
    """
    from std_srvs.srv import Trigger
    run = Path(run)
    table = json.loads((run / "processes.json").read_text(encoding="utf-8"))
    if name not in table:
        raise RuntimeError(f"{name} is not a process this run started; known: {', '.join(sorted(table))}")
    done = {"process": name, **launch_again(run, table, name)}
    if name == RELAY:
        return done
    manager = "lifecycle_manager_localization" if name in LOCALIZATION_NODES else "lifecycle_manager_navigation"
    answers = {"last": True}

    def active():
        try:
            answers["last"] = True
            return bool(call(node, Trigger, f"/{manager}/is_active", Trigger.Request(), timeout=3.0).success)
        except RuntimeError:
            answers["last"] = False    # resetting its nodes, or stuck on the dead one
            return False

    noticed = wait_for(lambda: not active(), 8.0)
    came_up = answers["last"] and wait_for(lambda: active() or not answers["last"], 14.0) and answers["last"]
    done.update(lifecycle_manager=manager, manager_noticed_the_death=noticed, manager_replaced=False)
    if not came_up:
        if manager not in table:
            raise RuntimeError(f"{manager} is unresponsive and this run has no record of how it was started")
        done["manager_replaced"] = True
        entry = table[manager]
        if alive(entry["pid"]):
            os.kill(entry["pid"], signal.SIGKILL)
            wait_for(lambda: not alive(entry["pid"]), 5, 0.1)
        time.sleep(1.0)
        done["states_found"] = {}
        for managed in managed_nodes(entry):
            try:
                done["states_found"][managed] = unconfigure(node, managed)
            except RuntimeError as exc:
                done["states_found"][managed] = f"unreachable ({exc})"
        launch_again(run, table, manager)
        came_up = wait_for(active, 45.0, 1.0)
    done["active"] = came_up
    if not came_up:
        raise RuntimeError(f"{name} was restarted but {manager} did not become active: {json.dumps(done)}")
    return done


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    direct = sub.add_parser("command")
    direct.add_argument("operation", choices=["pause", "resume", "status", "defer", "retry"])
    direct.add_argument("--parcels", nargs="*", default=[])
    direct.add_argument("--reason", default="")
    restore = sub.add_parser("restore-parameter")
    restore.add_argument("node")
    restore.add_argument("name")
    pose = sub.add_parser("relocalize")
    pose.add_argument("before", type=float, nargs=3, metavar=("X", "Y", "YAW"),
                      help="The map-to-odom correction from before the jump")
    sub.add_parser("clear-costmaps")
    again = sub.add_parser("restart")
    again.add_argument("run", type=Path)
    again.add_argument("name")
    args = parser.parse_args()
    rclpy.init()
    node = Node("recovery_tools", parameter_overrides=[Parameter("use_sim_time", value=True)])
    try:
        if args.action == "command":
            done = command(node, args.operation, args.parcels, args.reason)
        elif args.action == "restore-parameter":
            done = restore_parameter(node, args.node, args.name)
        elif args.action == "relocalize":
            done = relocalize(node, args.before)
        elif args.action == "clear-costmaps":
            done = clear_costmaps(node)
        else:
            done = restart(node, args.run, args.name)
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
