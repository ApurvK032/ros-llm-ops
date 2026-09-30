"""Read-only projection of mission snapshots into both simulation displays."""

import math
from dataclasses import dataclass

COLORS = {
    "awaiting_pickup": (0.96, 0.60, 0.12, 1.0),
    "onboard": (0.18, 0.47, 0.94, 1.0),
    "delivered": (0.10, 0.67, 0.39, 1.0),
    "cancelled": (0.49, 0.53, 0.59, 1.0),
    "deferred": (0.85, 0.20, 0.24, 1.0),
    "goal": (0.02, 0.66, 0.78, 1.0),
    "label": (0.08, 0.13, 0.22, 1.0),
}


@dataclass(frozen=True)
class Visual:
    key: int
    shape: str
    position: tuple
    scale: tuple
    color: tuple
    text: str = ""
    yaw: float = 0.0
    points: tuple = ()


HOLD_LABELS = {"no_route": "Held: no route from current pose", "clarification": "Waiting for clarification",
               "request_failed": "Paused: request not accepted", "operator": "Paused"}


def initial_state(world):
    return {"revision": 0, "pose": list(world.waypoints["HOME"]), "active": None,
            "paused": False, "hold": None, "language_pending": False, "finished": True,
            "closed": False, "remaining_stops": [], "plan_pending": False,
            "cancel_requested": False,
            "parcels": {pid: {**p, "state": "awaiting_pickup", "disposition": "inactive"}
                        for pid, p in world.config["parcels"].items()}}


def parcel_label(parcel):
    physical = {"awaiting_pickup": "Waiting", "onboard": "Onboard", "delivered": "Delivered"}[parcel["state"]]
    if parcel["disposition"] in {"cancelled", "deferred"}:
        return f"{parcel['disposition'].title()} · {physical.lower()}"
    return physical


def parcel_color(parcel):
    return COLORS.get(parcel["disposition"], COLORS[parcel["state"]])


def mission_label(state):
    if state.get("closed"):
        return "Session ended"
    if state.get("disconnected"):
        return "Disconnected"
    if state.get("cancel_requested"):
        return "Stopping"
    hold = state.get("hold") or {}
    if hold.get("kind") in HOLD_LABELS:
        return HOLD_LABELS[hold["kind"]]
    if state["paused"]:
        return "Paused"
    if state["language_pending"]:
        return "Interpreting request"
    if state["active"]:
        return "Delivering"
    if state["finished"]:
        used = [p for p in state["parcels"].values() if p["disposition"] != "inactive"]
        if not used:
            return "Ready"
        if any(p["disposition"] == "deferred" for p in used):
            return "Finished with deferred parcels"
        return "Mission complete"
    return "Planning next stop"


def stop_label(stop):
    verb = "Pick up" if stop["kind"] == "pickup" else "Deliver"
    return f"{verb} {stop['parcel']} · {stop['waypoint'].replace('_', ' ')}"


def project(world, state):
    """Cargo location depends exclusively on confirmed physical state."""
    visuals = []
    captions = {name: [] for name in world.waypoints}
    for pid, parcel in state["parcels"].items():
        if parcel["state"] != "onboard":
            station = parcel["drop"] if parcel["state"] == "delivered" else parcel["pickup"]
            captions[station].append(f"{pid} · {parcel_label(parcel)}")
    for i, (name, pose) in enumerate(world.waypoints.items()):
        x, y, z = world.station_position(name) if name != "HOME" else (*pose[:2], 0.0)
        caption = "\n".join([name.replace("_", " "), *captions[name]])
        visuals.append(Visual(10+i, "text", (x, y, z+1.05), (0, 0, 0.28), COLORS["label"], caption))
        if name != "HOME":
            ax, ay, yaw = pose
            tip = (ax+0.58*math.cos(yaw), ay+0.58*math.sin(yaw), 0.04)
            left = (tip[0]-0.18*math.cos(yaw)+0.13*math.sin(yaw), tip[1]-0.18*math.sin(yaw)-0.13*math.cos(yaw), 0.04)
            right = (tip[0]-0.18*math.cos(yaw)-0.13*math.sin(yaw), tip[1]-0.18*math.sin(yaw)+0.13*math.cos(yaw), 0.04)
            color = COLORS["delivered"] if name.startswith("DROP") else COLORS["awaiting_pickup"]
            visuals.append(Visual(30+i, "line", (0, 0, 0), (0.045, 0, 0), color,
                                  points=((ax, ay, 0.04), tip, left, tip, right)))
    carried = [pid for pid, p in state["parcels"].items() if p["state"] == "onboard"]
    for i, (pid, parcel) in enumerate(state["parcels"].items()):
        color = parcel_color(parcel)
        if parcel["state"] == "onboard":
            x, y, yaw = state["pose"]
            side = (carried.index(pid)-(len(carried)-1)/2)*0.22
            position = (x-math.sin(yaw)*side, y+math.cos(yaw)*side, 0.48)
            size = (0.19, 0.19, 0.19)
        else:
            waypoint = parcel["drop"] if parcel["state"] == "delivered" else parcel["pickup"]
            yaw = world.waypoints[waypoint][2]
            position, size = world.station_position(waypoint), tuple(world.config["parcel_size"])
        visuals.append(Visual(100+i*2, "box", position, size, color, yaw=yaw))
    if carried:
        x, y, _ = state["pose"]
        # Keep the carried manifest above station captions and the goal label.
        visuals.append(Visual(200, "text", (x, y, 2.1), (0, 0, 0.24), COLORS["label"], "ONBOARD\n"+", ".join(carried)))
    if state["active"]:
        x, y, _ = world.waypoints[state["active"]["waypoint"]]
        points = tuple((x+0.34*math.cos(t*math.tau/48), y+0.34*math.sin(t*math.tau/48), 0.05) for t in range(49))
        visuals.append(Visual(300, "line", (0, 0, 0), (0.05, 0, 0), COLORS["goal"], points=points))
        visuals.append(Visual(301, "text", (x, y, 0.9), (0, 0, 0.24), COLORS["goal"], "STOP AND FACE"))
    return visuals
