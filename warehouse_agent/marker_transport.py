"""Render the same visual records through RViz and Gazebo's marker service."""

import json
import math
import queue
import subprocess
import threading
import time
from itertools import pairwise

from .world import ROOT

NAMESPACE = "warehouse_agent"


def line_triangles(points, width):
    """A floor ribbon keeps the goal ring visible at any camera distance."""
    result = []
    for a, b in pairwise(points):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length == 0:
            continue
        ox, oy = -dy/length*width/2, dx/length*width/2
        al, ar = (a[0]+ox, a[1]+oy, a[2]), (a[0]-ox, a[1]-oy, a[2])
        bl, br = (b[0]+ox, b[1]+oy, b[2]), (b[0]-ox, b[1]-oy, b[2])
        result.extend((al, ar, br, al, br, bl))
    return result


def gazebo_message(visuals, removed=()):
    records = []
    for key in removed:
        records.append(f'marker {{ns: "{NAMESPACE}" id: {key} action: DELETE_MARKER}}')
    types = {"box": "BOX", "text": "TRIANGLE_LIST", "line": "TRIANGLE_LIST"}
    for v in visuals:
        x, y, z = v.position
        sx, sy, sz = v.scale
        points = v.points
        if v.shape == "text":
            from .gazebo_text import triangles
            points = triangles(v.text, v.position, sz)
            sx = sy = sz = 1
        elif v.shape == "line":
            points = line_triangles(points, sx)
            sx = sy = sz = 1
        r, g, b, a = v.color
        color = f"r: {r} g: {g} b: {b} a: {a}"
        pose = f"position {{x: {x} y: {y} z: {z}}} orientation {{z: {math.sin(v.yaw/2)} w: {math.cos(v.yaw/2)}}}"
        points = " ".join(f"point {{x: {p[0]:.5f} y: {p[1]:.5f} z: {p[2]:.5f}}}" for p in points)
        records.append(f'marker {{ns: "{NAMESPACE}" id: {v.key} action: ADD_MODIFY type: {types[v.shape]} '
                       f'pose {{{pose}}} scale {{x: {sx} y: {sy} z: {sz}}} '
                       f'material {{ambient {{{color}}} diffuse {{{color}}} lighting: false}} '
                       f'text: {json.dumps(v.text)} {points}}}')
    return " ".join(records)


def ros_message(visuals, removed, stamp):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
    array = MarkerArray()
    for key in removed:
        msg = Marker()
        msg.header.frame_id, msg.header.stamp = "map", stamp
        msg.ns, msg.id, msg.action = NAMESPACE, key, Marker.DELETE
        array.markers.append(msg)
    types = {"box": Marker.CUBE, "text": Marker.TEXT_VIEW_FACING, "line": Marker.LINE_STRIP}
    for v in visuals:
        msg = Marker()
        msg.header.frame_id, msg.header.stamp = "map", stamp
        msg.ns, msg.id, msg.type, msg.action = NAMESPACE, v.key, types[v.shape], Marker.ADD
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, v.position)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(v.yaw/2), math.cos(v.yaw/2)
        msg.scale.x, msg.scale.y, msg.scale.z = map(float, v.scale)
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = map(float, v.color)
        msg.text = v.text
        msg.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in v.points]
        array.markers.append(msg)
    return array


class GazeboMarkers:
    """Keep at most one pending frame; slow/closed GUI cannot delay navigation."""
    def __init__(self):
        self.frames = queue.Queue(maxsize=1)
        self.alive = True
        self.available = False
        self.error = "Waiting for Gazebo GUI"
        self.process = subprocess.Popen([str(ROOT/"build/marker_bridge/warehouse_marker_bridge")],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.thread = threading.Thread(target=self.worker, daemon=True)
        self.thread.start()

    def submit(self, visuals):
        try:
            self.frames.get_nowait()
        except queue.Empty:
            pass
        self.frames.put_nowait(visuals)

    def worker(self):
        known = set()
        reset = True
        previous = {}
        refresh_at = 0
        framed = False
        frame_retry_at = 0
        while self.alive:
            try:
                visuals = self.frames.get(timeout=0.2)
            except queue.Empty:
                continue
            current = {v.key for v in visuals}
            # Always delete absent IDs that this viewer could have created.
            removed = known-current
            changed = visuals if time.monotonic() >= refresh_at else [v for v in visuals if previous.get(v.key) != v]
            if not changed and not removed:
                continue
            try:
                clear = f'marker {{ns: "{NAMESPACE}" action: DELETE_ALL}} ' if reset else ""
                self.process.stdin.write(clear+gazebo_message(changed, removed)+"\n")
                self.process.stdin.flush()
                reply = self.process.stdout.readline().strip()
                self.available = reply == "ok"
                self.error = "" if self.available else "Gazebo GUI unavailable"
                if reply in {"ok", "rejected"}:
                    known = current
                    reset = False
                    previous = {v.key: v for v in visuals}
                    if time.monotonic() >= refresh_at:
                        refresh_at = time.monotonic()+2
                if self.available and not framed and time.monotonic() >= frame_retry_at:
                    from .capture_scene import frame_camera
                    framed = frame_camera()
                    frame_retry_at = time.monotonic()+5
                if not reply:
                    self.error = "Gazebo marker bridge stopped"
                    return
            except (BrokenPipeError, OSError, ValueError) as exc:
                self.error = str(exc)
                self.available = False
                return

    def close(self):
        self.alive = False
        self.process.terminate()
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.thread.join(timeout=1)
