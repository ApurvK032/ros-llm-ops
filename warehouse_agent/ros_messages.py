"""Convert supervisor snapshots to and from warehouse_interfaces messages (needs the built ROS workspace)."""


def status_message(snapshot, stamp=None, frame_id="map"):
    from warehouse_interfaces.msg import Hold, MissionStatus, Parcel, Stop

    def stop(s):
        return Stop(parcel=s["parcel"], kind=s["kind"], waypoint=s["waypoint"])

    msg = MissionStatus()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.revision = snapshot["revision"]
    msg.paused = snapshot["paused"]
    hold = snapshot.get("hold")
    msg.hold = [Hold(kind=hold["kind"], message=hold["message"])] if hold else []
    msg.language_pending = snapshot["language_pending"]
    msg.pending_request_id = snapshot.get("pending_request_id") or ""
    msg.x, msg.y, msg.yaw = (float(v) for v in snapshot["pose"])
    msg.priority = snapshot.get("priority") or ""
    msg.onboard_first = list(snapshot["onboard_first"])
    msg.active = [stop(snapshot["active"])] if snapshot["active"] else []
    msg.remaining_stops = [stop(s) for s in snapshot["remaining_stops"]]
    msg.plan_pending = snapshot["plan_pending"]
    msg.cancel_requested = snapshot["cancel_requested"]
    msg.finished = snapshot["finished"]
    msg.closed = snapshot["closed"]
    msg.parcels = [Parcel(id=pid, pickup=p["pickup"], drop=p["drop"], state=p["state"],
                          disposition=p["disposition"], reason=p.get("reason", ""))
                   for pid, p in snapshot["parcels"].items()]
    return msg


def status_snapshot(msg):
    """The inverse of status_message, so typed subscribers can reuse the snapshot-based views and checks."""
    def stop(s):
        return {"parcel": s.parcel, "kind": s.kind, "waypoint": s.waypoint}

    parcels = {}
    for p in msg.parcels:
        parcels[p.id] = {"pickup": p.pickup, "drop": p.drop, "state": p.state, "disposition": p.disposition}
        if p.reason:
            parcels[p.id]["reason"] = p.reason
    return {"revision": msg.revision, "paused": msg.paused,
            "hold": {"kind": msg.hold[0].kind, "message": msg.hold[0].message} if msg.hold else None,
            "language_pending": msg.language_pending, "pending_request_id": msg.pending_request_id or None,
            "pose": [msg.x, msg.y, msg.yaw], "priority": msg.priority or None,
            "onboard_first": list(msg.onboard_first), "active": stop(msg.active[0]) if msg.active else None,
            "remaining_stops": [stop(s) for s in msg.remaining_stops], "plan_pending": msg.plan_pending,
            "closed": msg.closed, "cancel_requested": msg.cancel_requested, "parcels": parcels,
            "finished": msg.finished}
