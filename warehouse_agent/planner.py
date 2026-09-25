"""Reuse WarehouseBot A*; greedy sequencing respects live cargo and priority."""

from dataclasses import dataclass, asdict
import importlib.util
import math
from .world import ROOT


@dataclass(frozen=True)
class Stop:
    parcel: str
    kind: str
    waypoint: str

    def as_dict(self):
        return asdict(self)


class PlanningError(ValueError):
    pass


class Planner:
    def __init__(self, world):
        self.world = world
        path = ROOT / "references/warehousebot/warehousebot/astar.py"
        if not path.exists():
            raise RuntimeError("Run scripts/fetch_reference.sh to fetch the pinned WarehouseBot planner")
        spec = importlib.util.spec_from_file_location("warehousebot_astar", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.astar = module.a_star
        self.cache = {}

    def distance(self, a, b):
        start, offset = self.world.snap(a)
        if start is None:
            return math.inf
        key = (start, self.world.cell(b))
        if key not in self.cache:
            self.cache[key] = self.astar(self.world.grid, *key)["cost"] * self.world.resolution
        return offset+self.cache[key]

    def plan(self, parcels, pose, priority=None, onboard_first=()):
        remaining = []
        for pid, parcel in parcels.items():
            if parcel["disposition"] != "active" or parcel["state"] == "delivered":
                continue
            if parcel["state"] == "awaiting_pickup":
                remaining.append(Stop(pid, "pickup", parcel["pickup"]))
            remaining.append(Stop(pid, "drop", parcel["drop"]))
        onboard = {pid for pid, p in parcels.items() if p["state"] == "onboard"}
        route, total = [], 0.0
        while remaining:
            eligible = [s for s in remaining if s.kind == "pickup" or s.parcel in onboard]
            protected = [s for s in eligible if s.kind == "drop" and s.parcel in onboard_first]
            preferred = [s for s in eligible if s.parcel == priority]
            choices = protected or preferred or eligible
            ranked = [(self.distance(pose, self.world.waypoints[s.waypoint]), s.parcel, s.kind, s) for s in choices]
            cost, _, _, stop = min(ranked)
            if not math.isfinite(cost):
                raise PlanningError(f"No estimated route to {stop.waypoint}; mission held")
            route.append(stop)
            total += cost
            remaining.remove(stop)
            onboard.add(stop.parcel) if stop.kind == "pickup" else onboard.discard(stop.parcel)
            pose = self.world.waypoints[stop.waypoint]
        return route, total
