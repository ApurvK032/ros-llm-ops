"""One metric geometry definition shared by Gazebo, Nav2 and task planning."""

import json
import math
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent.parent


class World:
    def __init__(self, path=ROOT / "config/warehouse.json"):
        self.config = json.loads(Path(path).read_text())
        self.waypoints = self.config["waypoints"]
        self.resolution = self.config["planning_resolution"]
        self.bounds = self.config["bounds"]
        self.grid = self.make_grid(self.resolution, self.config["planning_inflation"])
        for name, pose in self.waypoints.items():
            if self.blocked(*pose[:2], self.config["planning_inflation"]):
                raise ValueError(f"Approach pose {name} is inside an inflated obstacle")
        for name in self.config["stations"]:
            if not self.arrival(self.waypoints[name], name)["accepted"]:
                raise ValueError(f"Station {name} must face its parcel with sufficient clearance")

    @staticmethod
    def angle_error(a, b):
        return abs(math.atan2(math.sin(a-b), math.cos(a-b)))

    def station_position(self, name):
        return tuple(self.config["stations"][name]["parcel_position"])

    def arrival(self, pose, name):
        """Check the measured base pose against the approach and parcel separately."""
        goal = self.waypoints[name]
        parcel = self.station_position(name)
        distance = math.dist(pose[:2], parcel[:2])
        bearing = math.atan2(parcel[1]-pose[1], parcel[0]-pose[0])
        report = {
            "position_error": math.dist(pose[:2], goal[:2]),
            "heading_error": self.angle_error(pose[2], goal[2]),
            "facing_error": self.angle_error(pose[2], bearing),
            "parcel_distance": distance,
            # Conservative circumscribed footprints guarantee no 2D overlap.
            "parcel_clearance": distance-self.config["robot_radius"]-math.hypot(*self.config["parcel_size"][:2])/2,
        }
        report["accepted"] = (report["position_error"] <= self.config["arrival_tolerance"] and
                              report["heading_error"] <= self.config["heading_tolerance"] and
                              report["facing_error"] <= self.config["facing_tolerance"] and
                              report["parcel_clearance"] >= self.config["minimum_parcel_clearance"])
        return report

    def obstacles(self):
        x0, y0, x1, y1 = self.bounds
        pedestals = [{"name": "dock_"+name, "center": station["parcel_position"][:2],
                      "size": station["pedestal_size"][:2], "height": station["pedestal_size"][2]}
                     for name, station in self.config["stations"].items() if "pedestal_size" in station]
        return [*self.config["shelves"], *pedestals,
                {"name": "wall_west", "center": [x0, (y0+y1)/2], "size": [0.15, y1-y0]},
                {"name": "wall_east", "center": [x1, (y0+y1)/2], "size": [0.15, y1-y0]},
                {"name": "wall_south", "center": [(x0+x1)/2, y0], "size": [x1-x0, 0.15]},
                {"name": "wall_north", "center": [(x0+x1)/2, y1], "size": [x1-x0, 0.15]}]

    def blocked(self, x, y, margin=0):
        x0, y0, x1, y1 = self.bounds
        if not (x0+margin < x < x1-margin and y0+margin < y < y1-margin):
            return True
        return any(abs(x-o["center"][0]) <= o["size"][0]/2+margin and
                   abs(y-o["center"][1]) <= o["size"][1]/2+margin
                   for o in self.obstacles())

    def make_grid(self, resolution, margin=0):
        x0, y0, x1, y1 = self.bounds
        return [[int(self.blocked(x0+(c+0.5)*resolution, y0+(r+0.5)*resolution, margin))
                 for c in range(round((x1-x0)/resolution))]
                for r in range(round((y1-y0)/resolution))]

    def cell(self, pose):
        return (int((pose[1]-self.bounds[1])/self.resolution),
                int((pose[0]-self.bounds[0])/self.resolution))

    def centre(self, cell):
        return (self.bounds[0]+(cell[1]+0.5)*self.resolution, self.bounds[1]+(cell[0]+0.5)*self.resolution)

    def free(self, cell):
        return 0 <= cell[0] < len(self.grid) and 0 <= cell[1] < len(self.grid[0]) and not self.grid[cell[0]][cell[1]]

    def snap(self, pose):
        """Nearest free planning cell and its distance; the reach is too short to cross a rack."""
        start = self.cell(pose)
        if self.free(start):
            return start, 0.0
        radius = self.config["planning_inflation"]+self.resolution
        reach = math.ceil(radius/self.resolution)+1
        # Rounding makes geometric ties exact, so the lower cell wins them deterministically.
        nearby = [(round(math.dist(pose[:2], self.centre(cell)), 9), cell)
                  for cell in ((start[0]+dr, start[1]+dc) for dr in range(-reach, reach+1) for dc in range(-reach, reach+1))
                  if self.free(cell)]
        distance, cell = min(nearby, default=(math.inf, None))
        return (cell, distance) if distance <= radius else (None, math.inf)

    def generate(self, directory=ROOT / "sim/generated"):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        resolution = self.config["map_resolution"]
        grid = self.make_grid(resolution)
        # Occupancy images are top-down; planning rows increase with world y.
        pixels = bytes(0 if occupied else 254 for row in reversed(grid) for occupied in row)
        (directory / "warehouse.pgm").write_bytes(f"P5\n{len(grid[0])} {len(grid)}\n255\n".encode()+pixels)
        (directory / "warehouse.yaml").write_text(
            f"image: warehouse.pgm\nmode: trinary\nresolution: {resolution}\n"
            f"origin: [{self.bounds[0]}, {self.bounds[1]}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        models = []
        def box(name, size, pose, color):
            return (f'<visual name="{name}"><pose>{" ".join(map(str, pose))} 0 0 0</pose>'
                    f'<geometry><box><size>{" ".join(map(str, size))}</size></box></geometry>'
                    f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>')
        for obstacle in self.obstacles():
            x, y = obstacle["center"]
            sx, sy = obstacle["size"]
            shelf = obstacle["name"].startswith("shelf")
            height = 1.6 if shelf else obstacle.get("height", 0.65)
            parts = []
            if shelf:
                parts.append(box("base", (sx, sy, 0.42), (0, 0, 0.21), "0.20 0.29 0.38 1"))
                for index, z in enumerate((0.57, 1.20, 1.57)):
                    parts.append(box(f"deck_{index}", (sx, sy, 0.06), (0, 0, z), "0.88 0.46 0.12 1"))
                count = max(1, math.ceil(max(sx, sy)/1.5))
                for index in range(count+1):
                    for side in (-1, 1):
                        px = (index/count-0.5)*(sx-0.10) if sx >= sy else side*(sx-0.10)/2
                        py = side*(sy-0.10)/2 if sx >= sy else (index/count-0.5)*(sy-0.10)
                        parts.append(box(f"upright_{index}_{side}", (0.09, 0.09, height), (px, py, height/2), "0.13 0.28 0.45 1"))
            else:
                color = "0.18 0.52 0.42 1" if obstacle["name"].startswith("dock") else "0.55 0.58 0.62 1"
                parts.append(box("body", (sx, sy, height), (0, 0, height/2), color))
            models.append(f'''<model name="{obstacle['name']}"><static>true</static><pose>{x} {y} 0 0 0 0</pose><link name="body">
<collision name="collision"><pose>0 0 {height/2} 0 0 0</pose><geometry><box><size>{sx} {sy} {height}</size></box></geometry></collision>
{"".join(parts)}</link></model>''')
        for name, (x, y, yaw) in self.waypoints.items():
            color = "0.2 0.8 0.45 1" if name.startswith("DROP") else "0.95 0.65 0.1 1"
            models.append(f'''<model name="{name}"><static>true</static><pose>{x} {y} 0.008 0 0 {yaw}</pose><link name="marker"><visual name="pad"><geometry><cylinder><radius>0.27</radius><length>0.01</length></cylinder></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>''')
        (directory / "warehouse.sdf").write_text('''<?xml version="1.0"?>
<sdf version="1.9"><world name="warehouse_mvp">
<physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.5 0.2 -1</direction><cast_shadows>true</cast_shadows></light>
<model name="ground"><static>true</static><link name="ground"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry><material><ambient>0.82 0.83 0.85 1</ambient><diffuse>0.82 0.83 0.85 1</diffuse></material></visual></link></model>
''' + "\n".join(models) + "\n</world></sdf>\n")
        camera = self.config["overview_camera"]
        dx, dy = (self.bounds[0]+self.bounds[2])/2-camera[0], (self.bounds[1]+self.bounds[3])/2-camera[1]
        yaw, pitch = math.atan2(dy, dx), math.atan2(camera[2], math.hypot(dx, dy))
        gui = (ROOT / "sim/warehouse.gui.config").read_text()
        gui = re.sub(r"<camera_pose>.*?</camera_pose>",
                     f'<camera_pose>{" ".join(map(str, camera))} 0 {pitch} {yaw}</camera_pose>', gui)
        (directory / "warehouse.gui.config").write_text(gui)
        return directory


if __name__ == "__main__":
    print(World().generate())
