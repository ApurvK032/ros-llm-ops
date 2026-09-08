"""One metric geometry definition shared by Gazebo, Nav2 and task planning."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


class World:
    def __init__(self, path=ROOT / "config/warehouse.json"):
        self.config = json.loads(Path(path).read_text())
        self.waypoints = self.config["waypoints"]
        self.resolution = self.config["planning_resolution"]
        self.bounds = self.config["bounds"]
        self.grid = self.make_grid(self.resolution, self.config["planning_inflation"])

    def obstacles(self):
        x0, y0, x1, y1 = self.bounds
        return [*self.config["shelves"],
                {"name": "wall_west", "center": [x0, 0], "size": [0.15, y1-y0]},
                {"name": "wall_east", "center": [x1, 0], "size": [0.15, y1-y0]},
                {"name": "wall_south", "center": [0, y0], "size": [x1-x0, 0.15]},
                {"name": "wall_north", "center": [0, y1], "size": [x1-x0, 0.15]}]

    def blocked(self, x, y, margin=0):
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
        for obstacle in self.obstacles():
            x, y = obstacle["center"]
            sx, sy = obstacle["size"]
            color = "0.2 0.35 0.5 1" if obstacle["name"].startswith("shelf") else "0.55 0.58 0.62 1"
            models.append(f'''<model name="{obstacle['name']}"><static>true</static><pose>{x} {y} 0.6 0 0 0</pose><link name="body">
<collision name="collision"><geometry><box><size>{sx} {sy} 1.2</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>{sx} {sy} 1.2</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
</link></model>''')
        for name, (x, y, _) in self.waypoints.items():
            color = "0.2 0.8 0.45 1" if name.startswith("DROP") else "0.95 0.65 0.1 1"
            models.append(f'''<model name="{name}"><static>true</static><pose>{x} {y} 0.008 0 0 0</pose><link name="marker"><visual name="pad"><geometry><cylinder><radius>0.36</radius><length>0.01</length></cylinder></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>''')
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
        return directory


if __name__ == "__main__":
    print(World().generate())
