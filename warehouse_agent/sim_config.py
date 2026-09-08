"""Generate Nav2 parameters from its installed defaults and our world config."""

from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from .world import World


def generate():
    world = World()
    directory = world.generate()
    base = Path(get_package_share_directory("nav2_bringup")) / "params/nav2_params.yaml"
    parameters = yaml.safe_load(base.read_text())
    x, y, yaw = world.waypoints["HOME"]
    parameters["amcl"]["ros__parameters"].update({
        "set_initial_pose": True, "initial_pose": {"x": x, "y": y, "z": 0.0, "yaw": yaw}})
    (directory / "nav2_params.yaml").write_text(yaml.safe_dump(parameters, sort_keys=False))
    # The packaged URDF uses an older mesh layout. Repair a display-only copy.
    robot_share = Path(get_package_share_directory("nav2_minimal_tb3_sim"))
    robot_description = (robot_share/"urdf/turtlebot3_waffle.urdf").read_text()
    for mesh in (robot_share/"models/turtlebot3_model/meshes").glob("*.dae"):
        robot_description = robot_description.replace(
            "package://nav2_minimal_tb3_sim/models/"+mesh.name,
            "package://nav2_minimal_tb3_sim/"+str(mesh.relative_to(robot_share)))
    robot_file = directory/"warehouse_robot.urdf"
    robot_file.write_text(robot_description)
    rviz = yaml.safe_load((base.parent.parent/"rviz/nav2_default_view.rviz").read_text())
    manager = rviz["Visualization Manager"]
    manager["Displays"].append({
        "Class": "rviz_default_plugins/MarkerArray", "Name": "Warehouse mission",
        "Enabled": True, "Value": True, "Namespaces": {},
        "Topic": {"Value": "/warehouse/markers", "Depth": 1, "Durability Policy": "Transient Local",
                  "History Policy": "Keep Last", "Reliability Policy": "Reliable"}})
    for display in manager["Displays"]:
        if display.get("Name") == "RobotModel":
            display["Enabled"] = display["Value"] = True
            display["Description Source"] = "File"
            display["Description File"] = str(robot_file)
            # Avoid a topic update replacing the display file during RViz load.
            display["Description Topic"]["Value"] = ""
        if display.get("Name") in {"TF", "Amcl Particle Swarm", "Bumper Hit"}:
            display["Enabled"] = display["Value"] = False
    manager["Global Options"]["Background Color"] = "235; 240; 245"
    manager["Views"]["Current"].update({"X": 0.0, "Y": 0.0, "Scale": 65})
    (directory/"warehouse.rviz").write_text(yaml.safe_dump(rviz, sort_keys=False))
    print(directory)


if __name__ == "__main__":
    generate()
