"""Stock TurtleBot3/Nav2 simulation plus the read-only warehouse display."""
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = Path(__file__).resolve().parent.parent
    generated = root / "sim/generated"
    defaults = {
        "headless": "False", "use_rviz": "True", "show_panel": "True",
        "world": str(generated/"warehouse.sdf"), "map": str(generated/"warehouse.yaml"),
        "params_file": str(generated/"nav2_params.yaml"),
        "rviz_config_file": str(generated/"warehouse.rviz"),
        "x_pose": "-3.8", "y_pose": "-2.8", "yaw": "0.0",
    }
    description = LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()])
    robot_share = Path(get_package_share_directory("nav2_minimal_tb3_sim"))
    description.add_action(AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(robot_share/"models")))
    description.add_action(AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(robot_share.parent)))
    description.add_action(GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory("nav2_bringup"))/"launch/tb3_simulation_launch.py")),
        launch_arguments={**{k: LaunchConfiguration(k) for k in defaults if k not in {"show_panel", "headless"}},
                          "headless": "True"}.items())], scoped=True))
    description.add_action(ExecuteProcess(
        cmd=["gz", "sim", "-g", "--gui-config", str(root/"sim/warehouse.gui.config")],
        condition=UnlessCondition(LaunchConfiguration("headless")), output="screen"))

    def visuals(context):
        cmd = [sys.executable, "-B", "-m", "warehouse_agent.visualization"]
        if LaunchConfiguration("show_panel").perform(context).lower() == "true":
            cmd.append("--panel")
        if LaunchConfiguration("headless").perform(context).lower() == "true":
            cmd.append("--no-gazebo")
        return [ExecuteProcess(cmd=cmd, cwd=str(root), output="screen")]

    description.add_action(OpaqueFunction(function=visuals))
    return description
