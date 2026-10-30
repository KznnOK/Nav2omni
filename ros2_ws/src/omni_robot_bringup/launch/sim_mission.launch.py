"""sim_check (Gazebo, odometry, localization, RViz) + mode manager + mission.

No controller in the simulation: drive with
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_manual
and switch MANUAL <-> AUTO with
  ros2 service call /mode_manager/toggle_autonomy std_srvs/srv/Trigger
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch(package_name, launch_file_name, arguments):
    launch_file = (Path(get_package_share_directory(package_name))
                   / "launch" / launch_file_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    names = ["x", "y", "yaw", "gui", "rviz", "geometry_file", "sim_conditions"]
    sim = include_launch("omni_robot_bringup", "sim_check.launch.py",
                         {n: LaunchConfiguration(n) for n in names})
    mission = include_launch("omni_robot_mission", "mission.launch.py", {
        "use_sim_time": "true",
        "use_joy": LaunchConfiguration("use_joy"),
        "require_joy": LaunchConfiguration("use_joy"),
        "geometry_file": LaunchConfiguration("geometry_file"),
        "mission_params": LaunchConfiguration("mission_params"),
    })
    geometry = str(Path(get_package_share_directory("omni_robot_description"))
                   / "config" / "robot_geometry.yaml")
    conditions = str(Path(get_package_share_directory("omni_robot_gazebo"))
                     / "config" / "sim_conditions.yaml")
    params = str(Path(get_package_share_directory("omni_robot_mission"))
                 / "config" / "sakura_mission.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="-1.055"),
        DeclareLaunchArgument("y", default_value="-3.112"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("geometry_file", default_value=geometry),
        DeclareLaunchArgument("sim_conditions", default_value=conditions),
        DeclareLaunchArgument("mission_params", default_value=params),
        # true: a PS4 controller is connected to this PC (joy_node)
        DeclareLaunchArgument("use_joy", default_value="false"),
        sim,
        TimerAction(period=10.0, actions=[mission]),
    ])
