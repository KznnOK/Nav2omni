import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def include(package, filename):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / filename)))


def generate_launch_description():
    package_share = Path(get_package_share_directory("omni_robot_manual"))
    nav_share = Path(get_package_share_directory("omni_robot_nav2"))
    config = package_share / "config" / "manual.yaml"
    mission = nav_share / "config" / "mission.yaml"
    input_type = LaunchConfiguration("input")
    diagnostic = LaunchConfiguration("diagnostic")
    start_input_node = LaunchConfiguration("start_input_node")
    input_enabled = PythonExpression([
        "'", start_input_node, "' == 'true' and '", input_type, "' == 'keyboard'"
    ])
    joy_enabled = PythonExpression([
        "'", start_input_node, "' == 'true' and '", input_type, "' == 'joy'"
    ])
    keyboard = Node(
        package="omni_robot_manual",
        executable="keyboard_manual_control",
        parameters=[str(config)],
        condition=IfCondition(input_enabled),
    )
    return LaunchDescription([
        DeclareLaunchArgument("input", default_value="keyboard"),
        DeclareLaunchArgument("diagnostic", default_value="false"),
        DeclareLaunchArgument("start_input_node", default_value="true"),
        include("omni_robot_gazebo", "gazebo.launch.py"),
        TimerAction(period=5.0, actions=[
            include("omni_robot_gazebo", "odometry.launch.py")]),
        TimerAction(period=8.0, actions=[
            include("omni_robot_nav2", "localization.launch.py")]),
        TimerAction(period=9.0, actions=[Node(
            package="omni_robot_manual", executable="manual_cmd_mux",
            parameters=[str(config)])]),
        TimerAction(period=9.0, actions=[keyboard]),
        TimerAction(period=9.0, actions=[Node(
            package="omni_robot_manual", executable="joy_manual_control",
            parameters=[str(config), {"diagnostic": diagnostic}],
            condition=IfCondition(joy_enabled))]),
        TimerAction(period=9.0, actions=[Node(
            package="omni_robot_manual", executable="manual_recorder",
            parameters=[str(config), {
                "mission_config": str(mission),
                "input_mode": input_type,
            }])]),
    ])
