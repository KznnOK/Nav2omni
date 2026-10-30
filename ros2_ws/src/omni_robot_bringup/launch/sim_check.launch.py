"""Gazebo + odometry + localization + RViz for checking the model by eye.

No navigation is started; drive the robot with teleop_twist_keyboard.
The default spawn pose is in front of the dice zone (phase 2 report).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_launch(package_name, launch_file_name, arguments=None):
    launch_file = (Path(get_package_share_directory(package_name))
                   / "launch" / launch_file_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    rviz_config = (Path(get_package_share_directory("omni_robot_bringup"))
                   / "rviz" / "sim_check.rviz")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")

    gazebo = include_launch("omni_robot_gazebo", "gazebo.launch.py", {
        "spawn_x": x,
        "spawn_y": y,
        "spawn_yaw": yaw,
        "gui": LaunchConfiguration("gui"),
        "sim_conditions": LaunchConfiguration("sim_conditions"),
    })
    odometry = include_launch("omni_robot_gazebo", "odometry.launch.py")
    localization = include_launch("omni_robot_nav2", "localization.launch.py", {
        "initial_x": x,
        "initial_y": y,
        "initial_yaw": yaw,
    })

    # Simulation only: /ground_truth/odom is in Gazebo's "world" frame, which
    # coincides with "map" because the map origin is the field corner.
    world_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_world",
        arguments=["--frame-id", "map", "--child-frame-id", "world"],
        parameters=[{"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(rviz_config)],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        # Dice-zone front: dice zone left line outer edge x = -0.725,
        # minus half chassis 0.25 and a 0.05 m standoff (仮値).
        DeclareLaunchArgument("x", default_value="-1.025"),
        DeclareLaunchArgument("y", default_value="-3.112"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "sim_conditions",
            default_value=str(
                Path(get_package_share_directory("omni_robot_gazebo"))
                / "config" / "sim_conditions.yaml"),
        ),
        gazebo,
        world_frame,
        TimerAction(period=5.0, actions=[odometry]),
        TimerAction(period=8.0, actions=[localization]),
        TimerAction(period=8.0, actions=[rviz]),
    ])
