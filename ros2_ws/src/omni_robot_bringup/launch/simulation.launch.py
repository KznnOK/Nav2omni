from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_launch(package_name, launch_file_name):
    package_share = Path(
        get_package_share_directory(package_name)
    )
    launch_file = package_share / "launch" / launch_file_name

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file))
    )


def generate_launch_description():
    gazebo = include_launch(
        "omni_robot_gazebo",
        "gazebo.launch.py",
    )

    odometry = include_launch(
        "omni_robot_gazebo",
        "odometry.launch.py",
    )

    localization = include_launch(
        "omni_robot_nav2",
        "localization.launch.py",
    )

    navigation = include_launch(
        "omni_robot_nav2",
        "navigation.launch.py",
    )

    navigate_to_goal = include_launch(
        "omni_robot_nav2",
        "navigate_to_goal.launch.py",
    )

    return LaunchDescription([
        gazebo,
        TimerAction(
            period=5.0,
            actions=[odometry],
        ),
        TimerAction(
            period=8.0,
            actions=[localization],
        ),
        TimerAction(
            period=12.0,
            actions=[navigation],
        ),
        TimerAction(
            period=13.0,
            actions=[navigate_to_goal],
        ),
    ])
