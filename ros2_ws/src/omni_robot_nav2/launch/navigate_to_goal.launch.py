from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="omni_robot_nav2",
            executable="navigate_to_goal",
            name="navigate_to_goal",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
