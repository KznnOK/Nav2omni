from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="omni_robot_gazebo",
            executable="omni_odometry",
            name="omni_odometry",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
