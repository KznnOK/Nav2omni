from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("omni_robot_gazebo")
    geometry_file = os.path.join(
        package_share, "config", "wheel_geometry.yaml")
    ekf_file = os.path.join(package_share, "config", "ekf.yaml")
    return LaunchDescription([
        Node(
            package="omni_robot_gazebo",
            executable="imu_covariance",
            name="imu_covariance",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="omni_robot_gazebo",
            executable="omni_odometry",
            name="omni_odometry",
            output="screen",
            parameters=[{"use_sim_time": True}, geometry_file],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_file, {"use_sim_time": True}],
            # Nav2 (bt_navigator, controller_server) and mission_manager read /odom.
            remappings=[("odometry/filtered", "/odom")],
        ),
    ])
