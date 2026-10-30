from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("omni_robot_nav2")
    )

    map_yaml_file = (
        package_share
        / "maps"
        / "field_map.yaml"
    )

    amcl_params_file = (
        package_share
        / "config"
        / "amcl.yaml"
    )

    laser_filter_file = (
        package_share
        / "config"
        / "laser_filter.yaml"
    )

    return LaunchDescription([
        # Defaults match the initial pose in amcl.yaml (original start zone).
        DeclareLaunchArgument("initial_x", default_value="-3.1510900301327864"),
        DeclareLaunchArgument("initial_y", default_value="-3.136311120883107"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        # Masks the angles where the LiDAR sees the robot itself.
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain",
            output="screen",
            parameters=[str(laser_filter_file)],
            remappings=[
                ("scan", "/scan"),
                ("scan_filtered", "/scan_filtered"),
            ],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "yaml_filename": str(map_yaml_file),
                "use_sim_time": True,
            }],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                str(amcl_params_file),
                {
                    "initial_pose.x": ParameterValue(
                        LaunchConfiguration("initial_x"), value_type=float),
                    "initial_pose.y": ParameterValue(
                        LaunchConfiguration("initial_y"), value_type=float),
                    "initial_pose.yaw": ParameterValue(
                        LaunchConfiguration("initial_yaw"), value_type=float),
                },
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                ],
            }],
        ),
    ])
