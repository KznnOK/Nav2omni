"""Mode manager + sakura mission (+ joy/teleop for a PS4 controller).

Shared by the simulation and the real robot. In the simulation use
use_joy:=false require_joy:=false and switch with
  ros2 service call /mode_manager/toggle_autonomy std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from omni_robot_description.geometry import default_geometry_file


def generate_launch_description():
    share = get_package_share_directory("omni_robot_mission")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_joy", default_value="true"),
        DeclareLaunchArgument("require_joy", default_value="true"),
        DeclareLaunchArgument(
            "mission_params",
            default_value=os.path.join(share, "config", "sakura_mission.yaml")),
        DeclareLaunchArgument(
            "geometry_file", default_value=default_geometry_file()),
        Node(
            package="omni_robot_mission",
            executable="mode_manager",
            name="mode_manager",
            output="screen",
            parameters=[
                os.path.join(share, "config", "mode_manager.yaml"),
                {
                    "use_sim_time": use_sim_time,
                    "require_joy": ParameterValue(
                        LaunchConfiguration("require_joy"), value_type=bool),
                },
            ],
        ),
        Node(
            package="omni_robot_mission",
            executable="sakura_mission",
            name="sakura_mission",
            output="screen",
            parameters=[
                LaunchConfiguration("mission_params"),
                {
                    "use_sim_time": use_sim_time,
                    "geometry_file": LaunchConfiguration("geometry_file"),
                },
            ],
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("use_joy")),
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            output="screen",
            parameters=[
                os.path.join(share, "config", "teleop_ps4.yaml"),
                {"use_sim_time": use_sim_time},
            ],
            remappings=[("cmd_vel", "/cmd_vel_manual")],
            condition=IfCondition(LaunchConfiguration("use_joy")),
        ),
    ])
