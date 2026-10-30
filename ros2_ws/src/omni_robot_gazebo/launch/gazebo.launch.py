import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    description_share = get_package_share_directory(
        "omni_robot_description"
    )

    gazebo_share = get_package_share_directory(
        "omni_robot_gazebo"
    )

    gazebo_ros_share = get_package_share_directory(
        "gazebo_ros"
    )

    xacro_file = os.path.join(
        description_share,
        "urdf",
        "omni_robot.urdf.xacro"
    )

    world_file = os.path.join(
        gazebo_share,
        "worlds",
        "kansai_spring_robocon_2027.world"
    )

    controllers_file = os.path.join(
        gazebo_share,
        "config",
        "omni_controllers.yaml"
    )
    geometry_file = os.path.join(
        description_share,
        "config",
        "robot_geometry.yaml",
    )

    gazebo_params_file = os.path.join(
        gazebo_share,
        "config",
        "gazebo_ros.yaml",
    )

    model_path = os.path.join(
        gazebo_share,
        "models"
    )

    old_model_path = os.environ.get(
        "GAZEBO_MODEL_PATH",
        ""
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file,
            " controllers_file:=",
            controllers_file,
            " geometry_file:=",
            geometry_file,
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_share,
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "true",
            "params_file": gazebo_params_file,
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "omni_robot",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
            "-Y",
            LaunchConfiguration("spawn_yaw"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    omni_drive_controller = Node(
        package="omni_robot_gazebo",
        executable="omni_drive_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            },
        ],
    )

    return LaunchDescription([
        # Default spawn pose: the original start zone pose.
        DeclareLaunchArgument("spawn_x", default_value="-3.1510900301327864"),
        DeclareLaunchArgument("spawn_y", default_value="-3.136311120883107"),
        DeclareLaunchArgument("spawn_z", default_value="0.02"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("gui", default_value="true"),
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            model_path
            + (
                os.pathsep + old_model_path
                if old_model_path
                else ""
            ),
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        omni_drive_controller,
    ])
