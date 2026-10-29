import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


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
        gazebo_share,
        "config",
        "wheel_geometry.yaml",
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
            "-3.1510900301327864",
            "-y",
            "-3.136311120883107",
            "-z",
            "0.25",
            "-Y",
            "0.0",
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
            geometry_file,
        ],
    )

    return LaunchDescription([
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
