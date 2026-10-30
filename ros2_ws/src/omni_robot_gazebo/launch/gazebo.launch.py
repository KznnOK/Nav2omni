import os
import subprocess
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def spawn_with_sim_errors(context, xacro_file, controllers_file):
    """Spawn the Gazebo model with the hardware errors of sim_conditions.yaml
    (LiDAR and tracking wheel mounting, tracking wheel diameter, drive wheel
    friction).

    robot_state_publisher keeps the nominal description, so the errors stay
    unknown to the robot software as on real hardware.
    """
    with open(LaunchConfiguration("sim_conditions").perform(context)) as f:
        conditions = yaml.safe_load(f) or {}
    error = conditions.get("lidar_mount_error", {})
    args = [
        "xacro", xacro_file,
        f"controllers_file:={controllers_file}",
        "geometry_file:="
        + LaunchConfiguration("geometry_file").perform(context),
    ]
    for key in ("x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg"):
        args.append(f"lidar_err_{key}:={float(error.get(key, 0.0))}")
    tracking = conditions.get("tracking_wheel_error", {}) or {}
    args.append("tracking_diameter_scale:="
                f"{float(tracking.get('diameter_scale', 1.0))}")
    for index, wheel in enumerate(tracking.get("wheels", []) or []):
        args.append(f"tracking_err_x_{index}:={float(wheel.get('dx', 0.0))}")
        args.append(f"tracking_err_y_{index}:={float(wheel.get('dy', 0.0))}")
        args.append(f"tracking_err_angle_deg_{index}:="
                    f"{float(wheel.get('angle_deg', 0.0))}")
    drive = conditions.get("drive_wheel_error", {}) or {}
    args.append("drive_mu_rolling_scale:="
                f"{float(drive.get('mu_rolling_scale', 1.0))}")
    urdf = subprocess.run(args, check=True, capture_output=True,
                          text=True).stdout
    sim_urdf = tempfile.NamedTemporaryFile(
        "w", prefix="omni_robot_sim_", suffix=".urdf", delete=False)
    sim_urdf.write(urdf)
    sim_urdf.close()

    return [Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "omni_robot",
            "-file",
            sim_urdf.name,
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
            "-Y",
            LaunchConfiguration("spawn_yaw"),
        ],
    )]


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
    geometry_file = LaunchConfiguration("geometry_file")

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

    spawn_robot = OpaqueFunction(
        function=spawn_with_sim_errors,
        args=[xacro_file, controllers_file],
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
                "geometry_file": geometry_file,
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
        DeclareLaunchArgument(
            "geometry_file",
            default_value=os.path.join(
                description_share, "config", "robot_geometry.yaml"),
        ),
        DeclareLaunchArgument(
            "sim_conditions",
            default_value=os.path.join(
                gazebo_share, "config", "sim_conditions.yaml"),
        ),
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
