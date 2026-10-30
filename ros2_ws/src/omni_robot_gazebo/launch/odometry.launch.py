from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import yaml


def sensor_nodes(context):
    """IMU covariance (+ simulated gyro errors) and the tracking odometry.

    sim_conditions.yaml imu_error / tracking_wheel_error (quantize,
    lift_rate_hz) are applied by simulation-only nodes; tracking_odometry
    then reads /tracking/joint_states instead of /joint_states.
    """
    with open(LaunchConfiguration("sim_conditions").perform(context)) as f:
        conditions = yaml.safe_load(f) or {}
    imu = conditions.get("imu_error", {}) or {}
    tracking = conditions.get("tracking_wheel_error", {}) or {}
    geometry = {"geometry_file": LaunchConfiguration("geometry_file")}
    nodes = [Node(
        package="omni_robot_gazebo",
        executable="imu_covariance",
        name="imu_covariance",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "gyro_bias_z_deg_s": float(imu.get("gyro_bias_z_deg_s", 0.0)),
            "gyro_bias_random_walk_deg_s_sqrt_s": float(
                imu.get("gyro_bias_random_walk_deg_s_sqrt_s", 0.0)),
            "seed": int(imu.get("seed", 1)),
        }],
    )]
    encoder_errors = (bool(tracking.get("quantize", False))
                      or float(tracking.get("lift_rate_hz", 0.0)) > 0.0)
    joint_states = "/joint_states"
    if encoder_errors:
        joint_states = "/tracking/joint_states"
        nodes.append(Node(
            package="omni_robot_gazebo",
            executable="tracking_encoder_sim",
            name="tracking_encoder_sim",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "quantize": bool(tracking.get("quantize", False)),
                "lift_rate_hz": float(tracking.get("lift_rate_hz", 0.0)),
                "lift_duration_s": float(tracking.get("lift_duration_s", 0.02)),
                "seed": int(tracking.get("seed", 1)),
            }, geometry],
        ))
    # Tracking wheels + IMU yaw rate: the odometry fused by the EKF.
    nodes.append(Node(
        package="omni_robot_odometry",
        executable="tracking_odometry",
        name="tracking_odometry",
        output="screen",
        parameters=[{"use_sim_time": True,
                     "joint_states_topic": joint_states}, geometry],
    ))
    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("omni_robot_gazebo")
    ekf_file = os.path.join(package_share, "config", "ekf.yaml")
    geometry = {"geometry_file": LaunchConfiguration("geometry_file")}
    return LaunchDescription([
        DeclareLaunchArgument(
            "geometry_file",
            default_value=os.path.join(
                get_package_share_directory("omni_robot_description"),
                "config", "robot_geometry.yaml"),
        ),
        DeclareLaunchArgument(
            "sim_conditions",
            default_value=os.path.join(
                package_share, "config", "sim_conditions.yaml"),
        ),
        OpaqueFunction(function=sensor_nodes),
        Node(
            package="omni_robot_gazebo",
            executable="omni_odometry",
            name="omni_odometry",
            output="screen",
            parameters=[{"use_sim_time": True}, geometry],
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
