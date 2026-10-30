import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from omni_robot_description.geometry import (
    drive_wheel_layout, drive_wheel_radius, load_geometry, default_geometry_file)


class OmniOdometry(Node):
    """Estimate planar odometry from four omni-wheel joint velocities."""

    def __init__(self):
        super().__init__("omni_odometry")

        # Defaults come from robot_geometry.yaml
        geometry = load_geometry(
            self.declare_parameter(
                "geometry_file", default_geometry_file()).value)
        joint_names, wheel_x, wheel_y, wheel_yaw = drive_wheel_layout(geometry)
        self.declare_parameter("wheel_radius", drive_wheel_radius(geometry))
        self.declare_parameter("wheel_joint_names", joint_names)
        self.declare_parameter("wheel_x", wheel_x)
        self.declare_parameter("wheel_y", wheel_y)
        self.declare_parameter("wheel_direction_yaw", wheel_yaw)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("odom_topic", "/wheel/odom")

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.joint_names = list(
            self.get_parameter("wheel_joint_names").value
        )

        # Each row maps [vx, vy, wz] to wheel tangential velocity.
        wheel_geometry = list(zip(
            self.get_parameter("wheel_x").value,
            self.get_parameter("wheel_y").value,
            self.get_parameter("wheel_direction_yaw").value,
        ))
        if len(self.joint_names) != 4 or len(wheel_geometry) != 4:
            raise ValueError("wheel geometry must contain exactly four wheels")
        self.kinematic_matrix = np.array([
            [
                math.cos(yaw),
                math.sin(yaw),
                -math.cos(yaw) * y_pos + math.sin(yaw) * x_pos,
            ]
            for x_pos, y_pos, yaw in wheel_geometry
        ], dtype=float)
        self.inverse_kinematics = np.linalg.pinv(self.kinematic_matrix)

        self.wheel_velocities = np.zeros(4, dtype=float)
        self.have_joint_state = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self.joint_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            20,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.update)

        self.get_logger().info(
            f"Omni odometry started: /joint_states -> "
            f"{self.get_parameter('odom_topic').value} (tf={self.publish_tf})"
        )

    def joint_state_callback(self, msg: JointState):
        velocity_by_name = {
            name: velocity
            for name, velocity in zip(msg.name, msg.velocity)
        }

        if not all(name in velocity_by_name for name in self.joint_names):
            return

        self.wheel_velocities = np.array([
            velocity_by_name[name] for name in self.joint_names
        ], dtype=float)
        self.have_joint_state = True

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1.0e-9
        self.last_time = now

        if not self.have_joint_state or dt <= 0.0 or dt > 0.5:
            return

        wheel_tangential_velocities = self.wheel_radius * self.wheel_velocities
        vx, vy, wz = self.inverse_kinematics @ wheel_tangential_velocities

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (cos_yaw * vx - sin_yaw * vy) * dt
        self.y += (sin_yaw * vx + cos_yaw * vy) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + wz * dt),
            math.cos(self.yaw + wz * dt),
        )

        half_yaw = 0.5 * self.yaw
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.angular.z = float(wz)
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = float(self.x)
        transform.transform.translation.y = float(self.y)
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OmniOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
