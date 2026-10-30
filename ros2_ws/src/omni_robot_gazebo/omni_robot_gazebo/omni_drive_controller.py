import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from omni_robot_description.geometry import (
    drive_wheel_layout, drive_wheel_radius, load_geometry, default_geometry_file)


class OmniDriveController(Node):
    """Convert /cmd_vel into four wheel angular velocity commands."""

    def __init__(self):
        super().__init__("omni_drive_controller")

        # Robot parameters default to robot_geometry.yaml
        geometry = load_geometry(
            self.declare_parameter(
                "geometry_file", default_geometry_file()).value)
        _, wheel_x, wheel_y, wheel_yaw = drive_wheel_layout(geometry)
        self.declare_parameter("wheel_radius", drive_wheel_radius(geometry))
        self.declare_parameter("wheel_x", wheel_x)
        self.declare_parameter("wheel_y", wheel_y)
        self.declare_parameter("wheel_direction_yaw", wheel_yaw)
        self.declare_parameter(
            "max_wheel_speed",
            float(geometry["drive_wheels"]["max_wheel_speed"]),
        )
        self.declare_parameter("command_timeout", 0.5)

        self.wheel_radius = float(
            self.get_parameter("wheel_radius").value
        )

        self.max_wheel_speed = float(
            self.get_parameter("max_wheel_speed").value
        )

        self.command_timeout = float(
            self.get_parameter("command_timeout").value
        )

        # Each tuple:
        # (wheel x position, wheel y position, rolling direction yaw)
        #
        # Wheel order must match omni_controllers.yaml:
        # 1. front_left
        # 2. front_right
        # 3. rear_left
        # 4. rear_right
        wheel_x = list(self.get_parameter("wheel_x").value)
        wheel_y = list(self.get_parameter("wheel_y").value)
        wheel_yaw = list(self.get_parameter("wheel_direction_yaw").value)
        if not (len(wheel_x) == len(wheel_y) == len(wheel_yaw) == 4):
            raise ValueError("wheel geometry must contain exactly four wheels")
        self.wheels = list(zip(wheel_x, wheel_y, wheel_yaw))

        self.wheel_command_publisher = self.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10,
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.last_command_time = self.get_clock().now()
        self.robot_is_stopped = True

        self.timeout_timer = self.create_timer(
            0.05,
            self.command_timeout_callback,
        )

        self.get_logger().info(
            "Omni drive controller started"
        )

        self.get_logger().info(
            "Input: /cmd_vel"
        )

        self.get_logger().info(
            "Output: /wheel_velocity_controller/commands"
        )

    def cmd_vel_callback(self, msg):
        """Convert robot velocity into wheel angular velocities."""

        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        wheel_angular_velocities = []

        for wheel_x, wheel_y, wheel_yaw in self.wheels:
            rolling_x = math.cos(wheel_yaw)
            rolling_y = math.sin(wheel_yaw)

            linear_velocity = (
                rolling_x * linear_x
                + rolling_y * linear_y
            )

            rotational_velocity = angular_z * (
                -rolling_x * wheel_y
                + rolling_y * wheel_x
            )

            wheel_angular_velocity = (
                linear_velocity + rotational_velocity
            ) / self.wheel_radius

            wheel_angular_velocity = max(
                -self.max_wheel_speed,
                min(
                    self.max_wheel_speed,
                    wheel_angular_velocity,
                ),
            )

            wheel_angular_velocities.append(
                wheel_angular_velocity
            )

        command_message = Float64MultiArray()
        command_message.data = wheel_angular_velocities

        self.wheel_command_publisher.publish(
            command_message
        )

        self.last_command_time = self.get_clock().now()
        self.robot_is_stopped = False

    def command_timeout_callback(self):
        """Stop all wheels when /cmd_vel stops arriving."""

        elapsed_time = (
            self.get_clock().now()
            - self.last_command_time
        ).nanoseconds / 1.0e9

        if (
            elapsed_time > self.command_timeout
            and not self.robot_is_stopped
        ):
            self.publish_stop_command()
            self.robot_is_stopped = True

    def publish_stop_command(self):
        """Publish zero velocity to all four wheels."""

        stop_message = Float64MultiArray()
        stop_message.data = [
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        self.wheel_command_publisher.publish(
            stop_message
        )


def main(args=None):
    rclpy.init(args=args)

    node = OmniDriveController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_stop_command()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
