import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovariance(Node):
    """Attach explicit covariances to the noisy Gazebo IMU message.

    Simulation only: optionally adds a gyro z bias and a bias random walk
    (sim_conditions.yaml imu_error), which the robot software does not know.
    """

    def __init__(self):
        super().__init__("imu_covariance")
        self.bias = math.radians(
            self.declare_parameter("gyro_bias_z_deg_s", 0.0).value)
        self.random_walk = math.radians(self.declare_parameter(
            "gyro_bias_random_walk_deg_s_sqrt_s", 0.0).value)
        self.rng = random.Random(self.declare_parameter("seed", 1).value)
        self.last_time = None
        self.publisher = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.subscription = self.create_subscription(
            Imu, "/imu/data_raw_gazebo", self.callback, 10)
        if self.bias or self.random_walk:
            self.get_logger().info(
                f"simulated gyro z bias {math.degrees(self.bias):.3f} deg/s, "
                f"random walk {math.degrees(self.random_walk):.4f} deg/s/sqrt(s)")

    def callback(self, message):
        t = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.random_walk and self.last_time is not None and t > self.last_time:
            self.bias += self.random_walk * math.sqrt(t - self.last_time) \
                * self.rng.gauss(0.0, 1.0)
        self.last_time = t
        message.angular_velocity.z += self.bias
        message.orientation_covariance[0] = -1.0
        message.angular_velocity_covariance[0] = 1.0e-6
        message.angular_velocity_covariance[4] = 1.0e-6
        message.angular_velocity_covariance[8] = 4.0e-6
        message.linear_acceleration_covariance[0] = 1.0e-4
        message.linear_acceleration_covariance[4] = 1.0e-4
        message.linear_acceleration_covariance[8] = 1.0e-4
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovariance()
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
