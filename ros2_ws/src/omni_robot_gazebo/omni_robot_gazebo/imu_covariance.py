import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovariance(Node):
    """Attach explicit covariances to the noisy Gazebo IMU message."""

    def __init__(self):
        super().__init__("imu_covariance")
        self.publisher = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.subscription = self.create_subscription(
            Imu, "/imu/data_raw_gazebo", self.callback, 10)

    def callback(self, message):
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
