import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ManualCmdMux(Node):
    """Select one command source and always publish an explicit safe output."""

    def __init__(self):
        super().__init__("manual_cmd_mux")
        self.declare_parameter("mode", "manual")
        self.declare_parameter("manual_topic", "/cmd_vel_manual")
        self.declare_parameter("automatic_topic", "/cmd_vel_auto")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("input_timeout_s", 0.30)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.mode = str(self.get_parameter("mode").value)
        self.timeout = float(self.get_parameter("input_timeout_s").value)
        manual_topic = str(self.get_parameter("manual_topic").value)
        auto_topic = str(self.get_parameter("automatic_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.manual = Twist()
        self.automatic = Twist()
        self.manual_time = None
        self.auto_time = None
        self.last_mode = self.mode
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, manual_topic, self.manual_callback, 10)
        self.create_subscription(Twist, auto_topic, self.auto_callback, 10)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.publish_selected)

    def valid(self, message):
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        return all(math.isfinite(value) for value in values)

    def manual_callback(self, message):
        if self.valid(message):
            self.manual = message
            self.manual_time = self.get_clock().now()

    def auto_callback(self, message):
        if self.valid(message):
            self.automatic = message
            self.auto_time = self.get_clock().now()

    def fresh(self, stamp):
        return (
            stamp is not None
            and (self.get_clock().now() - stamp).nanoseconds / 1e9
            <= self.timeout
        )

    def publish_selected(self):
        mode = str(self.get_parameter("mode").value)
        if mode != self.last_mode:
            self.last_mode = mode
            self.manual = Twist()
            self.automatic = Twist()
            self.manual_time = None
            self.auto_time = None
        if mode == "manual" and self.fresh(self.manual_time):
            output = self.manual
        elif mode == "automatic" and self.fresh(self.auto_time):
            output = self.automatic
        else:
            output = Twist()
        if self.valid(output):
            self.publisher.publish(output)
        else:
            self.publisher.publish(Twist())

    def stop(self):
        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ManualCmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
