import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from .control_math import apply_deadzone, approach, limit_vector, shape


class JoyManualControl(Node):
    def __init__(self):
        super().__init__("joy_manual_control")
        names = (
            "joy_topic", "output_topic", "event_topic", "diagnostic",
            "joy_timeout_s", "publish_rate_hz", "deadzone", "input_curve",
            "deadman_button", "emergency_stop_button", "start_button",
            "pickup_button", "drop_button", "precision_button",
            "high_speed_button", "axis_forward", "axis_lateral",
            "axis_rotate", "invert_forward", "invert_lateral",
            "invert_rotate", "normal_max_linear", "normal_max_angular",
            "precision_max_linear", "precision_max_angular",
            "high_max_linear", "high_max_angular", "accel_limit_linear",
            "accel_limit_angular",
        )
        defaults = {
            "joy_topic": "/joy", "output_topic": "/cmd_vel_manual",
            "event_topic": "/manual/events", "diagnostic": False,
            "joy_timeout_s": 0.3, "publish_rate_hz": 20.0, "deadzone": 0.12,
            "input_curve": 1.0, "deadman_button": 0,
            "emergency_stop_button": 1, "start_button": 9, "pickup_button": 3,
            "drop_button": 2, "precision_button": 4, "high_speed_button": 5,
            "axis_forward": 1, "axis_lateral": 0, "axis_rotate": 2,
            "invert_forward": True, "invert_lateral": False,
            "invert_rotate": False, "normal_max_linear": 0.25,
            "normal_max_angular": 0.50, "precision_max_linear": 0.08,
            "precision_max_angular": 0.20, "high_max_linear": 0.35,
            "high_max_angular": 0.70, "accel_limit_linear": 0.60,
            "accel_limit_angular": 1.00,
        }
        for name in names:
            self.declare_parameter(name, defaults[name])
        self.timeout = float(self.get_parameter("joy_timeout_s").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.curve = float(self.get_parameter("input_curve").value)
        self.last_joy = None
        self.last_time = None
        self.last_buttons = []
        self.emergency = False
        self.current = Twist()
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10)
        self.events = self.create_publisher(
            String, str(self.get_parameter("event_topic").value), 10)
        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self.joy_callback, 10)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.publish_command)

    def button(self, buttons, name):
        index = int(self.get_parameter(name).value)
        return 0 <= index < len(buttons) and buttons[index] == 1

    def axis(self, axes, name, invert_name):
        index = int(self.get_parameter(name).value)
        value = axes[index] if 0 <= index < len(axes) else 0.0
        if not math.isfinite(value):
            return float("nan")
        value = apply_deadzone(value, self.deadzone)
        value = shape(value, self.curve)
        return -value if bool(self.get_parameter(invert_name).value) else value

    def emit(self, event, **values):
        message = String()
        message.data = json.dumps({"event": event, **values}, sort_keys=True)
        self.events.publish(message)

    def joy_callback(self, message):
        self.last_joy = message
        self.last_time = self.get_clock().now()
        buttons = list(message.buttons)
        for name, event in (
            ("start_button", "start_toggle"),
            ("pickup_button", "pickup"),
            ("drop_button", "drop"),
            ("emergency_stop_button", "emergency_stop"),
        ):
            if self.button(buttons, name) and not self.button(self.last_buttons, name):
                if event == "emergency_stop":
                    self.emergency = True
                self.emit(event)
        if self.button(buttons, "emergency_stop_button") is False:
            self.emergency = False
        if self.button(buttons, "precision_button") != self.button(
                self.last_buttons, "precision_button"):
            self.emit("mode_change")
        if self.button(buttons, "high_speed_button") != self.button(
                self.last_buttons, "high_speed_button"):
            self.emit("mode_change")
        self.last_buttons = buttons
        if bool(self.get_parameter("diagnostic").value):
            self.get_logger().info(
                "Joy axes=%s buttons=%s", message.axes, message.buttons)

    def target(self):
        if self.last_joy is None or self.last_time is None:
            return Twist()
        age = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        if age > self.timeout or self.emergency:
            return Twist()
        if not self.button(self.last_joy.buttons, "deadman_button"):
            return Twist()
        x = self.axis(self.last_joy.axes, "axis_forward", "invert_forward")
        y = self.axis(self.last_joy.axes, "axis_lateral", "invert_lateral")
        z = self.axis(self.last_joy.axes, "axis_rotate", "invert_rotate")
        if not all(math.isfinite(value) for value in (x, y, z)):
            return Twist()
        if self.button(self.last_joy.buttons, "precision_button"):
            linear = float(self.get_parameter("precision_max_linear").value)
            angular = float(self.get_parameter("precision_max_angular").value)
        elif self.button(self.last_joy.buttons, "high_speed_button"):
            linear = float(self.get_parameter("high_max_linear").value)
            angular = float(self.get_parameter("high_max_angular").value)
        else:
            linear = float(self.get_parameter("normal_max_linear").value)
            angular = float(self.get_parameter("normal_max_angular").value)
        x, y = limit_vector(x * linear, y * linear, linear)
        result = Twist()
        result.linear.x = x
        result.linear.y = y
        result.angular.z = z * angular
        return result

    def publish_command(self):
        target = self.target()
        now = self.get_clock().now()
        dt = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        linear_step = float(self.get_parameter("accel_limit_linear").value) * dt
        angular_step = float(self.get_parameter("accel_limit_angular").value) * dt
        self.current.linear.x = approach(self.current.linear.x, target.linear.x, linear_step)
        self.current.linear.y = approach(self.current.linear.y, target.linear.y, linear_step)
        self.current.angular.z = approach(self.current.angular.z, target.angular.z, angular_step)
        if self.last_time is None or (
            (now - self.last_time).nanoseconds / 1e9 > self.timeout
        ) or self.emergency:
            self.current = Twist()
        if not bool(self.get_parameter("diagnostic").value):
            self.publisher.publish(self.current)

    def stop(self):
        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = JoyManualControl()
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
