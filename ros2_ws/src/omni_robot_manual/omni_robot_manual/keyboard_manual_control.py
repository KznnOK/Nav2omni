import json
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .control_math import approach, limit_vector


class KeyboardManualControl(Node):
    def __init__(self):
        super().__init__("keyboard_manual_control")
        self.declare_parameter("output_topic", "/cmd_vel_manual")
        self.declare_parameter("event_topic", "/manual/events")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("key_timeout_s", 0.25)
        self.declare_parameter("normal_max_linear", 0.25)
        self.declare_parameter("normal_max_angular", 0.50)
        self.declare_parameter("precision_max_linear", 0.08)
        self.declare_parameter("precision_max_angular", 0.20)
        self.declare_parameter("high_max_linear", 0.35)
        self.declare_parameter("high_max_angular", 0.70)
        self.declare_parameter("accel_limit_linear", 0.60)
        self.declare_parameter("accel_limit_angular", 1.00)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10)
        self.events = self.create_publisher(
            String, str(self.get_parameter("event_topic").value), 10)
        self.keys = {}
        self.last_input = time.monotonic()
        self.current = Twist()
        self.running = True
        self.old_settings = None
        if not sys.stdin.isatty():
            self.get_logger().error(
                "Keyboard input requires a TTY. Start this node in an interactive "
                "terminal; no velocity commands will be published.")
            raise RuntimeError("stdin is not a TTY")
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.get_logger().info(
            "Keys: W/S forward/back, A/D strafe, Q/E rotate, "
            "Shift=high speed, C=precision, Space=emergency stop, "
            "Enter=start/stop, P=pickup, O=drop")
        self.get_logger().info(
            f"Output: {self.get_parameter('output_topic').value} | "
            f"max linear: normal={self.get_parameter('normal_max_linear').value:.2f}, "
            f"precision={self.get_parameter('precision_max_linear').value:.2f}, "
            f"high={self.get_parameter('high_max_linear').value:.2f} m/s | "
            f"max angular: normal={self.get_parameter('normal_max_angular').value:.2f}, "
            f"precision={self.get_parameter('precision_max_angular').value:.2f}, "
            f"high={self.get_parameter('high_max_angular').value:.2f} rad/s")
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.update)

    def emit(self, event):
        message = String()
        message.data = json.dumps({"event": event})
        self.events.publish(message)

    def read_keys(self):
        if not sys.stdin.isatty():
            return
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)
            self.last_input = time.monotonic()
            if key == "\x03":
                raise KeyboardInterrupt
            if key == " ":
                self.keys.clear()
                self.current = Twist()
                self.emit("emergency_stop")
            elif key == "\n" or key == "\r":
                self.emit("start_toggle")
            elif key.lower() == "p":
                self.emit("pickup")
            elif key.lower() == "o":
                self.emit("drop")
            elif key in ("w", "s", "a", "d", "q", "e"):
                self.keys[key] = True
            elif key in ("W", "S", "A", "D", "Q", "E"):
                self.keys[key.lower()] = True
                self.keys["high_speed"] = True
            elif key.lower() == "c":
                self.keys["c"] = True
            elif key == "\x7f":
                self.keys.clear()
        if time.monotonic() - self.last_input > float(
                self.get_parameter("key_timeout_s").value):
            self.keys.clear()

    def target(self):
        linear = float(self.get_parameter("normal_max_linear").value)
        angular = float(self.get_parameter("normal_max_angular").value)
        if any(self.keys.get(key, False) for key in ("w", "s", "a", "d")):
            if self.keys.get("w", False) or self.keys.get("s", False):
                linear = float(self.get_parameter("high_max_linear").value)
                angular = float(self.get_parameter("high_max_angular").value)
        if self.keys.get("c", False):
            linear = float(self.get_parameter("precision_max_linear").value)
            angular = float(self.get_parameter("precision_max_angular").value)
        elif self.keys.get("high_speed", False):
            linear = float(self.get_parameter("high_max_linear").value)
            angular = float(self.get_parameter("high_max_angular").value)
        x = linear * (int("w" in self.keys) - int("s" in self.keys))
        y = linear * (int("a" in self.keys) - int("d" in self.keys))
        z = angular * (int("q" in self.keys) - int("e" in self.keys))
        x, y = limit_vector(x, y, linear)
        result = Twist()
        result.linear.x = x
        result.linear.y = y
        result.angular.z = z
        return result

    def update(self):
        self.read_keys()
        target = self.target()
        dt = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        linear_step = float(self.get_parameter("accel_limit_linear").value) * dt
        angular_step = float(self.get_parameter("accel_limit_angular").value) * dt
        self.current.linear.x = approach(self.current.linear.x, target.linear.x, linear_step)
        self.current.linear.y = approach(self.current.linear.y, target.linear.y, linear_step)
        self.current.angular.z = approach(self.current.angular.z, target.angular.z, angular_step)
        self.publisher.publish(self.current)

    def stop(self):
        self.publisher.publish(Twist())
        if self.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = KeyboardManualControl()
    except RuntimeError as error:
        rclpy.logging.get_logger("keyboard_manual_control").error(str(error))
        if rclpy.ok():
            rclpy.shutdown()
        return
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
