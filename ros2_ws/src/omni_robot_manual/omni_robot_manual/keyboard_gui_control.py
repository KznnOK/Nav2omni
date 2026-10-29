import time

import pygame
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .control_math import approach, limit_vector


def command_from_keys(
        keys, normal_linear, normal_angular, precision_linear,
        precision_angular, high_linear, high_angular):
    """Recompute every Twist component from the complete key state."""
    x_input = int(keys.get("forward", False)) - int(
        keys.get("backward", False))
    y_input = int(keys.get("left", False)) - int(keys.get("right", False))
    yaw_input = int(keys.get("rotate_left", False)) - int(
        keys.get("rotate_right", False))
    if keys.get("precision", False):
        linear_limit, angular_limit = precision_linear, precision_angular
    elif keys.get("high_speed", False):
        linear_limit, angular_limit = high_linear, high_angular
    else:
        linear_limit, angular_limit = normal_linear, normal_angular
    x, y = limit_vector(
        x_input * linear_limit, y_input * linear_limit, linear_limit)
    command = Twist()
    command.linear.x = x
    command.linear.y = y
    command.angular.z = yaw_input * angular_limit
    if keys.get("emergency_stop", False):
        return Twist()
    return command


class KeyboardGuiControl(Node):
    def __init__(self):
        super().__init__("keyboard_gui_control")
        defaults = {
            "output_topic": "/cmd_vel_manual",
            "publish_rate_hz": 50.0,
            "input_timeout_s": 0.30,
            "normal_max_linear": 0.40,
            "normal_max_angular": 0.90,
            "precision_max_linear": 0.15,
            "precision_max_angular": 0.35,
            "high_max_linear": 0.60,
            "high_max_angular": 1.20,
            "linear_acceleration": 0.8,
            "linear_deceleration": 1.2,
            "angular_acceleration": 2.0,
            "angular_deceleration": 3.0,
            "window_width": 640,
            "window_height": 360,
            "window_title": "Omni Robot Manual Control",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10)
        self.keys = {
            "forward": False, "backward": False, "left": False,
            "right": False, "rotate_left": False, "rotate_right": False,
            "precision": False, "high_speed": False,
        }
        self.current = Twist()
        self.last_event_time = time.monotonic()
        self.closed = False
        self.window_focused = True
        self.emergency_latched = False
        self.emergency_reason = None
        self.last_status = None
        self.last_status_log = 0.0
        pygame.init()
        self.window = pygame.display.set_mode((
            int(self.get_parameter("window_width").value),
            int(self.get_parameter("window_height").value)))
        pygame.display.set_caption(
            str(self.get_parameter("window_title").value))
        pygame.key.set_repeat(50, 50)
        self.font = pygame.font.Font(None, 28)
        self.keycodes = self._keycodes()
        # SDL scancodes correspond to physical US-key positions.
        self.scancodes = {
            "forward": 26, "backward": 22, "left": 4, "right": 7,
            "rotate_left": 20, "rotate_right": 8, "precision": 224,
            "high_speed": 225, "emergency_stop": 44,
        }
        self.get_logger().info(
            "GUI keys: W/S forward, A/D strafe, Q/E rotate, "
            "Shift=high speed, Ctrl=precision, Space=emergency stop")
        self.get_logger().info(
            f"Output: {self.get_parameter('output_topic').value} | "
            f"max linear: normal={self.get_parameter('normal_max_linear').value:.2f}, "
            f"precision={self.get_parameter('precision_max_linear').value:.2f}, "
            f"high={self.get_parameter('high_max_linear').value:.2f} m/s | "
            f"max angular: normal={self.get_parameter('normal_max_angular').value:.2f}, "
            f"precision={self.get_parameter('precision_max_angular').value:.2f}, "
            f"high={self.get_parameter('high_max_angular').value:.2f} rad/s")
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value),
            self.update)

    def _keycodes(self):
        return {
            "forward": pygame.K_w, "backward": pygame.K_s,
            "left": pygame.K_a, "right": pygame.K_d,
            "rotate_left": pygame.K_q, "rotate_right": pygame.K_e,
            "precision": pygame.K_LCTRL, "high_speed": pygame.K_LSHIFT,
            "emergency_stop": pygame.K_SPACE,
        }

    def _set_key(self, keycode, scancode, pressed):
        for name, code in self.keycodes.items():
            if keycode == code or scancode == self.scancodes[name]:
                if name == "emergency_stop":
                    if pressed:
                        self.emergency_latched = True
                else:
                    self.keys[name] = pressed

    def _clear_motion_keys(self):
        for name in self.keys:
            self.keys[name] = False

    def _zero(self):
        self.current = Twist()
        self.publisher.publish(Twist())

    def _activate_emergency(self, reason):
        self.emergency_latched = True
        if self.emergency_reason != reason:
            self.emergency_reason = reason
            self.get_logger().warn(f"Emergency stop activated: {reason}")
        self._zero()

    def _handle_events(self):
        for event in pygame.event.get():
            self.last_event_time = time.monotonic()
            if event.type == pygame.QUIT:
                self.closed = True
                self._activate_emergency("window closed")
            elif event.type == pygame.KEYDOWN:
                self._set_key(
                    event.key, getattr(event, "scancode", -1), True)
                if event.key == self.keycodes["emergency_stop"]:
                    self._activate_emergency("Space pressed")
                elif self.emergency_latched:
                    self.emergency_latched = False
                    self.emergency_reason = None
            elif event.type == pygame.KEYUP:
                self._set_key(
                    event.key, getattr(event, "scancode", -1), False)
            elif event.type == pygame.WINDOWFOCUSLOST:
                self.window_focused = False
                self._clear_motion_keys()
                self._activate_emergency("window focus lost")
            elif event.type == pygame.WINDOWFOCUSGAINED:
                self.window_focused = True
                self._clear_motion_keys()
                self.emergency_latched = True
                self.last_event_time = time.monotonic()

    def _mode_limits(self):
        if self.keys["precision"]:
            return (float(self.get_parameter("precision_max_linear").value),
                    float(self.get_parameter("precision_max_angular").value),
                    "PRECISION")
        if self.keys["high_speed"]:
            return (float(self.get_parameter("high_max_linear").value),
                    float(self.get_parameter("high_max_angular").value), "HIGH")
        return (float(self.get_parameter("normal_max_linear").value),
                float(self.get_parameter("normal_max_angular").value), "NORMAL")

    def _slew(self, current, target, dt, acceleration, deceleration):
        rate = acceleration if abs(target) > abs(current) else deceleration
        return approach(current, target, rate * dt)

    def _draw_status(self, command, mode, emergency):
        self.window.fill((24, 28, 36))
        pressed = " ".join(
            name for name in (
                "forward", "backward", "left", "right",
                "rotate_left", "rotate_right") if self.keys[name]) or "-"
        speed = (command.linear.x ** 2 + command.linear.y ** 2) ** 0.5
        lines = (
            f"Pressed: {pressed}",
            f"Mode: {mode}  Focus: {self.window_focused}",
            f"linear.x: {command.linear.x:+.3f}  "
            f"linear.y: {command.linear.y:+.3f}",
            f"angular.z: {command.angular.z:+.3f}  speed: {speed:.3f} m/s",
            f"Emergency stop: {emergency}",
            "W/S A/D Q/E | Shift high | Ctrl precision | Space stop",
        )
        for index, line in enumerate(lines):
            self.window.blit(
                self.font.render(line, True, (235, 235, 235)),
                (18, 18 + index * 35))
        pygame.display.flip()

    def update(self):
        self._handle_events()
        if self.closed:
            for _ in range(5):
                self._zero()
            rclpy.shutdown()
            return
        physical_keys = pygame.key.get_pressed()
        physical_input_active = any(
            physical_keys[code] for code in self.keycodes.values()
            if code != self.keycodes["emergency_stop"])
        timed_out = (
            time.monotonic() - self.last_event_time
            > float(self.get_parameter("input_timeout_s").value)
            and not physical_input_active
        )
        if not self.window_focused or timed_out:
            self._clear_motion_keys()
            emergency = True
            if timed_out:
                self._activate_emergency("input timeout")
            elif not self.emergency_latched:
                self._activate_emergency("window focus lost")
        else:
            emergency = self.emergency_latched
        linear_limit, angular_limit, mode = self._mode_limits()
        target = command_from_keys(
            self.keys, linear_limit, angular_limit,
            linear_limit, angular_limit, linear_limit, angular_limit)
        dt = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.current.linear.x = self._slew(
            self.current.linear.x, target.linear.x, dt,
            float(self.get_parameter("linear_acceleration").value),
            float(self.get_parameter("linear_deceleration").value))
        self.current.linear.y = self._slew(
            self.current.linear.y, target.linear.y, dt,
            float(self.get_parameter("linear_acceleration").value),
            float(self.get_parameter("linear_deceleration").value))
        self.current.angular.z = self._slew(
            self.current.angular.z, target.angular.z, dt,
            float(self.get_parameter("angular_acceleration").value),
            float(self.get_parameter("angular_deceleration").value))
        if emergency:
            self.current = Twist()
        self.publisher.publish(self.current)
        now = time.monotonic()
        pressed = tuple(name for name in self.keys if self.keys[name])
        status = (
            pressed, mode, round(self.current.linear.x, 3),
            round(self.current.linear.y, 3), round(self.current.angular.z, 3),
            self.window_focused, emergency)
        if status != self.last_status or now - self.last_status_log >= 1.0:
            speed = (self.current.linear.x ** 2
                     + self.current.linear.y ** 2) ** 0.5
            self.get_logger().info(
                f"pressed={pressed or '-'} mode={mode} "
                f"twist=({self.current.linear.x:.3f},"
                f"{self.current.linear.y:.3f},{self.current.angular.z:.3f}) "
                f"speed={speed:.3f} focus={self.window_focused} "
                f"emergency={emergency} reason={self.emergency_reason or '-'}")
            self.last_status, self.last_status_log = status, now
        self._draw_status(self.current, mode, emergency)

    def stop(self):
        if not self.emergency_latched:
            self._activate_emergency("node shutdown")
        for _ in range(5):
            try:
                self.publisher.publish(Twist())
            except Exception:
                break
        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardGuiControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
