"""Manual / autonomous switching: the only node that publishes /cmd_vel.

Inputs
  /cmd_vel_manual  operator (teleop_twist_joy, or teleop_twist_keyboard in sim)
  /cmd_vel_auto    sakura_mission
  /joy             toggle button (rising edge) and controller liveness
Services
  ~/toggle_autonomy (std_srvs/Trigger), ~/set_autonomy (std_srvs/SetBool)
  for the simulation or a keyboard when no controller is connected.
Outputs
  /cmd_vel, /control_mode ("MANUAL"/"AUTO", latched),
  /autonomy_enabled (Bool, latched; the mission cancels its motion when false)

Safety
  * Commands of the input that is not selected are ignored.
  * If the selected input is silent for cmd_timeout_s, zero is sent.
  * If require_joy and /joy is silent for joy_timeout_s (controller lost),
    zero is sent and AUTO falls back to MANUAL.
  * AUTO -> MANUAL publishes autonomy_enabled=false and zero at once.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ModeManager(Node):

    def __init__(self):
        super().__init__("mode_manager")
        self.declare_parameter("toggle_button", 9)       # PS4 OPTIONS (仮値)
        self.declare_parameter("cmd_timeout_s", 0.2)
        self.declare_parameter("require_joy", True)
        self.declare_parameter("joy_timeout_s", 0.5)
        self.declare_parameter("publish_rate", 50.0)

        self.auto = False
        self.manual_cmd = None
        self.manual_time = None
        self.auto_cmd = None
        self.auto_time = None
        self.joy_time = None
        self.button_was_pressed = False
        self.stopped_reason = ""

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "/control_mode", LATCHED)
        self.enabled_pub = self.create_publisher(Bool, "/autonomy_enabled", LATCHED)
        self.create_subscription(Twist, "/cmd_vel_manual", self.on_manual, 10)
        self.create_subscription(Twist, "/cmd_vel_auto", self.on_auto, 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_service(Trigger, "~/toggle_autonomy", self.on_toggle)
        self.create_service(SetBool, "~/set_autonomy", self.on_set)
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate").value), self.tick)
        self.set_mode(False, "startup")

    def now(self):
        return self.get_clock().now()

    def age(self, stamp):
        if stamp is None:
            return float("inf")
        return (self.now() - stamp).nanoseconds * 1e-9

    def set_mode(self, auto, reason):
        changed = auto != self.auto
        self.auto = auto
        if not auto:
            # cancel autonomy first, then stop the robot immediately
            self.enabled_pub.publish(Bool(data=False))
            self.auto_cmd = None
            self.cmd_pub.publish(Twist())
        else:
            self.manual_cmd = None
            self.enabled_pub.publish(Bool(data=True))
        self.mode_pub.publish(String(data="AUTO" if auto else "MANUAL"))
        if changed or reason == "startup":
            self.get_logger().info(
                f"mode {'AUTO' if auto else 'MANUAL'} ({reason})")

    def joy_lost(self):
        return (bool(self.get_parameter("require_joy").value)
                and self.age(self.joy_time)
                > float(self.get_parameter("joy_timeout_s").value))

    def on_manual(self, msg):
        self.manual_cmd, self.manual_time = msg, self.now()

    def on_auto(self, msg):
        if self.auto:
            self.auto_cmd, self.auto_time = msg, self.now()

    def on_joy(self, msg):
        self.joy_time = self.now()
        index = int(self.get_parameter("toggle_button").value)
        pressed = index < len(msg.buttons) and msg.buttons[index] == 1
        if pressed and not self.button_was_pressed:
            self.set_mode(not self.auto, "joy button")
        self.button_was_pressed = pressed

    def on_toggle(self, request, response):
        return self.on_set(SetBool.Request(data=not self.auto), response)

    def on_set(self, request, response):
        if request.data and self.joy_lost():
            response.success = False
            response.message = "joy is not connected (require_joy)"
            return response
        self.set_mode(bool(request.data), "service")
        response.success = True
        response.message = "AUTO" if self.auto else "MANUAL"
        return response

    def tick(self):
        if self.joy_lost():
            if self.auto:
                self.set_mode(False, "joy lost")
            self.report_stop("joy lost")
            self.cmd_pub.publish(Twist())
            return
        cmd, stamp = ((self.auto_cmd, self.auto_time) if self.auto
                      else (self.manual_cmd, self.manual_time))
        timeout = float(self.get_parameter("cmd_timeout_s").value)
        if cmd is None or self.age(stamp) > timeout:
            self.report_stop("selected input timed out")
            self.cmd_pub.publish(Twist())
            return
        self.stopped_reason = ""
        self.cmd_pub.publish(cmd)

    def report_stop(self, reason):
        if reason != self.stopped_reason:
            self.stopped_reason = reason
            self.get_logger().debug(f"zero command: {reason}")


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
