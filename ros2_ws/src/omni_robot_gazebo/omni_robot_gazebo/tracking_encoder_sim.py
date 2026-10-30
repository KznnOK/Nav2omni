"""Simulation only: tracking wheel encoder errors (sim_conditions.yaml).

Republishes the tracking wheel joints of /joint_states on
/tracking/joint_states with
  * quantization to the encoder resolution (encoder_cpr counts/rev),
  * lift-off / slip events: at lift_rate_hz per wheel (Poisson), the reading
    stops changing for lift_duration_s and the travel in that time is lost.
tracking_odometry reads this topic instead of /joint_states only when an
error of this kind is enabled.
"""
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from omni_robot_description.geometry import (
    default_geometry_file, load_geometry, tracking_wheel_layout)


class TrackingEncoderSim(Node):

    def __init__(self):
        super().__init__("tracking_encoder_sim")
        geometry = load_geometry(self.declare_parameter(
            "geometry_file", default_geometry_file()).value)
        self.names = tracking_wheel_layout(geometry)[0]
        cpr = int(geometry["tracking_wheels"]["encoder_cpr"])
        self.step = (2.0 * math.pi / cpr
                     if self.declare_parameter("quantize", False).value else 0.0)
        self.lift_rate = float(self.declare_parameter("lift_rate_hz", 0.0).value)
        self.lift_duration = float(
            self.declare_parameter("lift_duration_s", 0.02).value)
        self.rng = random.Random(self.declare_parameter("seed", 1).value)
        self.offset = [0.0] * len(self.names)    # travel lost while lifted
        self.lifted_until = [-1.0] * len(self.names)
        self.held = [0.0] * len(self.names)
        self.last_time = None
        self.lift_count = 0
        self.publisher = self.create_publisher(
            JointState, "/tracking/joint_states", 50)
        self.create_subscription(JointState, "/joint_states", self.callback, 50)
        self.get_logger().info(
            f"tracking encoder errors: quantization step {self.step:.6f} rad, "
            f"lift {self.lift_rate} Hz x {self.lift_duration} s")

    def callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(n in positions for n in self.names):
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_time is None else max(0.0, t - self.last_time)
        self.last_time = t
        out = JointState()
        out.header = msg.header
        out.name = list(self.names)
        for i, name in enumerate(self.names):
            true = positions[name]
            if 0.0 < self.lifted_until[i] <= t:
                # touched down again: the travel while lifted is lost
                self.offset[i] = true - self.held[i]
                self.lifted_until[i] = -1.0
            if self.lifted_until[i] > t:
                reading = self.held[i]
            else:
                reading = true - self.offset[i]
                if self.lift_rate > 0.0 and self.rng.random() < self.lift_rate * dt:
                    self.lifted_until[i] = t + self.lift_duration
                    self.held[i] = reading
                    self.lift_count += 1
            if self.step > 0.0:
                reading = math.floor(reading / self.step) * self.step
            out.position.append(reading)
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingEncoderSim()
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
