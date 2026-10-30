"""Record mission diagnostics with ground truth, wheel commands/speeds and
the AMCL correction (map->odom) per control step. usage: recorder.py OUT.csv"""
import csv
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformListener

FIELDS = ["ref_x", "ref_y", "ref_v", "ref_yaw", "est_x", "est_y", "est_yaw",
          "raw_vx", "raw_vy", "raw_wz", "cmd_vx", "cmd_vy", "cmd_wz",
          "body_vx", "body_vy", "wheel_scale", "amcl_x", "amcl_y", "amcl_yaw", "pose_age"]
WHEELS = ["front_left_wheel_joint", "front_right_wheel_joint",
          "rear_left_wheel_joint", "rear_right_wheel_joint"]
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Rec(Node):
    def __init__(self, path):
        super().__init__("recorder", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "state"] + FIELDS
                        + ["gt_x", "gt_y", "gt_yaw", "gt_vx", "gt_vy", "gt_speed"]
                        + [f"wcmd_{i}" for i in range(4)]
                        + [f"wvel_{i}" for i in range(4)]
                        + ["mo_x", "mo_y", "mo_yaw", "odom_vx", "odom_vy"])
        self.gt = [math.nan] * 6
        self.wcmd = [math.nan] * 4
        self.wvel = [math.nan] * 4
        self.odom = [math.nan] * 2
        self.state = ""
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(Float64MultiArray, "/mission/debug", self.on_dbg, 50)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 50)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(Float64MultiArray,
                                 "/wheel_velocity_controller/commands", self.on_wcmd, 50)
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.create_subscription(String, "/mission/state", self.on_state, LATCHED)

    def on_state(self, m):
        self.state = m.data.split()[0]

    def on_gt(self, m):
        p = m.pose.pose
        t = m.twist.twist.linear
        self.gt = [p.position.x, p.position.y, yaw_of(p.orientation), t.x, t.y,
                   math.hypot(t.x, t.y)]

    def on_odom(self, m):
        self.odom = [m.twist.twist.linear.x, m.twist.twist.linear.y]

    def on_wcmd(self, m):
        self.wcmd = list(m.data)

    def on_js(self, m):
        v = dict(zip(m.name, m.velocity))
        if all(n in v for n in WHEELS):
            self.wvel = [v[n] for n in WHEELS]

    def on_dbg(self, m):
        try:
            tf = self.tf.lookup_transform("map", "odom", Time()).transform
            mo = [tf.translation.x, tf.translation.y, yaw_of(tf.rotation)]
        except Exception:
            mo = [math.nan] * 3
        self.w.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in
                         [m.data[0], self.state] + list(m.data[1:]) + self.gt
                         + self.wcmd + self.wvel + mo + self.odom])


def main():
    rclpy.init()
    n = Rec(sys.argv[1])
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.f.close()


if __name__ == "__main__":
    main()
