"""Record stamped streams for the cross-track bias study. usage: bias_rec.py OUT.npz
Streams (each row: stamp, values):
  gt   /ground_truth/odom   x y yaw vx vy (world)
  trk  /tracking/odom       x y yaw vx vy (body twist)
  ekf  /odom                x y yaw vx vy (body twist)
  mo   TF map->odom at each /mission/debug   x y yaw
  dbg  /mission/debug       est_x est_y est_yaw amcl_x amcl_y amcl_yaw
  amcl /amcl_pose           x y yaw
"""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

from omni_robot_mission.mission_node import DEBUG_FIELDS  # noqa: E402


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def st(s):
    return s.sec + s.nanosec * 1e-9


class Rec(Node):
    def __init__(self):
        super().__init__("bias_rec", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.d = {k: [] for k in ("gt", "trk", "ekf", "mo", "dbg", "amcl", "scan")}
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        for topic, key in (("/ground_truth/odom", "gt"), ("/tracking/odom", "trk"),
                           ("/odom", "ekf")):
            self.create_subscription(Odometry, topic,
                                     lambda m, k=key: self.on_odom(k, m), 100)
        self.create_subscription(Float64MultiArray, "/mission/debug", self.on_dbg, 50)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.scan_meta = None
        self.i = {n: DEBUG_FIELDS.index(n) for n in
                  ("est_x", "est_y", "est_yaw", "amcl_x", "amcl_y", "amcl_yaw")}

    def on_odom(self, key, m):
        p, t = m.pose.pose, m.twist.twist.linear
        self.d[key].append([st(m.header.stamp), p.position.x, p.position.y,
                            yaw_of(p.orientation), t.x, t.y])

    def on_scan(self, m):
        self.scan_meta = [m.angle_min, m.angle_increment, m.range_min, m.range_max]
        self.d["scan"].append([st(m.header.stamp)] + list(m.ranges))

    def on_amcl(self, m):
        p = m.pose.pose
        self.d["amcl"].append([st(m.header.stamp), p.position.x, p.position.y,
                               yaw_of(p.orientation)])

    def on_dbg(self, m):
        now = self.get_clock().now().nanoseconds * 1e-9
        v = list(m.data)
        self.d["dbg"].append([v[0]] + [v[self.i[n] + 1] for n in
                                       ("est_x", "est_y", "est_yaw", "amcl_x", "amcl_y", "amcl_yaw")])
        try:
            tf = self.tf.lookup_transform("map", "odom", Time()).transform
            self.d["mo"].append([now, tf.translation.x, tf.translation.y, yaw_of(tf.rotation)])
        except Exception:
            pass


def main():
    rclpy.init()
    n = Rec()
    try:
        rclpy.spin(n)
    except BaseException:
        pass
    np.savez(sys.argv[1], scan_meta=np.array(n.scan_meta), **{k: np.array(v, dtype=np.float32 if k == 'scan' else float) for k, v in n.d.items()})


if __name__ == "__main__":
    main()
