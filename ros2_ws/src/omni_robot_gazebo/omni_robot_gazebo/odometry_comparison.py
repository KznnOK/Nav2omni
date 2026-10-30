"""Simulation-only test: odometry error under repeated hard acceleration.

Publishes step changes of /cmd_vel (no acceleration limit, so the drive
wheels slip) and compares the displacement of each odometry source with the
Gazebo ground truth, both measured from the pose at the start of the test and
expressed in the start frame of the ground truth.

Sources: /tracking/odom (tracking wheels + IMU), /wheel/odom (drive wheels),
/odom (EKF fusing /tracking/odom and the IMU).
Use an open area: the pattern moves at most about 0.4 m from the start.
"""
import argparse
import csv
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

SOURCES = ["/tracking/odom", "/wheel/odom", "/odom"]

# (vx, vy, wz, seconds): abrupt starts, reversals and stops
CYCLE = [
    (1.0, 0.0, 0.0, 0.35), (-1.0, 0.0, 0.0, 0.35), (0.0, 0.0, 0.0, 0.4),
    (0.0, 1.0, 0.0, 0.35), (0.0, -1.0, 0.0, 0.35), (0.0, 0.0, 0.0, 0.4),
    (0.7, 0.7, 0.0, 0.35), (-0.7, -0.7, 0.0, 0.35), (0.0, 0.0, 0.0, 0.4),
    (0.0, 0.0, 3.0, 1.0), (0.0, 0.0, -3.0, 1.0), (0.0, 0.0, 0.0, 0.4),
    (0.6, 0.0, 2.0, 0.6), (-0.6, 0.0, -2.0, 0.6), (0.0, 0.0, 0.0, 0.6),
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose2d(msg):
    p = msg.pose.pose
    return p.position.x, p.position.y, yaw_of(p.orientation)


def relative(start, now):
    """Displacement of now in the start frame."""
    dx, dy = now[0] - start[0], now[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    dyaw = math.atan2(math.sin(now[2] - start[2]), math.cos(now[2] - start[2]))
    return c * dx + s * dy, -s * dx + c * dy, dyaw


class Comparison(Node):
    def __init__(self, cycles):
        super().__init__("odometry_comparison", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.steps = CYCLE * cycles + [(0.0, 0.0, 0.0, 1.0)]
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.latest = {}
        self.start = None
        self.max_error = {s: 0.0 for s in SOURCES}
        self.max_yaw_error = {s: 0.0 for s in SOURCES}
        self.max_true_speed = 0.0
        for topic in ["/ground_truth/odom"] + SOURCES:
            self.create_subscription(
                Odometry, topic, lambda m, t=topic: self.on_odom(t, m), 50)
        self.index = 0
        self.step_start = None
        self.done = False
        self.timer = self.create_timer(0.01, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, topic, msg):
        self.latest[topic] = msg
        if topic == "/ground_truth/odom":
            t = msg.twist.twist.linear
            self.max_true_speed = max(self.max_true_speed, math.hypot(t.x, t.y))
        if self.start is not None and topic == "/ground_truth/odom":
            self.update_errors()

    def errors(self, source):
        truth = relative(self.start["/ground_truth/odom"],
                         pose2d(self.latest["/ground_truth/odom"]))
        est = relative(self.start[source], pose2d(self.latest[source]))
        return (math.hypot(est[0] - truth[0], est[1] - truth[1]),
                abs(math.atan2(math.sin(est[2] - truth[2]),
                               math.cos(est[2] - truth[2]))))

    def update_errors(self):
        for source in SOURCES:
            position, yaw = self.errors(source)
            self.max_error[source] = max(self.max_error[source], position)
            self.max_yaw_error[source] = max(self.max_yaw_error[source], yaw)

    def tick(self):
        t = self.now()
        if t == 0.0 or self.done:
            return
        if self.start is None:
            if all(k in self.latest for k in ["/ground_truth/odom"] + SOURCES):
                self.start = {k: pose2d(v) for k, v in self.latest.items()}
                self.step_start = t
            return
        if t - self.step_start >= self.steps[self.index][3]:
            self.index += 1
            self.step_start = t
            if self.index >= len(self.steps):
                self.pub.publish(Twist())
                self.done = True
                return
        vx, vy, wz, _ = self.steps[self.index]
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--csv", default="")
    parser.add_argument("--label", default="")
    args = parser.parse_args()

    rclpy.init()
    node = Comparison(args.cycles)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    truth = relative(node.start["/ground_truth/odom"],
                     pose2d(node.latest["/ground_truth/odom"]))
    print(f"ground truth net displacement: dx={truth[0]:+.3f} "
          f"dy={truth[1]:+.3f} dyaw={math.degrees(truth[2]):+.2f} deg, "
          f"max speed {node.max_true_speed:.2f} m/s")
    rows = []
    print("| source | final position error [m] | final yaw error [deg] "
          "| max position error [m] | max yaw error [deg] |")
    print("|---|---:|---:|---:|---:|")
    for source in SOURCES:
        position, yaw = node.errors(source)
        row = {
            "label": args.label, "source": source,
            "final_position_error_m": round(position, 4),
            "final_yaw_error_deg": round(math.degrees(yaw), 3),
            "max_position_error_m": round(node.max_error[source], 4),
            "max_yaw_error_deg": round(math.degrees(node.max_yaw_error[source]), 3),
        }
        rows.append(row)
        print(f"| {source} | {row['final_position_error_m']} | "
              f"{row['final_yaw_error_deg']} | {row['max_position_error_m']} | "
              f"{row['max_yaw_error_deg']} |")
    if args.csv:
        write_header = True
        try:
            with open(args.csv) as stream:
                write_header = not stream.read(1)
        except FileNotFoundError:
            pass
        with open(args.csv, "a", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            if write_header:
                writer.writeheader()
            writer.writerows(rows)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
