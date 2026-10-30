"""Simulation-only check: which walls does the LiDAR see along the route?

For each pose in route_check_points.yaml the robot is moved there with
/gazebo/set_entity_state, one /scan_filtered message is taken and each valid
point is projected into the map with the true pose and the nominal LiDAR
mount (TF base_footprint -> lidar_scan_link). Each point is assigned to the
nearest wood-frame face from field_layout.yaml:
  x-wall: face normal along x (constrains the x position)
  y-wall: face normal along y (constrains the y position)
  other : farther than match_distance from any face (should be ~0)
Prints a Markdown table and optionally writes CSV.
"""
import argparse
import csv
import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from omni_robot_gazebo.field_layout import frame_rects, load_layout


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def rotate(q, v):
    """Rotate vector v by quaternion q (x, y, z, w attributes)."""
    x, y, z = v
    tx = 2 * (q.y * z - q.z * y)
    ty = 2 * (q.z * x - q.x * z)
    tz = 2 * (q.x * y - q.y * x)
    return (x + q.w * tx + (q.y * tz - q.z * ty),
            y + q.w * ty + (q.z * tx - q.x * tz),
            z + q.w * tz + (q.x * ty - q.y * tx))


def nearest_face(px, py, rects):
    """Return (distance, 'x' or 'y') of the nearest rectangle face."""
    best = (float("inf"), None)
    for _, x0, x1, y0, y1 in rects:
        # faces at x = x0, x1 (normal along x)
        cy = min(max(py, y0), y1)
        for fx in (x0, x1):
            d = math.hypot(px - fx, py - cy)
            if d < best[0]:
                best = (d, "x")
        cx = min(max(px, x0), x1)
        for fy in (y0, y1):
            d = math.hypot(px - cx, py - fy)
            if d < best[0]:
                best = (d, "y")
    return best


class VisibilityCheck(Node):
    def __init__(self):
        super().__init__("scan_visibility_check", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.set_state = self.create_client(
            SetEntityState, "/gazebo/set_entity_state")
        self.scan = None
        self.create_subscription(
            LaserScan, "/scan_filtered", self.scan_callback,
            qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scan_callback(self, msg):
        self.scan = msg

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def move_robot(self, x, y, yaw):
        request = SetEntityState.Request()
        state = EntityState()
        state.name = "omni_robot"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.005
        state.pose.orientation.z = math.sin(yaw / 2)
        state.pose.orientation.w = math.cos(yaw / 2)
        state.reference_frame = "world"
        request.state = state
        future = self.set_state.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().success

    def mount(self):
        end = time.time() + 15.0
        while not self.tf_buffer.can_transform(
                "base_footprint", "lidar_scan_link", rclpy.time.Time()):
            if time.time() > end:
                raise SystemExit("TF base_footprint -> lidar_scan_link missing")
            rclpy.spin_once(self, timeout_sec=0.1)
        transform = self.tf_buffer.lookup_transform(
            "base_footprint", "lidar_scan_link", rclpy.time.Time())
        t = transform.transform
        return t.translation.x, t.translation.y, t.rotation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--points", default=os.path.join(
        get_package_share_directory("omni_robot_gazebo"),
        "config", "route_check_points.yaml"))
    parser.add_argument("--match-distance", type=float, default=0.05)
    parser.add_argument("--settle", type=float, default=1.5,
                        help="wall seconds to wait after moving the robot")
    parser.add_argument("--csv", default="")
    args = parser.parse_args()

    with open(args.points) as stream:
        points = yaml.safe_load(stream)["points"]
    rects = frame_rects(load_layout())

    rclpy.init()
    node = VisibilityCheck()
    if not node.set_state.wait_for_service(timeout_sec=10.0):
        raise SystemExit("/gazebo/set_entity_state not available")
    node.spin_for(2.0)
    mx, my, mq = node.mount()

    rows = []
    for point in points:
        x, y, yaw_deg = point["pose"]
        yaw = math.radians(yaw_deg)
        node.move_robot(x, y, yaw)
        node.scan = None
        node.spin_for(args.settle)
        node.scan = None
        while node.scan is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        scan = node.scan
        counts = {"x": 0, "y": 0, "other": 0}
        nearest = float("inf")
        sx = x + math.cos(yaw) * mx - math.sin(yaw) * my
        sy = y + math.sin(yaw) * mx + math.cos(yaw) * my
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= scan.range_max):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            # sensor frame -> base_footprint (handles tilt and upside down)
            bx, by, _ = rotate(mq, (r * math.cos(angle), r * math.sin(angle), 0.0))
            px = sx + math.cos(yaw) * bx - math.sin(yaw) * by
            py = sy + math.sin(yaw) * bx + math.cos(yaw) * by
            distance, kind = nearest_face(px, py, rects)
            nearest = min(nearest, r)
            counts[kind if distance <= args.match_distance else "other"] += 1
        rows.append({
            "name": point["name"], "x": x, "y": y, "yaw_deg": yaw_deg,
            "x_wall_points": counts["x"], "y_wall_points": counts["y"],
            "unmatched_points": counts["other"],
            "nearest_return_m": round(nearest, 3),
        })

    print("| point | pose (x, y, yaw) | x-wall points | y-wall points "
          "| unmatched | nearest return [m] |")
    print("|---|---|---:|---:|---:|---:|")
    for row in rows:
        print(f"| {row['name']} | ({row['x']:.3f}, {row['y']:.3f}, "
              f"{row['yaw_deg']:.0f}) | {row['x_wall_points']} | "
              f"{row['y_wall_points']} | {row['unmatched_points']} | "
              f"{row['nearest_return_m']} |")
    if args.csv:
        with open(args.csv, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
