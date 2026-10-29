import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ManualRecorder(Node):
    """Record manual commands, poses, and operator milestone events."""

    def __init__(self):
        super().__init__("manual_recorder")
        self.declare_parameter("mission_config", "")
        self.declare_parameter("results_root", "results")
        self.declare_parameter("input_mode", "keyboard")
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("stop_speed_threshold", 0.03)
        mission_path = str(self.get_parameter("mission_config").value)
        if not mission_path:
            mission_path = os.path.join(
                get_package_share_directory("omni_robot_nav2"),
                "config", "mission.yaml")
        with open(mission_path, encoding="utf-8") as stream:
            values = yaml.safe_load(stream)["mission_manager"]["ros__parameters"]
        self.pickup = (float(values["pickup_x"]), float(values["pickup_y"]),
                       float(values["pickup_yaw"]))
        self.drop = (float(values["drop_x"]), float(values["drop_y"]),
                     float(values["drop_yaw"]))
        root = Path(str(self.get_parameter("results_root").value))
        run_id = datetime.now().strftime("manual_%Y%m%d_%H%M%S_%f")
        self.result_dir = root / run_id
        self.result_dir.mkdir(parents=True, exist_ok=False)
        self.trajectory_file = (self.result_dir / "trajectory.csv").open(
            "w", newline="", encoding="utf-8")
        self.trajectory = csv.writer(self.trajectory_file)
        self.trajectory.writerow([
            "elapsed_s", "state", "map_x", "map_y", "map_yaw",
            "odom_x", "odom_y", "odom_yaw", "linear_x", "linear_y",
            "angular_z", "input_mode", "deadman", "precision", "high_speed",
            "emergency_stop_count", "mode_switch_count", "distance_m",
        ])
        self.start_time = None
        self.pickup_time = None
        self.drop_time = None
        self.pickup_stop_start = None
        self.pickup_stop_duration = 0.0
        self.state = "IDLE"
        self.map_pose = None
        self.odom_pose = None
        self.command = Twist()
        self.distance = 0.0
        self.last_odom = None
        self.last_sample_time = None
        self.deadman = False
        self.precision = False
        self.high_speed = False
        self.emergency_stops = 0
        self.mode_switches = 0
        self.finished = False
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.map_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 20)
        self.create_subscription(Twist, "/cmd_vel_manual", self.command_callback, 20)
        self.create_subscription(String, "/manual/events", self.event_callback, 20)
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("sample_rate_hz").value),
            self.sample)
        self.get_logger().info(f"Manual results: {self.result_dir}")

    def map_callback(self, message):
        pose = message.pose.pose
        self.map_pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))

    def odom_callback(self, message):
        pose = message.pose.pose
        current = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        if self.last_odom is not None:
            self.distance += math.hypot(current[0] - self.last_odom[0],
                                        current[1] - self.last_odom[1])
        self.last_odom = current
        self.odom_pose = current

    def command_callback(self, message):
        self.command = message
        self.deadman = any(abs(value) > 1e-6 for value in (
            message.linear.x, message.linear.y, message.angular.z))

    def event_callback(self, message):
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignoring malformed manual event")
            return
        name = event.get("event")
        now = self.get_clock().now()
        if name == "start_toggle":
            if self.start_time is None:
                self.start_time = now
                self.state = "RUNNING"
                self.get_logger().info("Manual measurement started")
            elif self.drop_time is None:
                self.state = "RUNNING"
        elif name == "pickup" and self.start_time is not None:
            self.pickup_time = now
            self.state = "WAIT_AT_PICKUP"
            self.pickup_stop_start = None
            self.get_logger().info("Pickup milestone recorded")
        elif name == "drop" and self.start_time is not None:
            self.drop_time = now
            self.state = "COMPLETED"
            self.save_results()
            self.get_logger().info("Drop milestone recorded; results saved")
        elif name == "emergency_stop":
            self.emergency_stops += 1
        elif name == "mode_change":
            self.mode_switches += 1

    def sample(self):
        now = self.get_clock().now()
        if self.pickup_time is not None and self.drop_time is None:
            speed = math.hypot(self.command.linear.x, self.command.linear.y)
            speed = math.hypot(speed, self.command.angular.z)
            threshold = float(self.get_parameter("stop_speed_threshold").value)
            if speed <= threshold:
                if self.pickup_stop_start is None:
                    self.pickup_stop_start = now
                self.pickup_stop_duration = (
                    now - self.pickup_stop_start).nanoseconds / 1e9
            else:
                self.pickup_stop_start = None
        if self.start_time is None:
            return
        elapsed = (now - self.start_time).nanoseconds / 1e9
        map_pose = self.map_pose or (float("nan"), float("nan"), float("nan"))
        odom_pose = self.odom_pose or (float("nan"), float("nan"), float("nan"))
        self.trajectory.writerow([
            f"{elapsed:.6f}", self.state, *map_pose, *odom_pose,
            self.command.linear.x, self.command.linear.y, self.command.angular.z,
            str(self.get_parameter("input_mode").value), self.deadman,
            self.precision, self.high_speed, self.emergency_stops,
            self.mode_switches, f"{self.distance:.6f}",
        ])
        self.trajectory_file.flush()

    def save_results(self):
        if self.finished:
            return
        self.finished = True
        self.trajectory_file.flush()
        start = self.start_time
        pickup = self.pickup_time
        drop = self.drop_time
        duration = ((drop - start).nanoseconds / 1e9
                    if start is not None and drop is not None else float("nan"))
        pickup_duration = ((pickup - start).nanoseconds / 1e9
                           if start is not None and pickup is not None else float("nan"))
        drop_pose = self.map_pose or (float("nan"), float("nan"), float("nan"))
        summary_path = self.result_dir / "summary.csv"
        with summary_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["run_id", "input_mode", "pickup_time_s",
                             "pickup_stop_duration_s", "drop_time_s",
                             "total_time_s", "map_x", "map_y", "map_yaw",
                             "pickup_error_m", "drop_error_m", "drop_yaw_error_rad",
                             "distance_m", "emergency_stop_count",
                             "mode_switch_count", "success"])
            writer.writerow([
                self.result_dir.name, self.get_parameter("input_mode").value,
                pickup_duration, self.pickup_stop_duration,
                ((drop - start).nanoseconds / 1e9
                 if start is not None and drop is not None else float("nan")),
                duration, *drop_pose,
                float("nan"), math.hypot(drop_pose[0] - self.drop[0],
                                         drop_pose[1] - self.drop[1]),
                math.atan2(math.sin(drop_pose[2] - self.drop[2]),
                           math.cos(drop_pose[2] - self.drop[2])),
                self.distance, self.emergency_stops, self.mode_switches,
                self.pickup_stop_duration >= 3.0,
            ])
        metadata = {
            "run_id": self.result_dir.name,
            "input_mode": self.get_parameter("input_mode").value,
            "pickup": self.pickup,
            "drop": self.drop,
            "mission_config": str(self.get_parameter("mission_config").value),
        }
        with (self.result_dir / "metadata.yaml").open("w", encoding="utf-8") as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False)
        with (self.result_dir / "report.md").open("w", encoding="utf-8") as stream:
            stream.write(f"# Manual run {self.result_dir.name}\n\n")
            stream.write(f"- Input: {metadata['input_mode']}\n")
            stream.write(f"- Total time: {duration:.3f} s\n")
            stream.write(f"- Pickup stop: {self.pickup_stop_duration:.3f} s\n")
            stream.write(f"- Result directory: `{self.result_dir}`\n")

    def stop(self):
        self.save_results()
        self.trajectory_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = ManualRecorder()
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
