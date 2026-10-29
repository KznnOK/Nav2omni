import csv
import math
import os
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def quaternion_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(actual, target):
    return math.atan2(math.sin(actual - target), math.cos(actual - target))


class MissionManager(Node):
    """Run one complete narrow-section and Nav2 mission."""

    STATES = (
        "STARTUP",
        "WAIT_FOR_LOCALIZATION",
        "NARROW_SECTION",
        "STOP_AT_NARROW_EXIT",
        "NAVIGATE_TO_PICKUP",
        "WAIT_AT_PICKUP",
        "NAVIGATE_TO_ROTATION_POINT",
        "ROTATE_180",
        "NAVIGATE_TO_DROP",
        "STOP_AT_DROP",
        "SAVE_RESULTS",
        "COMPLETED",
        "FAILED",
    )

    def __init__(self):
        super().__init__("mission_manager")
        self.declare_parameter("trial_id", 1)
        self.declare_parameter("results_dir", "")
        self.declare_parameter("start_x", -3.1510900301327864)
        self.declare_parameter("start_y", -3.136311120883107)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("pickup_x", -2.4844879132843367)
        self.declare_parameter("pickup_y", 1.7525297248907472)
        self.declare_parameter("pickup_yaw", 0.0)
        self.declare_parameter("rotation_x", -1.0)
        self.declare_parameter("rotation_y", -2.4)
        self.declare_parameter("rotation_yaw", 0.0)
        self.declare_parameter("drop_x", -0.6351846679129013)
        self.declare_parameter("drop_y", -2.2289173701157674)
        self.declare_parameter("drop_yaw", math.pi)
        self.declare_parameter("narrow_exit_x", -1.35)
        self.declare_parameter("narrow_exit_y", -3.136311120883107)
        self.declare_parameter("narrow_speed", 0.08)
        self.declare_parameter("narrow_timeout_s", 35.0)
        self.declare_parameter("state_timeout_s", 120.0)
        self.declare_parameter("yaw_tolerance_rad", 0.35)
        self.declare_parameter("position_tolerance_m", 0.15)
        self.declare_parameter("stop_confirm_s", 0.3)
        self.declare_parameter("pickup_wait_s", 3.0)
        self.declare_parameter("section_only", False)
        self.declare_parameter("fixed_speed", 0.08)
        self.declare_parameter("fixed_segment_timeout_s", 45.0)
        self.declare_parameter("obstacle_stop_distance_m", 0.18)
        self.declare_parameter("scan_self_occlusion_min_range_m", 0.08)

        p = lambda name: float(self.get_parameter(name).value)
        self.start = (p("start_x"), p("start_y"), p("start_yaw"))
        self.pickup = (p("pickup_x"), p("pickup_y"), p("pickup_yaw"))
        self.rotation = (p("rotation_x"), p("rotation_y"), p("rotation_yaw"))
        self.drop = (p("drop_x"), p("drop_y"), p("drop_yaw"))
        self.exit_x = p("narrow_exit_x")
        self.exit_y = p("narrow_exit_y")
        self.narrow_speed = p("narrow_speed")
        self.narrow_timeout = p("narrow_timeout_s")
        self.state_timeout = p("state_timeout_s")
        self.yaw_tolerance = p("yaw_tolerance_rad")
        self.position_tolerance = p("position_tolerance_m")
        self.stop_confirm_s = p("stop_confirm_s")
        self.pickup_wait_s = p("pickup_wait_s")
        self.section_only = bool(self.get_parameter("section_only").value)
        self.fixed_speed = p("fixed_speed")
        self.fixed_segment_timeout = p("fixed_segment_timeout_s")
        self.obstacle_stop_distance = p("obstacle_stop_distance_m")
        self.scan_self_occlusion_min_range = p("scan_self_occlusion_min_range_m")
        self.fixed_points = [
            self.pickup,
            (-2.475, 0.595, 0.0),
            (-2.805, 0.595, 0.0),
            (-2.805, -0.445, 0.0),
            (-2.305, -0.445, 0.0),
            (-2.305, -2.395, 0.0),
            self.rotation,
        ]
        self.trial_id = int(self.get_parameter("trial_id").value)
        result_dir = str(self.get_parameter("results_dir").value)
        self.results_dir = Path(result_dir or os.path.join(os.getcwd(), "results"))
        self.results_dir.mkdir(parents=True, exist_ok=True)

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "/spin")
        self.amcl_state = self.create_client(GetState, "/amcl/get_state")
        self.map_state = self.create_client(GetState, "/map_server/get_state")
        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_callback, pose_qos
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 20)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 20)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)

        self.pose = None
        self.odom_pose = None
        self.odom_twist = Twist()
        self.scan = None
        self.scan_valid = False
        self.scan_blocked_count = 0
        self.last_odom_time = None
        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.state = "STARTUP"
        self.state_started = self.get_clock().now()
        self.node_started = self.state_started
        self.finished = False
        self.failure_state = ""
        self.failure_reason = ""
        self.pending_goal = None
        self.pending_result = None
        self.pending_kind = None
        self.lifecycle_futures = {}
        self.accepted = {}
        self.results = {}
        self.result_status = {}
        self.samples = []
        self.mission_started = None
        self.pickup_stop_since = None
        self.wait_started = None
        self.stop_since = None
        self.narrow_start_odom_x = None
        self.narrow_started = None
        self.narrow_finished = None
        self.narrow_distance = None
        self.pickup_pose = None
        self.drop_pose = None
        self.fixed_segment_index = 0
        self.fixed_segment_started = None
        self.fixed_segment_times = []
        self.fixed_max_yaw_error = 0.0
        self.fixed_stop_errors = []
        self.fixed_min_scan = None
        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info("Mission Manager state=STARTUP")

    def now(self):
        return self.get_clock().now()

    def elapsed(self):
        return (self.now() - self.state_started).nanoseconds / 1e9

    def transition(self, state):
        self.state = state
        self.state_started = self.now()
        self.get_logger().info(f"State {state} started")

    def amcl_callback(self, msg):
        self.pose = msg.pose.pose

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_twist = msg.twist.twist
        self.last_odom_time = self.now()

    def scan_callback(self, msg):
        self.scan = msg

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.now()

    def update(self):
        if self.finished:
            return
        self.record_sample()
        if self.state == "STARTUP":
            self.transition("WAIT_FOR_LOCALIZATION")
            return
        if self.state == "WAIT_FOR_LOCALIZATION":
            self.update_localization()
        elif self.state == "NARROW_SECTION":
            self.update_narrow_section()
        elif self.state == "FIXED_SECTION":
            self.update_fixed_section()
        elif self.state == "STOP_AT_FIXED_POINT":
            self.update_fixed_stop()
        elif self.state == "STOP_AT_NARROW_EXIT":
            self.update_stop("NAVIGATE_TO_PICKUP", "pickup", self.pickup)
        elif self.state in (
            "NAVIGATE_TO_PICKUP",
            "NAVIGATE_TO_ROTATION_POINT",
            "ROTATE_180",
            "NAVIGATE_TO_DROP",
        ):
            self.poll_action()
        elif self.state == "WAIT_AT_PICKUP":
            self.update_pickup_wait()
        elif self.state == "STOP_AT_DROP":
            self.update_stop("SAVE_RESULTS", None, None)
        elif self.state == "SAVE_RESULTS":
            self.write_results()
            self.transition("FAILED" if self.failure_reason else "COMPLETED")
            self.finished = True
            self.get_logger().info(f"State {self.state} reached")
        if self.state not in ("COMPLETED", "FAILED") and self.elapsed() > (
            self.narrow_timeout if self.state == "NARROW_SECTION"
            else self.state_timeout
        ):
            self.fail(f"state timeout after {self.elapsed():.1f}s")

    def update_localization(self):
        if self.pose is None or self.odom_pose is None:
            return
        if not self.nav_client.server_is_ready() or not self.spin_client.server_is_ready():
            return
        if not self.lifecycle_active("amcl", self.amcl_state):
            return
        if not self.lifecycle_active("map_server", self.map_state):
            return
        if not self.section_only:
            xy = math.hypot(self.pose.position.x - self.start[0],
                            self.pose.position.y - self.start[1])
            yaw = abs(
                angle_error(quaternion_yaw(self.pose.orientation), self.start[2])
            )
            if xy > self.position_tolerance or yaw > self.yaw_tolerance:
                self.fail(f"start pose mismatch: xy={xy:.3f} yaw={yaw:.3f}")
                return
        self.mission_started = self.now()
        self.narrow_start_odom_x = self.odom_pose.position.x
        self.narrow_started = self.now()
        self.transition("NARROW_SECTION")

    def lifecycle_active(self, name, client):
        if not client.service_is_ready():
            return False
        if name not in self.lifecycle_futures:
            self.lifecycle_futures[name] = client.call_async(GetState.Request())
            return False
        future = self.lifecycle_futures[name]
        if not future.done():
            return False
        del self.lifecycle_futures[name]
        if future.exception() is not None:
            self.fail(f"{name} lifecycle query failed")
            return False
        return future.result().current_state.id == 3

    def update_narrow_section(self):
        if self.odom_pose is None or self.last_odom_time is None:
            self.fail("odometry unavailable")
            return
        if (self.now() - self.last_odom_time).nanoseconds / 1e9 > 0.5:
            self.fail("odometry timeout")
            return
        if self.scan_is_blocked():
            self.fail("forward LiDAR obstacle detected")
            return
        yaw = quaternion_yaw(self.odom_pose.orientation)
        if abs(angle_error(yaw, 0.0)) > self.yaw_tolerance:
            self.fail(f"narrow-section yaw exceeded limit: {yaw:.3f}")
            return
        distance = self.odom_pose.position.x - self.narrow_start_odom_x
        target_distance = self.exit_x - self.start[0]
        if distance >= target_distance:
            self.narrow_distance = distance
            self.narrow_finished = self.now()
            self.stop_since = None
            self.publish_stop()
            self.transition("STOP_AT_NARROW_EXIT")
            return
        cmd = Twist()
        cmd.linear.x = self.narrow_speed
        cmd.angular.z = max(-0.2, min(0.2, -1.5 * angle_error(yaw, 0.0)))
        self.cmd_pub.publish(cmd)

    def update_fixed_section(self):
        if self.pose is None or self.last_odom_time is None:
            self.fail("fixed-section odometry/TF unavailable")
            return
        if (self.now() - self.last_odom_time).nanoseconds / 1e9 > 0.5:
            self.fail("fixed-section odometry timeout")
            return
        target = self.fixed_points[self.fixed_segment_index]
        dx = target[0] - self.pose.position.x
        dy = target[1] - self.pose.position.y
        distance = math.hypot(dx, dy)
        yaw = quaternion_yaw(self.pose.orientation)
        yaw_error = angle_error(yaw, 0.0)
        self.fixed_max_yaw_error = max(self.fixed_max_yaw_error, abs(yaw_error))
        if self.scan_is_blocked(self.command_angle(dx, dy)):
            self.fail(
                f"fixed-section obstacle at "
                f"({self.pose.position.x:.3f},{self.pose.position.y:.3f})"
            )
            return
        if self.fixed_segment_started is None:
            self.fixed_segment_started = self.now()
        if distance <= self.position_tolerance:
            self.publish_stop()
            self.transition("STOP_AT_FIXED_POINT")
            return
        if self.elapsed() > self.fixed_segment_timeout:
            self.fail(
                f"fixed segment {self.fixed_segment_index + 1} timeout"
            )
            return
        # Select one map axis, then rotate that command into the odometry frame.
        if abs(dx) >= abs(dy):
            map_x, map_y = math.copysign(self.fixed_speed, dx), 0.0
        else:
            map_x, map_y = 0.0, math.copysign(self.fixed_speed, dy)
        cmd = Twist()
        cmd.linear.x = math.cos(yaw) * map_x + math.sin(yaw) * map_y
        cmd.linear.y = -math.sin(yaw) * map_x + math.cos(yaw) * map_y
        cmd.angular.z = max(-0.2, min(0.2, -1.5 * yaw_error))
        self.cmd_pub.publish(cmd)

    def command_angle(self, dx, dy):
        if abs(dx) >= abs(dy):
            return 0.0 if dx >= 0.0 else math.pi
        return math.pi / 2.0 if dy >= 0.0 else -math.pi / 2.0

    def update_fixed_stop(self):
        self.publish_stop()
        if not self.is_stopped():
            self.stop_since = None
            return
        if self.stop_since is None:
            self.stop_since = self.now()
            target = self.fixed_points[self.fixed_segment_index]
            error = math.hypot(
                self.pose.position.x - target[0],
                self.pose.position.y - target[1],
            )
            self.fixed_stop_errors.append(error)
        if (self.now() - self.stop_since).nanoseconds / 1e9 < self.stop_confirm_s:
            return
        duration = (
            self.now() - self.fixed_segment_started
        ).nanoseconds / 1e9
        self.fixed_segment_times.append(duration)
        if self.fixed_segment_index == len(self.fixed_points) - 2:
            self.get_logger().info(
                "Fixed section reached rotation point; stopping before Spin"
            )
            self.transition("SAVE_RESULTS")
            return
        self.fixed_segment_index += 1
        self.fixed_segment_started = self.now()
        self.stop_since = None
        self.transition("FIXED_SECTION")

    def scan_is_blocked(self, direction=0.0):
        if self.scan is None:
            return False
        forward = []
        for i, distance in enumerate(self.scan.ranges):
            if not math.isfinite(distance):
                continue
            if (
                distance < max(
                    self.scan.range_min, self.scan_self_occlusion_min_range
                )
                or distance > self.scan.range_max
            ):
                continue
            angle = self.scan.angle_min + i * self.scan.angle_increment
            relative = math.atan2(
                math.sin(angle - direction), math.cos(angle - direction)
            )
            if abs(relative) <= math.radians(25):
                forward.append(distance)
        if not forward:
            return False
        self.scan_valid = True
        if forward:
            scan_min = min(forward)
            self.fixed_min_scan = (
                scan_min if self.fixed_min_scan is None
                else min(self.fixed_min_scan, scan_min)
            )
        if min(forward) < self.obstacle_stop_distance:
            self.scan_blocked_count += 1
        else:
            self.scan_blocked_count = 0
        return self.scan_blocked_count >= 2

    def update_stop(self, next_state, kind, pose):
        self.publish_stop()
        if not self.is_stopped():
            self.stop_since = None
            return
        if self.stop_since is None:
            self.stop_since = self.now()
        if (self.now() - self.stop_since).nanoseconds / 1e9 < self.stop_confirm_s:
            return
        if kind is None:
            self.transition(next_state)
        else:
            self.send_nav_goal(kind, pose)
            self.transition(next_state)

    def update_pickup_wait(self):
        self.publish_stop()
        if not self.is_stopped():
            self.pickup_stop_since = None
            return
        if self.pickup_stop_since is None:
            self.pickup_stop_since = self.now()
        if (self.now() - self.pickup_stop_since).nanoseconds / 1e9 < self.stop_confirm_s:
            return
        if self.wait_started is None:
            self.wait_started = self.now()
            self.get_logger().info("Pickup stopped; ROS-time 3 s wait started")
        if (self.now() - self.wait_started).nanoseconds / 1e9 >= self.pickup_wait_s:
            self.send_nav_goal("rotation", self.rotation)
            self.transition("NAVIGATE_TO_ROTATION_POINT")

    def send_nav_goal(self, kind, pose):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[:2]
        goal.pose.pose.orientation.z = math.sin(pose[2] / 2)
        goal.pose.pose.orientation.w = math.cos(pose[2] / 2)
        self.pending_kind = kind
        self.pending_goal = self.nav_client.send_goal_async(goal)
        self.pending_result = None
        self.get_logger().info(f"{kind} NavigateToPose sending once")

    def send_spin_goal(self):
        goal = Spin.Goal()
        goal.target_yaw = angle_error(math.pi, quaternion_yaw(self.pose.orientation))
        goal.time_allowance = Duration(sec=30, nanosec=0)
        self.pending_kind = "spin"
        self.pending_goal = self.spin_client.send_goal_async(goal)
        self.pending_result = None
        self.get_logger().info("spin sending once")

    def poll_action(self):
        if self.pending_goal is not None and self.pending_goal.done():
            handle = self.pending_goal.result()
            self.pending_goal = None
            if not handle.accepted:
                self.fail(f"{self.pending_kind} goal rejected")
                return
            self.accepted[self.pending_kind] = self.now()
            self.get_logger().info(f"{self.pending_kind} accepted")
            self.pending_result = handle.get_result_async()
        if self.pending_result is None or not self.pending_result.done():
            return
        result = self.pending_result.result()
        self.pending_result = None
        kind = self.pending_kind
        self.results[kind] = self.now()
        label = self.status_label(result.status)
        self.result_status[kind] = label
        self.get_logger().info(f"{kind} result {label}")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.fail(f"{kind} result {label}")
            return
        if kind == "pickup":
            self.pickup_pose = self.pose
            if self.section_only:
                self.fixed_segment_index = 0
                self.fixed_segment_started = self.now()
                self.transition("FIXED_SECTION")
                return
            self.transition("WAIT_AT_PICKUP")
        elif kind == "rotation":
            if self.section_only:
                self.publish_stop()
                self.transition("SAVE_RESULTS")
                return
            self.stop_since = None
            self.transition("ROTATE_180")
            self.send_spin_goal()
        elif kind == "spin":
            yaw = quaternion_yaw(self.pose.orientation)
            if abs(angle_error(yaw, math.pi)) > self.yaw_tolerance:
                self.fail(f"spin yaw verification failed: {yaw:.3f}")
                return
            self.send_nav_goal("drop", self.drop)
            self.transition("NAVIGATE_TO_DROP")
        elif kind == "drop":
            self.drop_pose = self.pose
            self.stop_since = None
            self.transition("STOP_AT_DROP")

    def is_stopped(self):
        odom = self.odom_twist
        return (
            abs(odom.linear.x) < 0.01
            and abs(odom.linear.y) < 0.01
            and abs(odom.angular.z) < 0.01
        )

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def status_label(self, status):
        return {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"STATUS_{status}")

    def record_sample(self):
        if self.pose is None or self.mission_started is None:
            return
        self.samples.append({
            "t": (self.now() - self.mission_started).nanoseconds / 1e9,
            "x": self.pose.position.x,
            "y": self.pose.position.y,
            "yaw": quaternion_yaw(self.pose.orientation),
            "vx": self.last_cmd.linear.x,
            "vy": self.last_cmd.linear.y,
            "wz": self.last_cmd.angular.z,
            "state": self.state,
        })

    def fail(self, reason):
        if self.finished:
            return
        self.publish_stop()
        self.failure_state = self.state
        self.failure_reason = reason
        self.get_logger().error(f"State {self.state} failed: {reason}")
        self.transition("SAVE_RESULTS")

    def duration(self, kind):
        if kind not in self.accepted or kind not in self.results:
            return ""
        return (self.results[kind] - self.accepted[kind]).nanoseconds / 1e9

    def pose_metrics(self, pose, target):
        if pose is None:
            return "", ""
        return (
            math.hypot(pose.position.x - target[0], pose.position.y - target[1]),
            abs(angle_error(quaternion_yaw(pose.orientation), target[2])),
        )

    def write_results(self):
        final_pose = self.pose
        pickup_xy, _ = self.pose_metrics(self.pickup_pose, self.pickup)
        drop_xy, drop_yaw = self.pose_metrics(self.drop_pose or final_pose, self.drop)
        total = ""
        if self.mission_started and self.results.get("drop"):
            total = (self.results["drop"] - self.mission_started).nanoseconds / 1e9
        summary_path = self.results_dir / f"mission_{self.trial_id}.csv"
        fields = [
            "trial_id", "success", "failed_state", "failure_reason",
            "narrow_result", "narrow_distance_m", "narrow_duration_s",
            "pickup_result", "start_to_pickup_s", "pickup_wait_s",
            "rotation_result", "pickup_to_rotation_s", "spin_result",
            "spin_duration_s", "drop_result", "rotation_to_drop_s",
            "fixed_section_result", "fixed_segment_times_s",
            "fixed_stop_errors_m", "fixed_max_yaw_error_rad",
            "fixed_min_scan_m",
            "final_x", "final_y", "final_yaw", "pickup_xy_error_m",
            "drop_xy_error_m", "drop_yaw_error_rad", "total_duration_s",
            "startup_localization_wait_s",
        ]
        values = {
            "trial_id": self.trial_id,
            "success": not bool(self.failure_reason),
            "failed_state": self.failure_state,
            "failure_reason": self.failure_reason,
            "narrow_result": "SUCCEEDED" if self.narrow_distance is not None else "FAILED",
            "narrow_distance_m": self.narrow_distance or "",
            "narrow_duration_s": (
                (self.narrow_finished - self.narrow_started).nanoseconds / 1e9
                if self.narrow_started and self.narrow_finished else ""
            ),
            "pickup_result": self.result_status.get("pickup", ""),
            "start_to_pickup_s": self.duration("pickup"),
            "pickup_wait_s": self.pickup_wait_s if self.wait_started else "",
            "rotation_result": self.result_status.get("rotation", ""),
            "pickup_to_rotation_s": self.duration("rotation"),
            "spin_result": self.result_status.get("spin", ""),
            "spin_duration_s": self.duration("spin"),
            "drop_result": self.result_status.get("drop", ""),
            "rotation_to_drop_s": self.duration("drop"),
            "fixed_section_result": (
                "SUCCEEDED" if self.section_only and not self.failure_reason
                else "FAILED" if self.section_only else ""
            ),
            "fixed_segment_times_s": ";".join(
                f"{value:.3f}" for value in self.fixed_segment_times
            ),
            "fixed_stop_errors_m": ";".join(
                f"{value:.3f}" for value in self.fixed_stop_errors
            ),
            "fixed_max_yaw_error_rad": (
                f"{self.fixed_max_yaw_error:.6f}" if self.section_only else ""
            ),
            "fixed_min_scan_m": (
                f"{self.fixed_min_scan:.3f}"
                if self.fixed_min_scan is not None else ""
            ),
            "final_x": final_pose.position.x if final_pose else "",
            "final_y": final_pose.position.y if final_pose else "",
            "final_yaw": quaternion_yaw(final_pose.orientation) if final_pose else "",
            "pickup_xy_error_m": pickup_xy,
            "drop_xy_error_m": drop_xy,
            "drop_yaw_error_rad": drop_yaw,
            "total_duration_s": total,
            "startup_localization_wait_s": (
                (self.mission_started - self.node_started).nanoseconds / 1e9
                if self.mission_started else ""
            ),
        }
        with summary_path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerow(values)
        trajectory_path = self.results_dir / f"mission_{self.trial_id}_trajectory.csv"
        with trajectory_path.open("w", newline="") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=["t", "x", "y", "yaw", "vx", "vy", "wz", "state"]
            )
            writer.writeheader()
            writer.writerows(self.samples)
        report_path = self.results_dir / f"mission_{self.trial_id}.md"
        with report_path.open("w") as stream:
            stream.write("# Mission result\n\n")
            stream.write("| Item | Result |\n|---|---|\n")
            stream.write(f"| Success | {values['success']} |\n")
            stream.write(f"| Failed state | {self.failure_state or '-'} |\n")
            stream.write(f"| Failure reason | {self.failure_reason or '-'} |\n")
            stream.write(
                f"| Startup/localization wait | "
                f"{values['startup_localization_wait_s'] or '-'} s |\n"
            )
            stream.write(
                f"| Narrow section | {values['narrow_result']} / "
                f"{values['narrow_duration_s'] or '-'} s / "
                f"{values['narrow_distance_m'] or '-'} m |\n"
            )
            for kind in ("pickup", "rotation", "spin", "drop"):
                stream.write(
                    f"| {kind} action | {self.result_status.get(kind, '-')} / "
                    f"{self.duration(kind) or '-'} s |\n"
                )
            if self.section_only:
                stream.write(
                    f"| Fixed section | {values['fixed_section_result']} / "
                    f"{values['fixed_segment_times_s'] or '-'} s per segment |\n"
                )
                stream.write(
                    f"| Fixed stop errors | "
                    f"{values['fixed_stop_errors_m'] or '-'} m |\n"
                )
                stream.write(
                    f"| Fixed max yaw error | "
                    f"{values['fixed_max_yaw_error_rad'] or '-'} rad |\n"
                )
                stream.write(
                    f"| Minimum scan range | "
                    f"{values['fixed_min_scan_m'] or '-'} m |\n"
                )
            stream.write(f"| Pickup wait | {values['pickup_wait_s'] or '-'} s |\n")
            stream.write(f"| Final pose | {values['final_x']}, {values['final_y']}, "
                         f"{values['final_yaw']} |\n")
            stream.write(f"| Pickup XY error | {pickup_xy} m |\n")
            stream.write(f"| Drop XY error | {drop_xy} m |\n")
            stream.write(f"| Drop yaw error | {drop_yaw} rad |\n")
            stream.write(f"| Total duration | {total} s |\n")


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.fail("interrupted")
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
