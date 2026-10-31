"""Sakura round-trip mission with a holonomic path follower.

Runs while /autonomy_enabled is true (set by mode_manager):
  GO_SAKURA      straight to the nearest reachable waypoint, then along the
                 graph to the sakura stop (theta = 0, left arm face to zone)
  WAIT_SAKURA_1  sakura_wait_s
  BACKOFF_1 / ROTATE_1 / REAPPROACH_1
                 CCW 90 deg so the +y arm face points to the zone; backs off
                 first only if the in-place turn would hit a wall/keep-out
  WAIT_SAKURA_2  sakura_wait_s
  BACKOFF_2 / ROTATE_2 / RETURN
                 go along the graph to the return pose (default: in front of
                 common zones 1-2, theta = 180 deg). return_rotation selects
                 how 90 -> 180 deg is done: "in_place" (back off if needed,
                 turn, then travel) or "moving" (turn while travelling, in a
                 section where the whole turn is clear)
  DONE / FAILED / ABORTED
Paths are shortcut and filleted (trajectory.SmoothPath) and followed with the
planned speed as feed-forward plus a small capped lateral correction. Stops
and turns use a deceleration law that reaches zero exactly at the target.
The heading stays fixed while travelling. Commands go to /cmd_vel_auto.

Pose: TF map->odom (AMCL) composed with odom->base_footprint (EKF). AMCL
corrections arrive as steps, so they are blended into the control pose at a
limited rate (correction_rate_xy / correction_rate_yaw_deg).
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from omni_robot_description.geometry import (
    default_geometry_file, drive_wheel_layout, drive_wheel_radius,
    load_geometry, outline_polygon)
from omni_robot_mission.collision import OccupancyChecker
from omni_robot_mission.route_graph import RouteGraph
from omni_robot_mission.trajectory import (
    CommandShaper, FrictionLimit, SmoothPath, stopping_speed, wrap)


DEBUG_FIELDS = ["ref_x", "ref_y", "ref_v", "ref_yaw", "est_x", "est_y",
                "est_yaw", "raw_vx", "raw_vy", "raw_wz", "cmd_vx", "cmd_vy",
                "cmd_wz", "body_vx", "body_vy", "wheel_scale", "amcl_x",
                "amcl_y", "amcl_yaw", "pose_age"]

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """Pose a (x, y, yaw) followed by the relative pose b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            wrap(a[2] + b[2]))


def relative(a, b):
    """Pose of b seen from a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2]))


class Move:
    """One motion step: a path at a fixed heading or an in-place rotation."""

    def __init__(self, kind, points=None, yaw=None, hold=None,
                 tolerance=0.03, yaw_tolerance=math.radians(3.0), label=None,
                 yaw_start=None):
        self.kind = kind
        self.yaw_start = yaw if yaw_start is None else yaw_start
        self.label = label
        self.points = points
        self.yaw = yaw
        self.hold = hold
        self.tolerance = tolerance
        self.yaw_tolerance = yaw_tolerance
        self.path = None
        self.settled_since = None


class SakuraMission(Node):

    def __init__(self):
        super().__init__("sakura_mission")
        p = self.declare_parameter
        p("geometry_file", default_geometry_file())
        p("waypoints", [""])
        p("edges", [""])
        p("sakura_node", "sakura_stop")
        p("sakura_yaw_deg", 0.0)
        p("sakura_rotated_yaw_deg", 90.0)
        p("sakura_wait_s", 3.0)
        p("return_node", "common_front")
        p("return_x", -0.793)
        p("return_y", -2.019)
        p("return_yaw_deg", 180.0)
        p("return_rotation", "in_place")
        p("rotation_length_max", 1.2)
        p("rotation_length_min", 0.4)
        p("travel_yaw_deg", 0.0)
        p("rotation_keepout", [""])
        p("rotation_backoff_direction", [1.0, 0.0])
        p("rotation_backoff_max", 0.5)
        p("clearance_margin", 0.02)
        p("path_clearance_margin", 0.15)
        p("position_tolerance", 0.05)
        p("yaw_tolerance_deg", 2.0)
        p("return_position_tolerance", 0.05)
        p("return_yaw_tolerance_deg", 2.0)
        p("stop_speed", 0.02)
        p("stop_yaw_rate", 0.05)
        p("stop_confirm_s", 0.2)
        p("state_timeout_s", 30.0)
        p("max_speed", 2.0)
        p("max_accel", 3.0)
        p("max_lateral_accel", 3.5)
        p("max_yaw_rate", 3.0)
        p("max_yaw_accel", 6.0)
        p("wheel_speed_margin", 0.9)
        p("fillet_max_radius", 0.8)
        p("shortcut", True)
        p("lookahead_time", 0.05)
        p("stop_decel_ratio", 0.8)
        p("final_gain", 3.0)
        p("lateral_gain", 3.0)
        p("feedback_max_speed", 0.15)
        p("yaw_gain", 5.0)
        p("feedback_max_yaw_rate", 0.5)
        p("command_accel_ratio", 1.3)
        p("correction_rate_xy", 0.3)
        p("correction_rate_yaw_deg", 10.0)
        p("pose_latency_compensation", True)
        p("use_lidar_correction", True)
        p("pose_latency_max_s", 0.1)
        p("control_rate", 50.0)
        # Acceleration limits from the drive-wheel friction (FrictionLimit)
        p("friction_limit_enabled", False)
        p("mu_drive", 0.6)              # 仮値: drive-wheel friction coefficient (mu2)
        p("friction_safety", 0.8)       # 仮値: usable share of the friction
        p("yaw_friction_reserve", 0.15)  # 仮値: share of the friction kept for turning
        # No-progress watchdog: command given but the robot stays still
        p("stall_time_s", 3.0)
        p("stall_max_retries", 2)
        p("stall_release_s", 0.5)

        self.geometry = load_geometry(self.get_parameter("geometry_file").value)
        _, self.wheel_x, self.wheel_y, self.wheel_yaw = drive_wheel_layout(
            self.geometry)
        self.wheel_radius = drive_wheel_radius(self.geometry)
        self.max_wheel_speed = float(
            self.geometry["drive_wheels"]["max_wheel_speed"])
        self.graph = RouteGraph(
            [w for w in self.get_parameter("waypoints").value if w],
            [e for e in self.get_parameter("edges").value if e])

        self.update_friction()
        self.last_command = (0.0, 0.0, 0.0)
        self.stall_since = None
        self.stall_count = 0
        self.release_until = None
        self.checker = None
        self.enabled = False
        self.state = "IDLE"
        self.state_since = None
        self.moves = []
        self.move = None
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.body_twist = (0.0, 0.0, 0.0)
        self.frozen_map_odom = None
        self.saturation_steps = 0
        self.shaper = None
        self.filtered = None
        self.last_odom_pose = None
        self.debug = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_auto", 10)
        self.state_pub = self.create_publisher(String, "/mission/state", LATCHED)
        self.path_pub = self.create_publisher(Path, "/mission/path", LATCHED)
        self.target_pub = self.create_publisher(
            PoseStamped, "/mission/target", LATCHED)
        # Diagnostics, one message per control step (see DEBUG_FIELDS)
        self.debug_pub = self.create_publisher(
            Float64MultiArray, "/mission/debug", 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, LATCHED)
        self.create_subscription(
            Bool, "/autonomy_enabled", self.on_enabled, LATCHED)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.dt = 1.0 / float(self.get_parameter("control_rate").value)
        self.timer = self.create_timer(self.dt, self.tick)
        self.publish_state("IDLE")

    # ------------------------------------------------------------ inputs
    def param(self, name):
        return self.get_parameter(name).value

    def update_friction(self):
        """(Re)build the friction limit from the parameters; called at the
        start of each mission so that parameters set at run time apply."""
        mass = float(self.geometry["mass"]["total"])
        chassis = self.geometry["chassis"]
        self.friction = FrictionLimit(
            bool(self.param("friction_limit_enabled")),
            float(self.param("mu_drive")), float(self.param("friction_safety")),
            float(self.param("yaw_friction_reserve")), mass,
            # box approximation of the yaw inertia of the chassis
            mass * (chassis["length"] ** 2 + chassis["width"] ** 2) / 12.0,
            float(self.geometry["drive_wheels"]["center_distance"]))

    def limit(self, name):
        """max_accel / max_lateral_accel / max_yaw_accel after the friction
        limit (diagonal value for the direction-independent uses)."""
        value = float(self.param(name))
        if name == "max_yaw_accel":
            return min(value, self.friction.yaw_accel)
        return min(value, self.friction.min_accel())

    def on_map(self, msg):
        keepout = [tuple(float(v) for v in k.split())
                   for k in self.param("rotation_keepout") if k]
        self.checker = OccupancyChecker(
            msg, outline_polygon(self.geometry),
            margin=float(self.param("clearance_margin")), keepout=keepout)
        travel = math.radians(self.param("travel_yaw_deg"))
        blocked = self.graph.check_edges(
            lambda a, b: self.checker.segment_clear(a, b, travel))
        for a, b in blocked:
            self.get_logger().warn(f"edge {a} - {b} is not clear of walls")
        self.get_logger().info(
            f"map received; {len(self.graph.nodes)} waypoints, "
            f"{len(blocked)} blocked edges")

    def on_odom(self, msg):
        t = msg.twist.twist
        self.speed = math.hypot(t.linear.x, t.linear.y)
        self.yaw_rate = t.angular.z
        self.body_twist = (t.linear.x, t.linear.y, t.angular.z)

    def on_enabled(self, msg):
        if msg.data and not self.enabled:
            self.enabled = True
            self.start_mission()
        elif not msg.data and self.enabled:
            self.enabled = False
            if self.state not in ("IDLE", "DONE", "FAILED"):
                self.publish_state("ABORTED")
            self.moves, self.move = [], None
            self.cmd_pub.publish(Twist())

    def lookup(self, parent, child, with_stamp=False):
        try:
            msg = self.tf_buffer.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        t = msg.transform
        pose = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        if with_stamp:
            return pose, Time.from_msg(msg.header.stamp)
        return pose

    def odom_pose_now(self):
        """odom -> base_footprint, moved forward by the odometry twist over
        the age of the latest transform (EKF period + delivery)."""
        found = self.lookup("odom", "base_footprint", with_stamp=True)
        if found is None:
            return None
        pose, stamp = found
        if not self.param("pose_latency_compensation"):
            return pose
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        age = max(0.0, min(float(self.param("pose_latency_max_s")), age))
        self.debug.update(pose_age=age)
        vx, vy, wz = self.body_twist
        return compose(pose, (vx * age, vy * age, wz * age))

    def update_pose(self, reset=False):
        """Control pose: odometry motion + AMCL correction at a limited rate."""
        map_odom = self.lookup("map", "odom")
        odom_base = self.odom_pose_now()
        if map_odom is None or odom_base is None:
            return self.filtered
        # Without LiDAR correction the pose is odometry only, anchored where
        # the mission started.
        if reset or self.frozen_map_odom is None:
            self.frozen_map_odom = map_odom
        if not self.param("use_lidar_correction"):
            map_odom = self.frozen_map_odom
        raw = compose(map_odom, odom_base)
        self.debug.update(amcl_x=raw[0], amcl_y=raw[1], amcl_yaw=raw[2])
        if reset or self.filtered is None or self.last_odom_pose is None:
            self.filtered = raw
        else:
            predicted = compose(self.filtered,
                                relative(self.last_odom_pose, odom_base))
            dx, dy = raw[0] - predicted[0], raw[1] - predicted[1]
            dyaw = wrap(raw[2] - predicted[2])
            step = float(self.param("correction_rate_xy")) * self.dt
            norm = math.hypot(dx, dy)
            if norm > step:
                dx, dy = dx * step / norm, dy * step / norm
            yaw_step = math.radians(self.param("correction_rate_yaw_deg")) * self.dt
            dyaw = max(-yaw_step, min(yaw_step, dyaw))
            self.filtered = (predicted[0] + dx, predicted[1] + dy,
                             wrap(predicted[2] + dyaw))
        self.last_odom_pose = odom_base
        return self.filtered

    # ------------------------------------------------------------ states
    def publish_state(self, state, detail=""):
        self.state = state
        self.state_since = self.get_clock().now()
        self.state_pub.publish(String(data=f"{state} {detail}".strip()))
        self.get_logger().info(f"state {state} {detail}")

    def fail(self, reason):
        self.moves, self.move = [], None
        self.cmd_pub.publish(Twist())
        self.publish_state("FAILED", reason)

    def start_mission(self):
        self.saturation_steps = 0
        self.update_friction()
        self.stall_since, self.stall_count, self.release_until = None, 0, None
        if self.friction.enabled:
            self.get_logger().info(
                f"friction limit: mu={self.param('mu_drive')}, translation "
                f"{self.friction.min_accel():.2f} m/s^2 (diagonal) to "
                f"{self.friction.min_accel() * math.sqrt(2.0):.2f} m/s^2 (face), "
                f"yaw {self.friction.yaw_accel:.2f} rad/s^2")
        ratio = float(self.param("command_accel_ratio"))
        # The plan turns on arcs with up to max_lateral_accel, which may be
        # larger than max_accel; the command must be able to follow that.
        accel = max(float(self.param("max_accel")),
                    float(self.param("max_lateral_accel")))
        if self.friction.enabled:  # no direction can exceed the face value
            accel = min(accel, self.friction.min_accel() * math.sqrt(2.0))
        self.shaper = CommandShaper(
            float(self.param("max_speed")) * 1.05,
            accel * ratio,
            float(self.param("max_yaw_rate")) * 1.05,
            self.limit("max_yaw_accel") * ratio)
        if self.checker is None:
            self.fail("no map")
            return
        pose = self.update_pose(reset=True)
        if pose is None:
            self.fail("no pose (TF map -> base_footprint)")
            return
        yaw = math.radians(self.param("sakura_yaw_deg"))
        points = self.graph_route(pose, self.param("sakura_node"))
        if points is None:
            self.fail("no reachable waypoint from the start pose")
            return
        self.moves = [self.path_move(points, yaw, precise=True, label="GO_SAKURA")]
        self.next_state("GO_SAKURA")

    def graph_route(self, pose, goal_node, yaws=None):
        x, y, yaw = pose
        headings = {yaw} | set(yaws or (math.radians(self.param("travel_yaw_deg")),))

        def clear(a, b):
            return all(self.checker.segment_clear(a, b, h) for h in headings)

        entry = self.graph.nearest_reachable(x, y, clear)
        if entry is None:
            return None
        names = self.graph.shortest_path(entry, goal_node)
        if names is None:
            return None
        self.get_logger().info(f"route: start -> {' -> '.join(names)}")
        return [(x, y)] + [self.graph.nodes[n] for n in names]

    def path_move(self, points, yaw, precise, label=None):
        if precise:
            tolerance = float(self.param("position_tolerance"))
            yaw_tol = math.radians(self.param("yaw_tolerance_deg"))
        else:
            tolerance, yaw_tol = 0.03, math.radians(3.0)
        return Move("path", points=points, yaw=yaw, tolerance=tolerance,
                    yaw_tolerance=yaw_tol, label=label)

    def rotate_move(self, x, y, yaw_to, label=None):
        return Move("rotate", yaw=yaw_to, hold=(x, y), tolerance=0.03,
                    yaw_tolerance=math.radians(self.param("yaw_tolerance_deg")),
                    label=label)

    def rotation_plan(self, x, y, yaw_from, yaw_to):
        """Moves turning from yaw_from to yaw_to near (x, y) without contact.

        Returns (moves_before_turn, turn, turn_point) or None.
        """
        if self.checker.rotation_clear(x, y, yaw_from, yaw_to):
            return [], self.rotate_move(x, y, yaw_to), (x, y)
        dx, dy = self.param("rotation_backoff_direction")
        norm = math.hypot(dx, dy)
        dx, dy = dx / norm, dy / norm
        steps = int(float(self.param("rotation_backoff_max")) / 0.01)
        for k in range(1, steps + 1):
            bx, by = x + 0.01 * k * dx, y + 0.01 * k * dy
            if (self.checker.segment_clear((x, y), (bx, by), yaw_from)
                    and self.checker.rotation_clear(bx, by, yaw_from, yaw_to)):
                self.get_logger().info(
                    f"rotation needs a {0.01 * k:.2f} m back-off")
                return ([self.path_move([(x, y), (bx, by)], yaw_from, False)],
                        self.rotate_move(bx, by, yaw_to), (bx, by))
        return None

    def next_state(self, state):
        self.publish_state(state)

    # ------------------------------------------------------------ control
    def tick(self):
        if not self.enabled or self.state in ("IDLE", "DONE", "FAILED", "ABORTED"):
            return
        pose = self.update_pose()
        elapsed = (self.get_clock().now() - self.state_since).nanoseconds * 1e-9
        if elapsed > float(self.param("state_timeout_s")):
            self.fail(f"timeout in {self.state}")
            return
        if self.state.startswith("WAIT"):
            self.cmd_pub.publish(Twist())
            if elapsed >= float(self.param("sakura_wait_s")):
                self.after_wait()
            return
        if pose is None:
            self.cmd_pub.publish(Twist())
            return
        if self.move is None:
            if not self.moves:
                self.after_moves()
                return
            self.move = self.moves.pop(0)
            if not self.activate(self.move, pose):
                return
            if self.move.label and self.move.label != self.state:
                self.next_state(self.move.label)
        if self.release_until is not None:
            if self.get_clock().now() < self.release_until:
                self.cmd_pub.publish(Twist())
                return
            self.release_until = None
        if self.control(self.move, pose):
            self.move = None
            self.stall_since, self.stall_count = None, 0
            if not self.moves:
                self.after_moves()
        else:
            self.check_stall(pose)

    def check_stall(self, pose):
        """No-progress watchdog: a command is given but the robot stays still.
        First tries to recover (zero command for stall_release_s, then re-plan
        from the current pose); after stall_max_retries, FAILED with the cause."""
        vx, vy, wz = self.last_command
        commanded = math.hypot(vx, vy) > 0.03 or abs(wz) > 0.1
        still = self.speed < 0.01 and abs(self.yaw_rate) < 0.02
        now = self.get_clock().now()
        if not (commanded and still):
            self.stall_since = None
            return
        if self.stall_since is None:
            self.stall_since = now
        stalled = (now - self.stall_since).nanoseconds * 1e-9
        if stalled < float(self.param("stall_time_s")):
            return
        move = self.move
        if move.kind == "path":
            gx, gy = move.path.goal
        else:
            gx, gy = move.hold
        cause = (f"no motion for {stalled:.1f} s in {self.state} although "
                 f"commanded (vx,vy,wz)=({vx:.3f},{vy:.3f},{wz:.3f}); "
                 f"goal error (dx,dy,dyaw)=({gx - pose[0]:.3f},{gy - pose[1]:.3f},"
                 f"{wrap(move.yaw - pose[2]):.3f}); "
                 f"mu_drive={self.param('mu_drive')}")
        self.stall_count += 1
        self.stall_since = None
        if self.stall_count > int(self.param("stall_max_retries")):
            self.fail("stalled: " + cause)
            return
        self.get_logger().warn(
            f"stall {self.stall_count}/{self.param('stall_max_retries')}: "
            f"{cause}; releasing and re-planning")
        self.shaper.reset()
        self.cmd_pub.publish(Twist())
        self.release_until = now + Duration(
            seconds=float(self.param("stall_release_s")))
        if move.kind == "path":
            self.activate(move, pose)

    def speed_limit(self, direction, yaw_per_m):
        """Max speed in a body-frame direction with every wheel <= its limit,
        including the wheel speed used to turn at yaw_per_m (rad per m)."""
        peak = 0.0
        for wx, wy, wyaw in zip(self.wheel_x, self.wheel_y, self.wheel_yaw):
            lever = -math.cos(wyaw) * wy + math.sin(wyaw) * wx
            peak = max(peak, abs(math.cos(direction - wyaw) + yaw_per_m * lever))
        return (float(self.param("wheel_speed_margin")) * self.max_wheel_speed
                * self.wheel_radius / max(peak, 1e-6))

    def activate(self, move, pose):
        """Build the smoothed path from the actual pose when a move starts."""
        if move.kind == "path":
            points = [(pose[0], pose[1])] + list(move.points[1:])
            margin = float(self.param("path_clearance_margin"))
            sweep = self.checker.sweep_radius() + margin
            try:
                move.path = SmoothPath(
                    points, move.yaw_start,
                    lambda x, y, yaw: not self.checker.pose_collides(
                        x, y, yaw, margin=margin),
                    float(self.param("max_speed")), float(self.param("max_accel")),
                    float(self.param("max_lateral_accel")),
                    self.speed_limit, start_speed=self.speed,
                    max_radius=float(self.param("fillet_max_radius")),
                    shortcut=bool(self.param("shortcut")), yaw_end=move.yaw,
                    rotation_clear=lambda x, y: self.checker.circle_clear(x, y, sweep),
                    rotation_length=float(self.param("rotation_length_max")),
                    rotation_min_length=float(self.param("rotation_length_min")),
                    max_yaw_rate=float(self.param("max_yaw_rate")),
                    max_yaw_accel=self.limit("max_yaw_accel"),
                    friction=self.friction)
            except (ValueError, ZeroDivisionError) as error:
                self.fail(f"path smoothing failed: {error}")
                return False
            path = move.path
            self.publish_path([(s[0], s[1]) for s in path.samples[::5]]
                              + [path.goal])
            turn = (f", turn in s=[{path.rotation_section[0]:.2f}, "
                    f"{path.rotation_section[1]:.2f}] m" if path.rotation_section else "")
            self.get_logger().info(
                f"path {move.label or ''}: {len(path.corners)} corners after "
                f"shortcut, fillet radii {path.radii}, length {path.length:.2f} m, "
                f"planned {path.planned_time():.2f} s{turn}")
            self.publish_target(path.goal[0], path.goal[1], move.yaw)
        else:
            self.publish_target(move.hold[0], move.hold[1], move.yaw)
        return True

    def control(self, move, pose):
        """One control step. Returns True when the move is complete."""
        x, y, yaw = pose
        decel_ratio = float(self.param("stop_decel_ratio"))
        fb_max = float(self.param("feedback_max_speed"))
        if move.kind == "path":
            path = move.path
            i = path.project(x, y)
            px, py, tx, ty, _, s = path.samples[i]
            remaining = path.length - s
            gx, gy = path.goal
            if remaining > 0.02:
                ahead = max(0.01, self.speed * float(self.param("lookahead_time")))
                j = path.sample_at(s + ahead)
                v_ff = path.speed[j]
                yaw_ref = path.yaws[j]
                yaw_ff = v_ff * path.yaw_rate_per_m[j]
                fx, fy = path.samples[j][2], path.samples[j][3]
                # lateral (cross-track) correction only, capped
                ex, ey = px - x, py - y
                along = ex * tx + ey * ty
                lx, ly = ex - along * tx, ey - along * ty
                gain = float(self.param("lateral_gain"))
                cx, cy = gain * lx, gain * ly
                norm = math.hypot(cx, cy)
                if norm > fb_max:
                    cx, cy = cx * fb_max / norm, cy * fb_max / norm
                vx, vy = v_ff * fx + cx, v_ff * fy + cy
                self.debug.update(ref_x=px, ref_y=py, ref_v=v_ff)
            else:
                # final approach: speed that is zero exactly at the goal
                d = math.hypot(gx - x, gy - y)
                speed = stopping_speed(
                    d, decel_ratio * self.limit("max_accel"),
                    float(self.param("final_gain")))
                vx, vy = ((gx - x) / d * speed, (gy - y) / d * speed) if d > 1e-6 \
                    else (0.0, 0.0)
                yaw_ref, yaw_ff = move.yaw, 0.0
                self.debug.update(ref_x=gx, ref_y=gy, ref_v=0.0)
            yaw_err = wrap(yaw_ref - yaw)
            wz = yaw_ff + max(-float(self.param("feedback_max_yaw_rate")),
                              min(float(self.param("feedback_max_yaw_rate")),
                                  float(self.param("yaw_gain")) * yaw_err))
            target = (gx, gy, move.yaw)
            near_end = remaining <= 0.02
        else:
            hx, hy = move.hold
            yaw_err = wrap(move.yaw - yaw)
            rate = stopping_speed(
                abs(yaw_err), decel_ratio * self.limit("max_yaw_accel"),
                float(self.param("yaw_gain")))
            wz = math.copysign(min(rate, float(self.param("max_yaw_rate"))), yaw_err)
            # hold the position with a small capped correction
            gain = float(self.param("final_gain"))
            vx, vy = gain * (hx - x), gain * (hy - y)
            norm = math.hypot(vx, vy)
            if norm > fb_max:
                vx, vy = vx * fb_max / norm, vy * fb_max / norm
            target = (hx, hy, move.yaw)
            near_end = True
            self.debug.update(ref_x=hx, ref_y=hy, ref_v=0.0)
        self.debug.update(ref_yaw=yaw_ref if move.kind == "path" else move.yaw,
                          est_x=x, est_y=y, est_yaw=yaw,
                          raw_vx=vx, raw_vy=vy, raw_wz=wz)

        vx, vy, wz = self.shaper.shape(
            vx, vy, wz, self.dt, yaw,
            # path following keeps the command_accel_ratio margin; stop,
            # re-approach and hold corrections are capped by the friction
            self.friction if (move.kind != "path" or near_end) else None)
        self.last_command = (vx, vy, wz)
        self.publish_command(vx, vy, wz, yaw)

        ok = (near_end
              and abs(target[0] - x) <= move.tolerance
              and abs(target[1] - y) <= move.tolerance
              and abs(wrap(target[2] - yaw)) <= move.yaw_tolerance
              and self.speed <= float(self.param("stop_speed"))
              and abs(self.yaw_rate) <= float(self.param("stop_yaw_rate")))
        now = self.get_clock().now()
        if not ok:
            move.settled_since = None
            return False
        if move.settled_since is None:
            move.settled_since = now
        if (now - move.settled_since).nanoseconds * 1e-9 >= float(
                self.param("stop_confirm_s")):
            self.shaper.reset()
            self.cmd_pub.publish(Twist())
            return True
        return False

    def after_wait(self):
        goal = self.graph.nodes[self.param("sakura_node")]
        if self.state == "WAIT_SAKURA_1":
            yaw0 = math.radians(self.param("sakura_yaw_deg"))
            yaw1 = math.radians(self.param("sakura_rotated_yaw_deg"))
            plan = self.rotation_plan(goal[0], goal[1], yaw0, yaw1)
            if plan is None:
                self.fail("no clear place to rotate at the sakura zone")
                return
            before, turn, point = plan
            for move in before:
                move.label = "BACKOFF_1"
            turn.label = "ROTATE_1"
            self.moves = before + [turn]
            if before:
                self.moves.append(self.path_move(
                    [point, goal], yaw1, True, label="REAPPROACH_1"))
            self.next_state(self.moves[0].label)
        elif self.state == "WAIT_SAKURA_2":
            yaw1 = math.radians(self.param("sakura_rotated_yaw_deg"))
            final = math.radians(self.param("return_yaw_deg"))
            if self.param("return_rotation") == "moving":
                self.start_return(yaw1)
            elif abs(wrap(final - yaw1)) > 1e-3:
                plan = self.rotation_plan(goal[0], goal[1], yaw1, final)
                if plan is None:
                    self.fail("no clear place to turn back")
                    return
                before, turn, _ = plan
                for move in before:
                    move.label = "BACKOFF_2"
                turn.label = "ROTATE_2"
                self.moves = before + [turn]
                self.next_state(self.moves[0].label)
            else:
                self.start_return(final)

    def after_moves(self):
        if self.state == "GO_SAKURA":
            self.next_state("WAIT_SAKURA_1")
        elif self.state in ("BACKOFF_1", "ROTATE_1", "REAPPROACH_1"):
            self.next_state("WAIT_SAKURA_2")
        elif self.state in ("BACKOFF_2", "ROTATE_2"):
            self.start_return(math.radians(self.param("return_yaw_deg")))
        elif self.state == "RETURN":
            self.cmd_pub.publish(Twist())
            self.publish_state(
                "DONE", f"wheel_speed_limited_steps={self.saturation_steps}")

    def start_return(self, yaw_start):
        """Graph route to the return pose; the heading goes from yaw_start to
        return_yaw_deg (turning along the path when they differ)."""
        pose = self.filtered
        final_yaw = math.radians(self.param("return_yaw_deg"))
        points = self.graph_route(pose, self.param("return_node"),
                                  yaws=(yaw_start, final_yaw))
        if points is None:
            self.fail("no route back")
            return
        goal = (float(self.param("return_x")), float(self.param("return_y")))
        if math.hypot(goal[0] - points[-1][0], goal[1] - points[-1][1]) > 1e-3:
            points.append(goal)
        move = self.path_move(points, final_yaw, True, label="RETURN")
        move.yaw_start = yaw_start
        move.tolerance = float(self.param("return_position_tolerance"))
        move.yaw_tolerance = math.radians(self.param("return_yaw_tolerance_deg"))
        self.moves = [move]
        self.next_state("RETURN")

    def publish_command(self, vx, vy, wz, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = c * vx + s * vy, -s * vx + c * vy
        # keep the direction when a wheel would exceed its speed limit
        peak = 0.0
        for wx, wy, wyaw in zip(self.wheel_x, self.wheel_y, self.wheel_yaw):
            cw, sw = math.cos(wyaw), math.sin(wyaw)
            wheel = (cw * bx + sw * by + wz * (-cw * wy + sw * wx)) / self.wheel_radius
            peak = max(peak, abs(wheel))
        scale = 1.0
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            bx, by, wz = bx * scale, by * scale, wz * scale
            self.saturation_steps += 1
        self.debug.update(cmd_vx=vx, cmd_vy=vy, cmd_wz=wz, body_vx=bx,
                          body_vy=by, wheel_scale=scale)
        self.debug_pub.publish(Float64MultiArray(data=[
            self.get_clock().now().nanoseconds * 1e-9]
            + [float(self.debug.get(k, float("nan"))) for k in DEBUG_FIELDS]))
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = bx, by, wz
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------ visuals
    def publish_path(self, points):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for px, py in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = px, py
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_target(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)
        self.target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SakuraMission()
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
