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
                 turn back to the travel heading (where it is clear) and go
                 along the graph to the return pose
  DONE / FAILED / ABORTED
The heading stays fixed while travelling. Commands go to /cmd_vel_auto; the
pose comes from TF map -> base_footprint (AMCL + EKF). When autonomy is
switched off the motion is dropped at once and a zero command is sent.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from omni_robot_description.geometry import (
    default_geometry_file, drive_wheel_layout, drive_wheel_radius,
    load_geometry, outline_polygon)
from omni_robot_mission.collision import OccupancyChecker
from omni_robot_mission.route_graph import RouteGraph
from omni_robot_mission.trajectory import (
    CommandShaper, PathMotion, YawMotion, wrap)


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Move:
    """One motion step: a path at a fixed heading or an in-place rotation."""

    def __init__(self, kind, path=None, yaw=None, rotation=None, hold=None,
                 tolerance=0.03, yaw_tolerance=math.radians(3.0), label=None):
        self.kind = kind
        self.label = label
        self.path = path
        self.yaw = yaw
        self.rotation = rotation
        self.hold = hold
        self.tolerance = tolerance
        self.yaw_tolerance = yaw_tolerance
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
        p("return_node", "dice_front")
        p("return_x", -1.055)
        p("return_y", -3.112)
        p("return_yaw_deg", 0.0)
        p("travel_yaw_deg", 0.0)
        p("rotation_keepout", [""])
        p("rotation_backoff_direction", [1.0, 0.0])
        p("rotation_backoff_max", 0.5)
        p("clearance_margin", 0.02)
        p("position_tolerance", 0.05)
        p("yaw_tolerance_deg", 2.0)
        p("return_position_tolerance", 0.05)
        p("return_yaw_tolerance_deg", 2.0)
        p("stop_speed", 0.02)
        p("stop_confirm_s", 0.2)
        p("state_timeout_s", 30.0)
        p("max_speed", 1.0)
        p("max_accel", 2.0)
        p("max_yaw_rate", 3.0)
        p("max_yaw_accel", 6.0)
        p("corner_deviation", 0.02)
        p("position_gain", 4.0)
        p("yaw_gain", 5.0)
        p("command_accel_ratio", 1.3)
        p("control_rate", 50.0)

        self.geometry = load_geometry(self.get_parameter("geometry_file").value)
        _, self.wheel_x, self.wheel_y, self.wheel_yaw = drive_wheel_layout(
            self.geometry)
        self.wheel_radius = drive_wheel_radius(self.geometry)
        self.max_wheel_speed = float(
            self.geometry["drive_wheels"]["max_wheel_speed"])
        self.graph = RouteGraph(
            [w for w in self.get_parameter("waypoints").value if w],
            [e for e in self.get_parameter("edges").value if e])

        self.checker = None
        self.enabled = False
        self.state = "IDLE"
        self.state_since = None
        self.moves = []
        self.move = None
        self.wait_until = None
        self.speed = 0.0
        self.saturation_steps = 0
        self.shaper = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_auto", 10)
        self.state_pub = self.create_publisher(String, "/mission/state", LATCHED)
        self.path_pub = self.create_publisher(Path, "/mission/path", LATCHED)
        self.target_pub = self.create_publisher(
            PoseStamped, "/mission/target", LATCHED)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, LATCHED)
        self.create_subscription(
            Bool, "/autonomy_enabled", self.on_enabled, LATCHED)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.tick)
        self.publish_state("IDLE")

    # ------------------------------------------------------------ inputs
    def param(self, name):
        return self.get_parameter(name).value

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

    def pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", Time())
        except TransformException:
            return None
        t = tf.transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

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
        self.shaper = CommandShaper(
            float(self.param("max_speed")) * 1.2,
            float(self.param("max_accel")) * float(self.param("command_accel_ratio")),
            float(self.param("max_yaw_rate")) * 1.2,
            float(self.param("max_yaw_accel")) * float(self.param("command_accel_ratio")))
        if self.checker is None:
            self.fail("no map")
            return
        pose = self.pose()
        if pose is None:
            self.fail("no pose (TF map -> base_footprint)")
            return
        goal = self.graph.nodes[self.param("sakura_node")]
        yaw = math.radians(self.param("sakura_yaw_deg"))
        points = self.graph_route(pose, self.param("sakura_node"))
        if points is None:
            self.fail("no reachable waypoint from the start pose")
            return
        self.publish_target(goal[0], goal[1], yaw)
        self.moves = [self.path_move(points, yaw, precise=True)]
        self.next_state("GO_SAKURA")

    def graph_route(self, pose, goal_node):
        x, y, yaw = pose
        travel = math.radians(self.param("travel_yaw_deg"))

        def clear(a, b):
            return (self.checker.segment_clear(a, b, yaw)
                    and self.checker.segment_clear(a, b, travel))

        entry = self.graph.nearest_reachable(x, y, clear)
        if entry is None:
            return None
        names = self.graph.shortest_path(entry, goal_node)
        if names is None:
            return None
        self.get_logger().info(f"route: start -> {' -> '.join(names)}")
        points = [(x, y)] + [self.graph.nodes[n] for n in names]
        if math.hypot(points[1][0] - x, points[1][1] - y) < 1e-3:
            points.pop(0)
        self.publish_path(points)
        return points

    def path_move(self, points, yaw, precise):
        tolerance = float(self.param("position_tolerance")) if precise else 0.03
        yaw_tol = math.radians(self.param("yaw_tolerance_deg")) if precise \
            else math.radians(3.0)
        return Move("path", path=PathMotion(
            points, float(self.param("max_speed")), float(self.param("max_accel")),
            float(self.param("corner_deviation"))), yaw=yaw,
            tolerance=tolerance, yaw_tolerance=yaw_tol)

    def rotate_move(self, x, y, yaw_from, yaw_to):
        return Move("rotate", rotation=YawMotion(
            yaw_from, yaw_to, float(self.param("max_yaw_rate")),
            float(self.param("max_yaw_accel"))), hold=(x, y),
            tolerance=0.03,
            yaw_tolerance=math.radians(self.param("yaw_tolerance_deg")))

    def rotation_plan(self, x, y, yaw_from, yaw_to):
        """Moves turning from yaw_from to yaw_to near (x, y) without contact.

        Returns (moves_before_turn, turn, turn_point) or None.
        """
        if self.checker.rotation_clear(x, y, yaw_from, yaw_to):
            return [], self.rotate_move(x, y, yaw_from, yaw_to), (x, y)
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
                        self.rotate_move(bx, by, yaw_from, yaw_to), (bx, by))
        return None

    def next_state(self, state):
        self.publish_state(state)

    # ------------------------------------------------------------ control
    def tick(self):
        if not self.enabled or self.state in ("IDLE", "DONE", "FAILED", "ABORTED"):
            return
        elapsed = (self.get_clock().now() - self.state_since).nanoseconds * 1e-9
        if elapsed > float(self.param("state_timeout_s")):
            self.fail(f"timeout in {self.state}")
            return
        if self.state.startswith("WAIT"):
            self.cmd_pub.publish(Twist())
            if elapsed >= float(self.param("sakura_wait_s")):
                self.after_wait()
            return
        pose = self.pose()
        if pose is None:
            self.cmd_pub.publish(Twist())
            return
        if self.move is None:
            if not self.moves:
                self.after_moves(pose)
                return
            self.move = self.moves.pop(0)
            if self.move.label and self.move.label != self.state:
                self.next_state(self.move.label)
        if self.control(self.move, pose):
            self.move = None
            if not self.moves:
                self.after_moves(pose)

    def after_wait(self):
        pose = self.pose()
        if pose is None:
            return
        x, y, _ = pose
        goal = self.graph.nodes[self.param("sakura_node")]
        if self.state == "WAIT_SAKURA_1":
            yaw0 = math.radians(self.param("sakura_yaw_deg"))
            yaw1 = math.radians(self.param("sakura_rotated_yaw_deg"))
            plan = self.rotation_plan(goal[0], goal[1], yaw0, yaw1)
            if plan is None:
                self.fail("no clear place to rotate at the sakura zone")
                return
            before, turn, point = plan
            self.publish_target(goal[0], goal[1], yaw1)
            for move in before:
                move.label = "BACKOFF_1"
            turn.label = "ROTATE_1"
            self.moves = before + [turn]
            if before:
                back = self.path_move([point, goal], yaw1, True)
                back.label = "REAPPROACH_1"
                self.moves.append(back)
            self.next_state(self.moves[0].label)
        elif self.state == "WAIT_SAKURA_2":
            yaw1 = math.radians(self.param("sakura_rotated_yaw_deg"))
            travel = math.radians(self.param("travel_yaw_deg"))
            if abs(wrap(travel - yaw1)) > 1e-3:
                plan = self.rotation_plan(goal[0], goal[1], yaw1, travel)
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
                self.moves = []
                self.start_return()

    def after_moves(self, pose):
        if self.state == "GO_SAKURA":
            self.next_state("WAIT_SAKURA_1")
        elif self.state in ("BACKOFF_1", "ROTATE_1", "REAPPROACH_1"):
            self.next_state("WAIT_SAKURA_2")
        elif self.state in ("BACKOFF_2", "ROTATE_2"):
            self.start_return()
        elif self.state == "RETURN":
            self.cmd_pub.publish(Twist())
            self.publish_state(
                "DONE", f"wheel_speed_limited_steps={self.saturation_steps}")

    def start_return(self):
        pose = self.pose()
        travel = math.radians(self.param("travel_yaw_deg"))
        points = self.graph_route(pose, self.param("return_node"))
        if points is None:
            self.fail("no route back")
            return
        goal = (float(self.param("return_x")), float(self.param("return_y")))
        if math.hypot(goal[0] - points[-1][0], goal[1] - points[-1][1]) > 1e-3:
            points.append(goal)
        final_yaw = math.radians(self.param("return_yaw_deg"))
        self.publish_target(goal[0], goal[1], final_yaw)
        move = self.path_move(points, travel, True)
        move.tolerance = float(self.param("return_position_tolerance"))
        move.yaw_tolerance = math.radians(self.param("return_yaw_tolerance_deg"))
        self.moves = [move]
        if abs(wrap(final_yaw - travel)) > 1e-3:
            if not self.checker.rotation_clear(goal[0], goal[1], travel, final_yaw):
                self.fail("return yaw cannot be reached without contact")
                return
            self.moves.append(self.rotate_move(goal[0], goal[1], travel, final_yaw))
        self.publish_path(points)
        self.next_state("RETURN")

    def control(self, move, pose):
        """One control step. Returns True when the move is complete."""
        x, y, yaw = pose
        kp = float(self.param("position_gain"))
        ky = float(self.param("yaw_gain"))
        if move.kind == "path":
            motion = move.path
            motion.step(self.dt)
            (rx, ry), (tx, ty), v = motion.reference()
            vx = v * tx + kp * (rx - x)
            vy = v * ty + kp * (ry - y)
            if motion.finished:
                # never faster than what still stops within the remaining error
                d = math.hypot(rx - x, ry - y)
                limit = math.sqrt(2.0 * float(self.param("max_accel")) * d)
                speed = math.hypot(vx, vy)
                if speed > limit > 0.0:
                    vx, vy = vx * limit / speed, vy * limit / speed
            wz = ky * wrap(move.yaw - yaw)
            target = (motion.goal[0], motion.goal[1], move.yaw)
            done_ref = motion.finished
        else:
            motion = move.rotation
            motion.step(self.dt)
            ref_yaw, rate = motion.reference()
            hx, hy = move.hold
            vx, vy = kp * (hx - x), kp * (hy - y)
            wz = rate + ky * wrap(ref_yaw - yaw)
            target = (hx, hy, motion.goal_yaw)
            done_ref = motion.finished

        vx, vy, wz = self.shaper.shape(vx, vy, wz, self.dt)
        self.publish_command(vx, vy, wz, yaw)

        if not done_ref:
            move.settled_since = None
            return False
        ok = (abs(target[0] - x) <= move.tolerance
              and abs(target[1] - y) <= move.tolerance
              and abs(wrap(target[2] - yaw)) <= move.yaw_tolerance
              and self.speed <= float(self.param("stop_speed")))
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

    def publish_command(self, vx, vy, wz, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = c * vx + s * vy, -s * vx + c * vy
        # keep the direction when a wheel would exceed its speed limit
        peak = 0.0
        for wx, wy, wyaw in zip(self.wheel_x, self.wheel_y, self.wheel_yaw):
            cw, sw = math.cos(wyaw), math.sin(wyaw)
            wheel = (cw * bx + sw * by + wz * (-cw * wy + sw * wx)) / self.wheel_radius
            peak = max(peak, abs(wheel))
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            bx, by, wz = bx * scale, by * scale, wz * scale
            self.saturation_steps += 1
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
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
