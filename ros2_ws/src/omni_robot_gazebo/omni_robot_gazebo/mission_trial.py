"""Simulation-only evaluation of the sakura round trip.

Per trial:
  1. MANUAL, put the robot at the start pose (nominal or random in the dice
     front region, only collision-free poses), reset AMCL there (stands in
     for the localization that keeps running during manual driving),
  2. optionally set sakura_mission parameters (--set name=value),
  3. switch to AUTO and follow /mission/state until DONE/FAILED/timeout,
  4. compare with the Gazebo ground truth:
     - pose error at the sakura zone before and after the rotation and at
       the return pose (entry into WAIT_SAKURA_1, WAIT_SAKURA_2, DONE),
     - localization error of the control pose the mission used
       (/mission/debug) vs the truth interpolated at the same time, with the
       three largest peaks and where they happened,
     - smallest distance between the outline (max outline + LiDAR) at the
       true pose and any field frame, and where; 0 = wall contact.
Writes one CSV row per trial.
"""
import argparse
import bisect
import csv
import math
import os
import random
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool

from omni_robot_description.geometry import load_geometry
from omni_robot_gazebo.field_layout import load_layout
from omni_robot_gazebo.lidar_mount_study import Study

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
ROTATION_STATES = ("BACKOFF_1", "ROTATE_1", "REAPPROACH_1")
RETURN_STATES = ("BACKOFF_2", "ROTATE_2", "RETURN")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_value(text):
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    return {"true": True, "false": False}.get(text.lower(), text)


class Trial(Node):

    def __init__(self):
        super().__init__("mission_trial", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.set_state = self.create_client(
            SetEntityState, "/gazebo/set_entity_state")
        self.set_autonomy = self.create_client(
            SetBool, "/mode_manager/set_autonomy")
        self.set_params = self.create_client(
            SetParameters, "/sakura_mission/set_parameters")
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.truth = None
        self.state = None
        self.target = None
        self.events = []
        self.samples = []
        self.targets = []
        self.on_truth_sample = None
        self.create_subscription(
            Odometry, "/ground_truth/odom", self.on_truth, 50)
        self.create_subscription(String, "/mission/state", self.on_state, LATCHED)
        self.create_subscription(
            PoseStamped, "/mission/target", self.on_target, LATCHED)
        # control pose actually used by the mission (time-stamped)
        self.debug = []
        self.create_subscription(
            Float64MultiArray, "/mission/debug", self.on_debug, 100)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        p = msg.pose.pose
        self.truth = (p.position.x, p.position.y, yaw_of(p.orientation),
                      math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y))
        if self.on_truth_sample:
            self.on_truth_sample()

    def on_debug(self, msg):
        # t, ref_x, ref_y, ref_v, ref_yaw, est_x, est_y, est_yaw, ...
        d = msg.data
        self.debug.append((d[0], d[5], d[6], d[7], d[3],
                           (self.state or "?").split()[0]))

    def on_state(self, msg):
        self.state = msg.data
        self.events.append((self.now(), msg.data, self.truth, self.target))

    def on_target(self, msg):
        p = msg.pose
        self.target = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.targets.append((self.now(), self.target))

    def call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {client.srv_name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def spin_for(self, sim_seconds, wall_limit=60.0):
        start, wall = self.now(), time.time()
        while self.now() - start < sim_seconds and time.time() - wall < wall_limit:
            rclpy.spin_once(self, timeout_sec=0.02)

    def place(self, x, y, yaw):
        state = EntityState()
        state.name = "omni_robot"
        state.pose.position.x, state.pose.position.y = x, y
        state.pose.position.z = 0.005
        state.pose.orientation.z = math.sin(yaw / 2)
        state.pose.orientation.w = math.cos(yaw / 2)
        state.reference_frame = "world"
        self.call(self.set_state, SetEntityState.Request(state=state))
        self.spin_for(1.0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # stamp 0: AMCL uses the latest odom TF (a current stamp can be ahead
        # of the TF buffer and the pose is then dropped)
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01 ** 2
        msg.pose.covariance[35] = math.radians(1.0) ** 2
        self.initial_pub.publish(msg)
        self.spin_for(2.0)

    def set_mission_params(self, values):
        if not values:
            return
        request = SetParameters.Request()
        for name, value in values.items():
            pv = ParameterValue()
            if isinstance(value, bool):
                pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, value
            elif isinstance(value, (int, float)):
                pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(value)
            else:
                pv.type, pv.string_value = ParameterType.PARAMETER_STRING, value
            request.parameters.append(Parameter(name=name, value=pv))
        result = self.call(self.set_params, request)
        for name, r in zip(values, result.results):
            if not r.successful:
                raise RuntimeError(f"could not set {name}: {r.reason}")


def pose_error(truth, target):
    if truth is None or target is None:
        return None
    return (truth[0] - target[0], truth[1] - target[1], wrap(truth[2] - target[2]))


def truth_at(samples, t):
    """Ground truth (x, y, yaw) linearly interpolated at time t."""
    times = [s[0] for s in samples]
    i = bisect.bisect_left(times, t)
    if i == 0 or i >= len(samples):
        return None
    (t0, x0, y0, a0), (t1, x1, y1, a1) = samples[i - 1], samples[i]
    w = (t - t0) / max(t1 - t0, 1e-9)
    return (x0 + w * (x1 - x0), y0 + w * (y1 - y0), a0 + w * wrap(a1 - a0))


def estimate_errors(node, args):
    """Control pose used by the mission vs ground truth at the same time.

    Returns (t, error_m, state, true_x, true_y, speed) per control step and
    optionally writes a time series CSV.
    """
    errors = []
    rows = []
    previous = None
    for t, ex, ey, eyaw, ref_v, state in node.debug:
        truth = truth_at(node.samples, t)
        if truth is None or math.isnan(ex):
            continue
        speed = 0.0
        if previous is not None and t > previous[0]:
            speed = math.hypot(truth[0] - previous[1], truth[1] - previous[2]) / (t - previous[0])
        previous = (t, truth[0], truth[1])
        error = math.hypot(ex - truth[0], ey - truth[1])
        errors.append((t, error, state, truth[0], truth[1], speed))
        rows.append([round(t, 3), state, round(ex, 4), round(ey, 4), round(eyaw, 4),
                     round(truth[0], 4), round(truth[1], 4), round(truth[2], 4),
                     round(speed, 3), round(ref_v, 3) if not math.isnan(ref_v) else "",
                     round(error * 1000, 1)])
    if args.timeseries:
        with open(args.timeseries, "w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["t", "state", "est_x", "est_y", "est_yaw", "true_x",
                             "true_y", "true_yaw", "true_speed", "planned_v",
                             "est_error_mm"])
            writer.writerows(rows)
    return errors


SETTLE_STATES = {"GO_SAKURA": "go", "ROTATE_1": "rotate1",
                 "REAPPROACH_1": "reapproach1", "ROTATE_2": "rotate2",
                 "RETURN": "return_stop"}


def settle_rows(row, node, args):
    """Overshoot and settling time of each stop/turn from the ground truth.

    Translation: overshoot = how far the robot went past the target along the
    approach direction (taken 0.2 m before the target). Rotation: degrees
    past the target yaw. Settling time = from the first sample inside the
    tolerance to the end of the state (arrival accepted by the mission).
    """
    intervals = []
    for k, (stamp, text, _, _) in enumerate(node.events):
        end = node.events[k + 1][0] if k + 1 < len(node.events) else node.now()
        intervals.append((text.split()[0], stamp, end))
    for state, key in SETTLE_STATES.items():
        span = [(t0, t1) for name, t0, t1 in intervals if name == state]
        for suffix in ("overshoot", "settle_s"):
            row[f"{key}_{suffix}"] = ""
        if not span:
            continue
        t0, t1 = span[0]
        targets = [tg for t, tg in node.targets if t0 - 0.1 <= t < t1]
        samples = [s for s in node.samples if t0 <= s[0] <= t1]
        if not targets or not samples:
            continue
        tx, ty, tyaw = targets[-1]
        rotation = state.startswith("ROTATE")
        if rotation:
            sign = 1.0 if wrap(tyaw - samples[0][3]) >= 0 else -1.0
            errors = [sign * math.degrees(wrap(s[3] - tyaw)) for s in samples]
            overshoot = max(0.0, max(errors))
            inside = [s[0] for s, e in zip(samples, errors)
                      if abs(e) <= args.yaw_tolerance_deg]
            row[f"{key}_overshoot"] = f"{overshoot:.2f}deg"
        else:
            far = [s for s in samples if math.hypot(s[1] - tx, s[2] - ty) >= 0.2]
            ref = far[-1] if far else samples[0]
            ux, uy = tx - ref[1], ty - ref[2]
            norm = math.hypot(ux, uy)
            after = [s for s in samples if s[0] >= ref[0]]
            if norm > 1e-3:
                ux, uy = ux / norm, uy / norm
                overshoot = max(0.0, max((s[1] - tx) * ux + (s[2] - ty) * uy
                                         for s in after))
            else:
                overshoot = 0.0
            tol = args.return_tolerance if state == "RETURN" else args.tolerance
            inside = [s[0] for s in after
                      if abs(s[1] - tx) <= tol and abs(s[2] - ty) <= tol]
            row[f"{key}_overshoot"] = f"{overshoot * 1000:.1f}mm"
        row[f"{key}_settle_s"] = round(t1 - inside[0], 3) if inside else ""


def run_trial(node, study, start, args, overrides):
    node.call(node.set_autonomy, SetBool.Request(data=False))
    node.spin_for(0.5)
    node.place(*start)
    node.set_mission_params(overrides)
    node.events = []
    node.samples = []
    node.targets = []
    node.debug = []
    tracking = {"contact": False, "max_speed": 0.0, "contact_at": "",
                "min_clearance": float("inf"), "min_clearance_at": ""}

    def sample():
        if node.truth is None:
            return
        x, y, yaw, speed = node.truth
        node.samples.append((node.now(), x, y, yaw))
        tracking["max_speed"] = max(tracking["max_speed"], speed)
        state = (node.state or "?").split()[0]
        clearance, frame = study.clearance(x, y, yaw)
        if clearance < tracking["min_clearance"]:
            tracking["min_clearance"] = clearance
            tracking["min_clearance_at"] = (
                f"{state}@({x:.3f},{y:.3f},{math.degrees(yaw):.1f}) {frame}")
        if clearance <= 0.0 and not tracking["contact"]:
            tracking["contact"] = True
            tracking["contact_at"] = f"{state}@({x:.3f},{y:.3f},{math.degrees(yaw):.1f})"

    node.on_truth_sample = sample
    t0 = node.now()
    node.call(node.set_autonomy, SetBool.Request(data=True))
    wall = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.02)
        state = (node.state or "").split()[0] if node.state else ""
        if state in ("DONE", "FAILED") and node.events and node.events[-1][0] > t0:
            break
        if node.now() - t0 > args.timeout or time.time() - wall > args.timeout * 3:
            break
    node.on_truth_sample = None
    t_end = node.now()
    node.call(node.set_autonomy, SetBool.Request(data=False))

    first = {}
    for stamp, text, truth, target in node.events:
        name = text.split()[0]
        first.setdefault(name, (stamp, text, truth, target))

    def t(name):
        return first[name][0] if name in first else None

    def earliest(names):
        times = [t(n) for n in names if t(n) is not None]
        return min(times) if times else None

    final_text = node.events[-1][1] if node.events else ""
    final = final_text.split()[0] if final_text else ""
    row = {
        "label": args.label,
        "start_x": round(start[0], 4), "start_y": round(start[1], 4),
        "start_yaw_deg": round(math.degrees(start[2]), 2),
    }
    row.update({k: v for k, v in overrides.items()})
    go_end, rot_start = t("WAIT_SAKURA_1"), earliest(ROTATION_STATES)
    rot_end, ret_start = t("WAIT_SAKURA_2"), earliest(RETURN_STATES)
    done = t("DONE")
    row["time_go_s"] = round(go_end - t0, 3) if go_end else ""
    row["time_rotation_s"] = round(rot_end - rot_start, 3) if rot_end and rot_start else ""
    row["time_return_s"] = round(done - ret_start, 3) if done and ret_start else ""
    row["time_total_s"] = round(done - t0, 3) if done else ""

    reasons = []
    for key, state in (("sakura", "WAIT_SAKURA_1"), ("sakura_rotated", "WAIT_SAKURA_2"),
                       ("return", "DONE")):
        if state in first:
            _, _, truth, target = first[state]
            err = pose_error(truth, target)
        else:
            err = None
        tol = args.return_tolerance if key == "return" else args.tolerance
        if err is None:
            for suffix in ("dx_mm", "dy_mm", "dyaw_deg"):
                row[f"{key}_{suffix}"] = ""
            continue
        row[f"{key}_dx_mm"] = round(err[0] * 1000, 1)
        row[f"{key}_dy_mm"] = round(err[1] * 1000, 1)
        row[f"{key}_dyaw_deg"] = round(math.degrees(err[2]), 2)
        if abs(err[0]) > tol or abs(err[1]) > tol:
            reasons.append(f"{key} position error")
        if abs(math.degrees(err[2])) > args.yaw_tolerance_deg:
            reasons.append(f"{key} yaw error")
    settle_rows(row, node, args)
    errors = estimate_errors(node, args)
    peaks = []
    for e in sorted(errors, key=lambda e: -e[1]):
        if all(abs(e[0] - p[0]) > 1.0 for p in peaks):
            peaks.append(e)
        if len(peaks) == 3:
            break
    row["max_est_error_mm"] = round(peaks[0][1] * 1000, 1) if peaks else ""
    row["est_error_peaks"] = " | ".join(
        f"{p[1] * 1000:.0f}mm {p[2]}@({p[3]:.2f},{p[4]:.2f}) v={p[5]:.2f}"
        for p in peaks)
    row["max_speed"] = round(tracking["max_speed"], 3)
    row["min_wall_clearance_mm"] = round(tracking["min_clearance"] * 1000, 1)
    row["min_wall_clearance_at"] = tracking["min_clearance_at"]
    row["wall_contact"] = tracking["contact"]
    row["contact_at"] = tracking["contact_at"]
    if tracking["contact"]:
        reasons.insert(0, "wall contact")
    if final == "FAILED":
        reasons.insert(0, "mission failed: " + final_text[len("FAILED"):].strip())
    elif final != "DONE":
        reasons.insert(0, f"timeout (last state {final or 'none'})")
    row["wheel_limited_steps"] = (final_text.split("=")[-1]
                                  if "wheel_speed_limited_steps" in final_text else "")
    row["success"] = not reasons
    row["failure"] = "; ".join(reasons)
    row["sim_duration_s"] = round(t_end - t0, 2)
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--random-start", action="store_true")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--start-region", default=os.path.join(
        get_package_share_directory("omni_robot_gazebo"),
        "config", "start_region.yaml"))
    parser.add_argument("--set", action="append", default=[],
                        help="sakura_mission parameter name=value")
    parser.add_argument("--tolerance", type=float, default=0.05)
    parser.add_argument("--return-tolerance", type=float, default=0.05)
    parser.add_argument("--yaw-tolerance-deg", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--label", default="")
    parser.add_argument("--csv", default="")
    parser.add_argument("--timeseries", default="",
                        help="per control step estimate vs truth (CSV)")
    args = parser.parse_args()

    overrides = {}
    for item in args.set:
        name, value = item.split("=", 1)
        overrides[name] = parse_value(value)
    with open(args.start_region) as stream:
        region = yaml.safe_load(stream)
    study = Study(load_geometry(), load_layout(), standoff=float(region["standoff"]))
    rng = random.Random(args.seed)

    rclpy.init()
    node = Trial()
    node.spin_for(1.0)
    rows = []
    for index in range(args.trials):
        start = (study.center[0], study.center[1], 0.0)
        if args.random_start:
            for _ in range(1000):
                candidate = (
                    study.center[0] + rng.uniform(-region["half_x"], region["half_x"]),
                    study.center[1] + rng.uniform(-region["half_y"], region["half_y"]),
                    math.radians(rng.uniform(-region["half_yaw_deg"],
                                             region["half_yaw_deg"])))
                if study.start_ok(*candidate):
                    start = candidate
                    break
        row = run_trial(node, study, start, args, overrides)
        row["trial"] = index + 1
        rows.append(row)
        print(" ".join(f"{k}={v}" for k, v in row.items()), flush=True)
        if args.csv:
            exists = os.path.exists(args.csv) and os.path.getsize(args.csv) > 0
            with open(args.csv, "a", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(row.keys()))
                if not exists:
                    writer.writeheader()
                writer.writerow(row)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
