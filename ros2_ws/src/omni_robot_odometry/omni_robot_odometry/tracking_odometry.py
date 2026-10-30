"""Planar odometry from two passive tracking wheels and the IMU yaw rate.

Wheel i measures the floor displacement along its rolling direction a_i at
the contact point (x_i, y_i) in base_link:
    d_i = cos(a_i) * (dx - y_i * dth) + sin(a_i) * (dy + x_i * dth)
dth comes from the IMU, so the two equations give dx, dy. For the default
layout (a_1 = 90 deg, a_2 = 0 deg) this is
    dy = d1 - x1 * dth,   dx = d2 + y2 * dth.
Shared by the simulation and the real robot. Publishes /tracking/odom
(no TF; robot_localization owns odom -> base_footprint).
"""
import bisect
import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

from omni_robot_description.geometry import (
    default_geometry_file, load_geometry, tracking_wheel_layout)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class TrackingOdometry(Node):

    def __init__(self):
        super().__init__("tracking_odometry")

        # Defaults come from robot_geometry.yaml
        names, radius, xs, ys, angles = tracking_wheel_layout(load_geometry(
            self.declare_parameter(
                "geometry_file", default_geometry_file()).value))
        self.declare_parameter("wheel_joint_names", names)
        self.declare_parameter("wheel_radius", radius)
        self.declare_parameter("wheel_x", xs)
        self.declare_parameter("wheel_y", ys)
        self.declare_parameter("wheel_rolling_angle", angles)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_topic", "/tracking/odom")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("twist_covariance_xy", 1.0e-4)
        self.declare_parameter("twist_covariance_yaw", 1.0e-4)

        self.joint_names = list(self.get_parameter("wheel_joint_names").value)
        self.radius = float(self.get_parameter("wheel_radius").value)
        wheel_x = list(self.get_parameter("wheel_x").value)
        wheel_y = list(self.get_parameter("wheel_y").value)
        angles = list(self.get_parameter("wheel_rolling_angle").value)
        if not (len(self.joint_names) == len(wheel_x) == len(wheel_y)
                == len(angles) == 2):
            raise ValueError("exactly two tracking wheels are supported")

        # d_i + dth * k_i = [cos a_i, sin a_i] . [dx, dy]
        self.k = [math.cos(a) * y - math.sin(a) * x
                  for x, y, a in zip(wheel_x, wheel_y, angles)]
        c1, s1 = math.cos(angles[0]), math.sin(angles[0])
        c2, s2 = math.cos(angles[1]), math.sin(angles[1])
        det = c1 * s2 - s1 * c2
        if abs(det) < 1e-6:
            raise ValueError("tracking wheels must not be parallel")
        self.inverse = (s2 / det, -s1 / det, -c2 / det, c1 / det)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.cov_xy = float(self.get_parameter("twist_covariance_xy").value)
        self.cov_yaw = float(self.get_parameter("twist_covariance_yaw").value)

        # Integrated IMU yaw as (time, angle) for interpolation.
        self.imu_times = deque(maxlen=500)
        self.imu_yaw = deque(maxlen=500)
        self.last_imu_rate = 0.0

        self.last_positions = None
        self.last_time = None
        self.last_yaw_integral = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10)
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value), self.imu_callback, 50)
        self.create_subscription(
            JointState, "/joint_states", self.joint_callback, 50)

        self.get_logger().info(
            f"Tracking odometry: {self.joint_names} r={self.radius:.4f} "
            f"x={wheel_x} y={wheel_y} "
            f"angle_deg={[round(math.degrees(a), 2) for a in angles]}")

    def imu_callback(self, msg):
        t = stamp_seconds(msg.header.stamp)
        rate = msg.angular_velocity.z
        if self.imu_times:
            dt = t - self.imu_times[-1]
            if dt <= 0.0:
                return
            # trapezoidal integration of the yaw rate
            angle = self.imu_yaw[-1] + 0.5 * (rate + self.last_imu_rate) * dt
        else:
            angle = 0.0
        self.imu_times.append(t)
        self.imu_yaw.append(angle)
        self.last_imu_rate = rate

    def yaw_integral_at(self, t):
        times = self.imu_times
        if not times:
            return None
        if t >= times[-1]:
            return self.imu_yaw[-1] + self.last_imu_rate * (t - times[-1])
        if t <= times[0]:
            return self.imu_yaw[0]
        i = bisect.bisect_right(times, t)
        t0, t1 = times[i - 1], times[i]
        a0, a1 = self.imu_yaw[i - 1], self.imu_yaw[i]
        return a0 + (a1 - a0) * (t - t0) / (t1 - t0)

    def joint_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.joint_names):
            return
        t = stamp_seconds(msg.header.stamp)
        yaw_integral = self.yaw_integral_at(t)
        if yaw_integral is None:
            return
        current = [positions[name] for name in self.joint_names]
        if self.last_positions is None:
            self.last_positions = current
            self.last_time = t
            self.last_yaw_integral = yaw_integral
            return
        dt = t - self.last_time
        if dt <= 0.0:
            return

        d = [self.radius * (now - before)
             for now, before in zip(current, self.last_positions)]
        dth = yaw_integral - self.last_yaw_integral
        b1 = d[0] + dth * self.k[0]
        b2 = d[1] + dth * self.k[1]
        i11, i12, i21, i22 = self.inverse
        dx = i11 * b1 + i12 * b2
        dy = i21 * b1 + i22 * b2

        # integrate in the odom frame with the mid-step heading
        heading = self.yaw + 0.5 * dth
        self.x += math.cos(heading) * dx - math.sin(heading) * dy
        self.y += math.sin(heading) * dx + math.cos(heading) * dy
        self.yaw = math.atan2(math.sin(self.yaw + dth), math.cos(self.yaw + dth))

        self.last_positions = current
        self.last_time = t
        self.last_yaw_integral = yaw_integral

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        odom.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        odom.twist.twist.linear.x = dx / dt
        odom.twist.twist.linear.y = dy / dt
        odom.twist.twist.angular.z = dth / dt
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = self.cov_xy
        odom.twist.covariance[7] = self.cov_xy
        odom.twist.covariance[35] = self.cov_yaw
        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingOdometry()
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
