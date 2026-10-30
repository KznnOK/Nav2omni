"""Reference motions for the holonomic follower (no ROS dependency).

PathMotion moves a reference point along a polyline with a trapezoidal
speed profile: it accelerates at max_accel, never exceeds max_speed, and
slows down in time for each corner and for the end point. The corner speed
is chosen so that the overshoot past a corner stays near corner_deviation:
    v_c = sqrt(2 * d * a / (2 sin(turn / 2)))
YawMotion does the same for an in-place rotation.
"""
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PathMotion:

    def __init__(self, points, max_speed, max_accel, corner_deviation):
        self.points = [tuple(p) for p in points]
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.cumulative = [0.0]
        for (ax, ay), (bx, by) in zip(self.points, self.points[1:]):
            self.cumulative.append(self.cumulative[-1] + math.hypot(bx - ax, by - ay))
        self.length = self.cumulative[-1]
        # speed limits at points along the path: (s, v)
        self.limits = []
        for i in range(1, len(self.points) - 1):
            turn = self._turn_angle(i)
            change = 2.0 * math.sin(0.5 * turn)
            if change > 1e-3:
                corner = math.sqrt(2.0 * corner_deviation * max_accel / change)
                self.limits.append((self.cumulative[i], min(max_speed, corner)))
        self.limits.append((self.length, 0.0))
        self.s = 0.0
        self.v = 0.0

    def _turn_angle(self, i):
        (ax, ay), (bx, by), (cx, cy) = self.points[i - 1:i + 2]
        first = math.atan2(by - ay, bx - ax)
        second = math.atan2(cy - by, cx - bx)
        return abs(wrap(second - first))

    def allowed_speed(self, s):
        speed = self.max_speed
        for limit_s, limit_v in self.limits:
            if limit_s >= s:
                speed = min(speed, math.sqrt(
                    limit_v ** 2 + 2.0 * self.max_accel * (limit_s - s)))
        return speed

    def step(self, dt):
        if self.s >= self.length:
            self.v = 0.0
            return
        self.v = min(self.v + self.max_accel * dt,
                     self.allowed_speed(self.s + self.v * dt))
        self.s = min(self.length, self.s + self.v * dt)
        if self.s >= self.length:
            self.v = 0.0

    @property
    def finished(self):
        return self.s >= self.length

    def reference(self):
        """Reference point, unit tangent and speed."""
        if self.length <= 0.0:
            return self.points[-1], (0.0, 0.0), 0.0
        i = 0
        while i < len(self.cumulative) - 2 and self.cumulative[i + 1] < self.s:
            i += 1
        (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
        seg = max(1e-9, self.cumulative[i + 1] - self.cumulative[i])
        t = min(1.0, max(0.0, (self.s - self.cumulative[i]) / seg))
        tangent = ((bx - ax) / seg, (by - ay) / seg)
        return (ax + t * (bx - ax), ay + t * (by - ay)), tangent, self.v

    @property
    def goal(self):
        return self.points[-1]


class YawMotion:

    def __init__(self, yaw_from, yaw_to, max_rate, max_accel):
        self.start = yaw_from
        self.delta = wrap(yaw_to - yaw_from)
        self.goal_yaw = yaw_from + self.delta
        self.max_rate = max_rate
        self.max_accel = max_accel
        self.progress = 0.0
        self.rate = 0.0

    def step(self, dt):
        remaining = abs(self.delta) - self.progress
        if remaining <= 0.0:
            self.rate = 0.0
            return
        allowed = math.sqrt(2.0 * self.max_accel * max(0.0, remaining - self.rate * dt))
        self.rate = min(self.rate + self.max_accel * dt, self.max_rate, allowed)
        self.progress = min(abs(self.delta), self.progress + self.rate * dt)

    @property
    def finished(self):
        return self.progress >= abs(self.delta)

    def reference(self):
        sign = 1.0 if self.delta >= 0 else -1.0
        return self.start + sign * self.progress, sign * self.rate


class CommandShaper:
    """Limits the commanded velocity change (map frame) and the speed."""

    def __init__(self, max_speed, max_accel, max_yaw_rate, max_yaw_accel):
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.max_yaw_rate = max_yaw_rate
        self.max_yaw_accel = max_yaw_accel
        self.vx = self.vy = self.wz = 0.0

    def reset(self, vx=0.0, vy=0.0, wz=0.0):
        self.vx, self.vy, self.wz = vx, vy, wz

    def shape(self, vx, vy, wz, dt):
        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            vx, vy = vx * self.max_speed / speed, vy * self.max_speed / speed
        dvx, dvy = vx - self.vx, vy - self.vy
        change = math.hypot(dvx, dvy)
        limit = self.max_accel * dt
        if change > limit:
            dvx, dvy = dvx * limit / change, dvy * limit / change
        self.vx += dvx
        self.vy += dvy
        wz = max(-self.max_yaw_rate, min(self.max_yaw_rate, wz))
        dwz = max(-self.max_yaw_accel * dt, min(self.max_yaw_accel * dt, wz - self.wz))
        self.wz += dwz
        return self.vx, self.vy, self.wz
