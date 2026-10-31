"""Path smoothing, speed planning and the tracking laws (no ROS dependency).

SmoothPath
  1. shortcut: drop waypoints while the straight line stays clear of walls,
  2. fillet: replace each corner with the largest clear circular arc,
  3. sample every `step` m with position, unit tangent and curvature,
  4. speed plan over the whole path (forward/backward passes):
       v <= max_speed
       v <= direction limit of the X-configured wheels at the fixed heading
       v <= sqrt(max_lateral_accel / curvature)                  (arcs)
       tangential acceleration <= sqrt(a^2 - (v^2 * curvature)^2)
       with FrictionLimit enabled, both are also limited by the drive-wheel
       friction in the body-frame direction of travel (see FrictionLimit)
       v = 0 at the end.
The follower projects the robot onto the path and commands the planned speed
there (feed-forward along the tangent) plus a small, capped lateral
correction, so no lag can build up and the robot is never pushed above the
plan to catch up.
"""
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class FrictionLimit:
    """Acceleration limits from the drive-wheel friction (X-configured omni
    wheels, equal wheel loads, no load transfer).

    The wheels roll along the body diagonals e1, e2 (45 deg to the chassis
    faces); opposite wheels share an axis, so the force along each diagonal is
    F_i - F_j with |F| <= mu*m*g/4. A translational acceleration a is feasible
    when |a.e1| <= B and |a.e2| <= B, with
        B = safety * mu * g / 2 * (1 - yaw_reserve)
    (yaw_reserve = share of the friction kept for turning). Along a chassis
    face this gives 0.71*mu*g, along a diagonal 0.5*mu*g (times the shares).
    `yaw_accel` caps the angular acceleration the same way:
        yaw_reserve * safety * mu * m * g * R / I.
    Disabled: no limit (inf)."""

    G = 9.81

    def __init__(self, enabled=False, mu=0.6, safety=0.8, yaw_reserve=0.15,
                 mass=20.0, yaw_inertia=0.83, wheel_radius_arm=0.2245):
        self.enabled = bool(enabled)
        if self.enabled:
            self.budget = safety * mu * self.G / 2.0 * (1.0 - yaw_reserve)
            self.yaw_accel = (yaw_reserve * safety * mu * mass * self.G
                              * wheel_radius_arm / yaw_inertia)
        else:
            self.budget = float("inf")
            self.yaw_accel = float("inf")

    @staticmethod
    def shares(direction):
        """|cos|, |sin| of the direction measured from a wheel diagonal."""
        return (abs(math.cos(direction - math.pi / 4.0)),
                abs(math.sin(direction - math.pi / 4.0)))

    def max_accel(self, direction):
        """Largest acceleration along a body-frame direction."""
        if not self.enabled:
            return float("inf")
        return self.budget / max(max(self.shares(direction)), 1e-9)

    def min_accel(self):
        """Direction-independent (diagonal) limit."""
        return self.budget

    def tangential(self, direction, normal_accel):
        """Largest tangential acceleration while `normal_accel` acts across
        the path (conservative: signs ignored). Direction: body-frame heading
        of the path tangent."""
        if not self.enabled:
            return float("inf")
        t1, t2 = self.shares(direction)
        n1, n2 = t2, t1  # the normal is the tangent turned by 90 deg
        best = float("inf")
        for t, n in ((t1, n1), (t2, n2)):
            if t > 1e-9:
                best = min(best, (self.budget - normal_accel * n) / t)
        return best


def stopping_speed(distance, decel, gain):
    """Speed that stops exactly at `distance` with deceleration `decel`;
    linear (gain * distance) very close to the goal to avoid chattering."""
    distance = max(0.0, distance)
    return min(math.sqrt(2.0 * decel * distance), gain * distance)


class SmoothPath:
    """See the module docstring. Heading: fixed at yaw_start, or, when
    yaw_end differs, turned along the path inside one section where
    rotation_clear(x, y) says the whole turn is free of obstacles (smoothstep
    profile over at most rotation_length m)."""

    def __init__(self, points, yaw_start, is_clear_pose, max_speed, max_accel,
                 max_lateral_accel, speed_limit, start_speed=0.0,
                 max_radius=0.8, step=0.01, shortcut=True, yaw_end=None,
                 rotation_clear=None, rotation_length=1.2,
                 rotation_min_length=0.4, max_yaw_rate=3.0, max_yaw_accel=6.0,
                 friction=None):
        self.friction = friction or FrictionLimit()
        self.yaw = yaw_start if yaw_end is None else yaw_end
        self.yaw_start = yaw_start
        self.max_speed = max_speed
        self.max_accel = max_accel
        turning = yaw_end is not None and abs(wrap(yaw_end - yaw_start)) > 1e-3
        headings = (yaw_start, yaw_end) if turning else (yaw_start,)
        pose_clear = is_clear_pose

        def is_clear_pose(x, y):
            # geometry must be clear at the start and the end heading
            return all(pose_clear(x, y, a) for a in headings)
        points = [tuple(p) for p in points]
        # remove duplicates
        cleaned = [points[0]]
        for p in points[1:]:
            if math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) > 1e-4:
                cleaned.append(p)
        points = cleaned
        if shortcut and len(points) > 2:
            points = self._shortcut(points, is_clear_pose, step)
        self.corners = points
        self.radii = []
        self.samples = self._fillet_and_sample(points, is_clear_pose, max_radius, step)
        self.length = self.samples[-1][5]
        self.rotation_section = None
        self._yaw_profile(turning, yaw_end, rotation_clear, rotation_length,
                          rotation_min_length)
        self._plan_speed(max_lateral_accel, speed_limit, start_speed,
                         max_yaw_rate, max_yaw_accel)
        self.index = 0

    # ---------------------------------------------------------------- geometry
    def _segment_clear(self, a, b, is_clear_pose, step):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        count = max(1, int(math.ceil(length / (2 * step))))
        return all(is_clear_pose(a[0] + (b[0] - a[0]) * k / count,
                                 a[1] + (b[1] - a[1]) * k / count)
                   for k in range(count + 1))

    def _shortcut(self, points, is_clear_pose, step, spacing=0.05):
        """Greedy string pulling over points every `spacing` m on the polyline,
        so a shortcut may start or end part-way along a segment."""
        dense = [points[0]]
        for a, b in zip(points, points[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            count = max(1, int(math.ceil(length / spacing)))
            dense += [(a[0] + (b[0] - a[0]) * k / count,
                       a[1] + (b[1] - a[1]) * k / count) for k in range(1, count + 1)]
        points = dense
        result = [points[0]]
        i = 0
        while i < len(points) - 1:
            j = len(points) - 1
            while j > i + 1 and not self._segment_clear(
                    points[i], points[j], is_clear_pose, step):
                j -= 1
            result.append(points[j])
            i = j
        # drop points that lie on a straight line
        cleaned = [result[0]]
        for k in range(1, len(result) - 1):
            ax, ay = cleaned[-1]
            bx, by = result[k]
            cx, cy = result[k + 1]
            if abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)) > 1e-6:
                cleaned.append(result[k])
        cleaned.append(result[-1])
        return cleaned

    @staticmethod
    def _arc(corner, before, after, radius):
        """Fillet of radius at corner: (tangent point in, out, center, angle)."""
        bx, by = corner
        ux, uy = before[0] - bx, before[1] - by
        vx, vy = after[0] - bx, after[1] - by
        lu, lv = math.hypot(ux, uy), math.hypot(vx, vy)
        ux, uy, vx, vy = ux / lu, uy / lu, vx / lv, vy / lv
        interior = math.acos(max(-1.0, min(1.0, ux * vx + uy * vy)))
        turn = math.pi - interior
        tangent = radius * math.tan(0.5 * turn)
        p_in = (bx + ux * tangent, by + uy * tangent)
        p_out = (bx + vx * tangent, by + vy * tangent)
        # center along the bisector
        hx, hy = ux + vx, uy + vy
        lh = math.hypot(hx, hy)
        dist = radius / math.cos(0.5 * turn)
        center = (bx + hx / lh * dist, by + hy / lh * dist)
        return p_in, p_out, center, turn, tangent

    def _arc_points(self, p_in, p_out, center, step):
        a0 = math.atan2(p_in[1] - center[1], p_in[0] - center[0])
        a1 = math.atan2(p_out[1] - center[1], p_out[0] - center[0])
        delta = wrap(a1 - a0)
        radius = math.hypot(p_in[0] - center[0], p_in[1] - center[1])
        count = max(2, int(math.ceil(abs(delta) * radius / step)))
        return [(center[0] + radius * math.cos(a0 + delta * k / count),
                 center[1] + radius * math.sin(a0 + delta * k / count))
                for k in range(count + 1)], radius, delta

    def _fillet_and_sample(self, points, is_clear_pose, max_radius, step):
        # choose radii corner by corner (tangent length <= half of each side,
        # whole side for the first and last segment)
        fillets = [None] * len(points)
        for i in range(1, len(points) - 1):
            prev_len = math.hypot(points[i][0] - points[i - 1][0],
                                  points[i][1] - points[i - 1][1])
            next_len = math.hypot(points[i + 1][0] - points[i][0],
                                  points[i + 1][1] - points[i][1])
            limit_prev = prev_len * (1.0 if i == 1 else 0.5)
            limit_next = next_len * (1.0 if i == len(points) - 2 else 0.5)
            radius = max_radius
            chosen = None
            while radius >= 0.02:
                p_in, p_out, center, turn, tangent = self._arc(
                    points[i], points[i - 1], points[i + 1], radius)
                if turn < 1e-3:
                    break
                if tangent <= min(limit_prev, limit_next):
                    arc, _, _ = self._arc_points(p_in, p_out, center, 2 * step)
                    if all(is_clear_pose(x, y) for x, y in arc):
                        chosen = (p_in, p_out, center)
                        break
                radius *= 0.85
            fillets[i] = chosen
            self.radii.append(round(radius, 3) if chosen else 0.0)

        samples = []  # x, y, tx, ty, curvature, s

        def add_line(a, b):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            if length < 1e-6:
                return
            tx, ty = (b[0] - a[0]) / length, (b[1] - a[1]) / length
            count = max(1, int(math.ceil(length / step)))
            for k in range(0 if not samples else 1, count + 1):
                append(a[0] + (b[0] - a[0]) * k / count,
                       a[1] + (b[1] - a[1]) * k / count, tx, ty, 0.0)

        def append(x, y, tx, ty, curvature):
            if samples:
                s = samples[-1][5] + math.hypot(x - samples[-1][0], y - samples[-1][1])
            else:
                s = 0.0
            samples.append((x, y, tx, ty, curvature, s))

        current = points[0]
        for i in range(1, len(points)):
            fillet = fillets[i] if i < len(points) - 1 else None
            if fillet is None:
                add_line(current, points[i])
                current = points[i]
                continue
            p_in, p_out, center = fillet
            add_line(current, p_in)
            arc, radius, delta = self._arc_points(p_in, p_out, center, step)
            sign = 1.0 if delta > 0 else -1.0
            for x, y in arc[1:]:
                rx, ry = x - center[0], y - center[1]
                tx, ty = -sign * ry / radius, sign * rx / radius
                append(x, y, tx, ty, 1.0 / radius)
            current = p_out
        if len(samples) == 1:
            samples.append(samples[0][:5] + (0.0,))
        return samples

    # ---------------------------------------------------------------- heading
    def _yaw_profile(self, turning, yaw_end, rotation_clear, length_max,
                     length_min):
        n = len(self.samples)
        self.yaws = [self.yaw_start] * n
        self.yaw_rate_per_m = [0.0] * n     # dyaw/ds
        self.yaw_rate_slope = [0.0] * n     # d2yaw/ds2
        if not turning:
            return
        if rotation_clear is None:
            raise ValueError("turning path needs rotation_clear")
        flags = [rotation_clear(s[0], s[1]) for s in self.samples]
        start = None
        section = None
        for i, ok in enumerate(flags + [False]):
            if ok and start is None:
                start = i
            if not ok and start is not None:
                run = self.samples[i - 1][5] - self.samples[start][5]
                if run >= length_min:
                    section = (self.samples[start][5],
                               self.samples[start][5] + min(run, length_max))
                    break
                start = None
        if section is None:
            raise ValueError("no clear section to turn while moving")
        self.rotation_section = section
        delta = wrap(yaw_end - self.yaw_start)
        s0, s1 = section
        length = s1 - s0
        for i, sample in enumerate(self.samples):
            u = min(1.0, max(0.0, (sample[5] - s0) / length))
            self.yaws[i] = self.yaw_start + delta * (3 * u * u - 2 * u ** 3)
            if 0.0 < u < 1.0:
                self.yaw_rate_per_m[i] = delta * 6 * u * (1 - u) / length
                self.yaw_rate_slope[i] = delta * 6 * (1 - 2 * u) / length ** 2

    # ---------------------------------------------------------------- speed
    def _plan_speed(self, max_lateral_accel, speed_limit, start_speed,
                    max_yaw_rate, max_yaw_accel):
        n = len(self.samples)
        limit = []
        for i, (x, y, tx, ty, curvature, s) in enumerate(self.samples):
            v = self.max_speed
            # body-frame direction at the planned heading, plus the wheel
            # speed used by turning (yaw rate = v * dyaw/ds)
            direction = math.atan2(ty, tx) - self.yaws[i]
            g = self.yaw_rate_per_m[i]
            v = min(v, speed_limit(direction, g))
            if abs(g) > 1e-6:
                v = min(v, max_yaw_rate / abs(g))
            if abs(self.yaw_rate_slope[i]) > 1e-6:
                v = min(v, math.sqrt(max_yaw_accel / abs(self.yaw_rate_slope[i])))
            if curvature > 1e-6:
                v = min(v, math.sqrt(min(max_lateral_accel,
                                         self.friction.max_accel(direction))
                                     / curvature))
            limit.append(v)

        def tangential(v, curvature, i):
            lateral = v * v * curvature
            a = math.sqrt(max(self.max_accel ** 2 - lateral ** 2,
                              (0.2 * self.max_accel) ** 2))
            if self.friction.enabled:
                direction = math.atan2(self.samples[i][3], self.samples[i][2]) \
                    - self.yaws[i]
                a = min(a, max(self.friction.tangential(direction, lateral),
                               0.2 * self.friction.min_accel()))
            return a

        speed = list(limit)
        speed[0] = min(speed[0], start_speed)
        for i in range(1, n):
            ds = self.samples[i][5] - self.samples[i - 1][5]
            a = tangential(speed[i - 1], self.samples[i - 1][4], i - 1)
            speed[i] = min(speed[i], math.sqrt(speed[i - 1] ** 2 + 2 * a * ds))
        speed[-1] = 0.0
        for i in range(n - 2, -1, -1):
            ds = self.samples[i + 1][5] - self.samples[i][5]
            a = tangential(speed[i + 1], self.samples[i + 1][4], i + 1)
            speed[i] = min(speed[i], math.sqrt(speed[i + 1] ** 2 + 2 * a * ds))
        self.speed = speed
        self.speed_limit = limit

    def planned_time(self):
        total = 0.0
        for i in range(1, len(self.samples)):
            ds = self.samples[i][5] - self.samples[i - 1][5]
            v = 0.5 * (self.speed[i] + self.speed[i - 1])
            total += ds / max(v, 1e-3)
        return total

    # ---------------------------------------------------------------- tracking
    def project(self, x, y, window=200):
        """Index of the closest sample, searched forward from the last one."""
        best, best_d = self.index, float("inf")
        start = max(0, self.index - 20)
        for i in range(start, min(len(self.samples), self.index + window)):
            sx, sy = self.samples[i][0], self.samples[i][1]
            d = (sx - x) ** 2 + (sy - y) ** 2
            if d < best_d:
                best, best_d = i, d
        self.index = best
        return best

    def sample_at(self, s):
        lo, hi = self.index, len(self.samples) - 1
        while lo < hi and self.samples[lo][5] < s:
            lo += 1
        return lo

    @property
    def goal(self):
        return self.samples[-1][0], self.samples[-1][1]


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
