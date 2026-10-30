"""Geometric study of a LiDAR mounting (no simulation needed).

For one robot_geometry.yaml it reports
  * the self-occlusion mask (chassis/wheel shapes at the scan height),
  * the collision outline including the LiDAR protrusion,
  * the start region in front of the dice zone: from the nominal center,
    which offsets in x, y and yaw are allowed when
      - the outline stays inside area 1 and touches no frame or dice marker,
      - no wall inside the unmasked field of view is closer than range_min
        (it would be invisible to the LiDAR),
  * whether the provisional route points collide with the outline.
"""
import argparse
import math

import yaml

from omni_robot_description.geometry import (
    lidar_mount_pose, lidar_self_occlusion, load_geometry, outline_extents,
    outline_polygon, ray_polygon_overlap)
from omni_robot_gazebo.field_layout import (
    _to_world, frame_rects, load_layout, zone_world)


def rect_polygon(rect):
    _, x0, x1, y0, y1 = rect
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def transform(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in points]


def polygons_overlap(a, b):
    """Separating axis test for convex polygons (touching counts as overlap)."""
    for polygon in (a, b):
        for i in range(len(polygon)):
            ax, ay = polygon[i]
            bx, by = polygon[(i + 1) % len(polygon)]
            nx, ny = -(by - ay), bx - ax
            pa = [nx * px + ny * py for px, py in a]
            pb = [nx * px + ny * py for px, py in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


class Study:
    def __init__(self, geometry, layout, standoff):
        self.geometry = geometry
        self.outline = outline_polygon(geometry)
        self.extents = outline_extents(geometry)
        self.frames = [rect_polygon(r) for r in frame_rects(layout)]
        dice = _to_world(layout, layout["dice_marker"]["outer"])
        self.dice = rect_polygon(("dice",) + dice)
        self.area1 = zone_world(layout, "area1")
        lidar = geometry["lidar"]
        self.range_min = float(lidar["range_min"])
        self.mask = lidar_self_occlusion(geometry)
        self.half_fov = math.radians(float(lidar["fov_deg"])) / 2
        self.mount = lidar_mount_pose(geometry)
        self.ray_angles = [
            -self.half_fov + k * math.radians(0.25)
            for k in range(int(round(2 * self.half_fov / math.radians(0.25))) + 1)
        ]
        self.ray_angles = [a for a in self.ray_angles
                           if not any(lo < a < hi for lo, hi in self.mask)]
        # Nominal start: right edge `standoff` left of the dice zone line,
        # centered in area 1 between its bottom frame and top line.
        dice_line_outer = _to_world(layout, layout["lines"]["dice_zone_left"])[0]
        self.center = (
            dice_line_outer - self.extents["+x"] - standoff,
            0.5 * ((self.area1[2] + self.extents["-y"])
                   + (self.area1[3] - self.extents["+y"])),
        )

    def collides(self, x, y, yaw):
        body = transform(self.outline, x, y, yaw)
        return any(polygons_overlap(body, frame) for frame in self.frames)

    def wall_too_close(self, x, y, yaw):
        """A wall in the unmasked view nearer than range_min."""
        mx, my, _, myaw, upside_down = self.mount
        c, s = math.cos(yaw), math.sin(yaw)
        sx, sy = x + c * mx - s * my, y + s * mx + c * my
        near = []
        for frame in self.frames:
            px = min(max(sx, frame[0][0]), frame[1][0])
            py = min(max(sy, frame[0][1]), frame[2][1])
            if math.hypot(px - sx, py - sy) < self.range_min:
                near.append(frame)
        if not near:
            return False
        direction = -1.0 if upside_down else 1.0
        for angle in self.ray_angles:
            heading = yaw + myaw + direction * angle
            dx, dy = math.cos(heading), math.sin(heading)
            for frame in near:
                if ray_polygon_overlap(sx, sy, dx, dy, frame, self.range_min) > 0:
                    return True
        return False

    def start_ok(self, x, y, yaw):
        body = transform(self.outline, x, y, yaw)
        x0, x1, y0, y1 = self.area1
        if any(not (x0 <= px <= x1 and y0 <= py <= y1) for px, py in body):
            return False
        if self.collides(x, y, yaw) or polygons_overlap(body, self.dice):
            return False
        return not self.wall_too_close(x, y, yaw)

    def start_region(self, half_x, half_y, half_yaw_deg):
        cx, cy = self.center
        per_yaw = []
        total = good = 0
        for yaw_deg in range(-int(half_yaw_deg), int(half_yaw_deg) + 1, 5):
            yaw = math.radians(yaw_deg)
            xs = [k * 0.005 for k in range(-int(half_x / 0.005), int(half_x / 0.005) + 1)]
            ys = [k * 0.002 for k in range(-int(half_y / 0.002), int(half_y / 0.002) + 1)]
            ok_x = [dx for dx in xs if self.start_ok(cx + dx, cy, yaw)]
            ok_y = [dy for dy in ys if self.start_ok(cx, cy + dy, yaw)]
            per_yaw.append((yaw_deg,
                            (min(ok_x), max(ok_x)) if ok_x else None,
                            (min(ok_y), max(ok_y)) if ok_y else None))
            for dx in xs[::5]:
                for dy in ys[::5]:
                    total += 1
                    good += self.start_ok(cx + dx, cy + dy, yaw)
        return per_yaw, good / total


def fmt_range(r):
    return "none" if r is None else f"{r[0]:+.3f} .. {r[1]:+.3f}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry", default=None)
    parser.add_argument("--layout", default=None)
    parser.add_argument("--route", default=None,
                        help="route_check_points.yaml to check for collisions")
    parser.add_argument("--standoff", type=float, default=0.05,
                        help="仮値: gap between robot right edge and dice line")
    parser.add_argument("--half-x", type=float, default=0.30)
    parser.add_argument("--half-y", type=float, default=0.15)
    parser.add_argument("--half-yaw-deg", type=float, default=10.0)
    args = parser.parse_args()

    geometry = load_geometry(args.geometry)
    study = Study(geometry, load_layout(args.layout), args.standoff)
    face = geometry["lidar"]["mount"]["face"]
    print(f"face {face}")
    print("mask_deg " + ", ".join(
        f"[{math.degrees(lo):.2f}, {math.degrees(hi):.2f}]" for lo, hi in study.mask))
    print("outline_extents " + ", ".join(
        f"{k}={v:.3f}" for k, v in study.extents.items()))
    print(f"start_center x={study.center[0]:.3f} y={study.center[1]:.3f}")
    per_yaw, fraction = study.start_region(
        args.half_x, args.half_y, args.half_yaw_deg)
    for yaw_deg, rx, ry in per_yaw:
        print(f"start yaw {yaw_deg:+3d}: dx {fmt_range(rx)}  dy {fmt_range(ry)}")
    print(f"start_feasible_fraction {fraction:.3f}")
    if args.route:
        with open(args.route) as stream:
            points = yaml.safe_load(stream)["points"]
        for point in points:
            x, y, yaw_deg = point["pose"]
            yaw = math.radians(yaw_deg)
            flags = []
            if study.collides(x, y, yaw):
                flags.append("COLLIDES")
            if study.wall_too_close(x, y, yaw):
                flags.append("WALL<range_min")
            print(f"route {point['name']}: {' '.join(flags) or 'ok'}")


if __name__ == "__main__":
    main()
