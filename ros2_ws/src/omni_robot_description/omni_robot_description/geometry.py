"""Load robot_geometry.yaml and derive the values used by every node.

Shared by simulation and the real robot: the URDF, the drive controller and
all odometry nodes take their dimensions from here so they cannot drift apart.
"""
import math
import os

import yaml


def default_geometry_file():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory("omni_robot_description"),
        "config", "robot_geometry.yaml")


def load_geometry(path=None):
    with open(path or default_geometry_file()) as stream:
        return yaml.safe_load(stream)


def drive_wheel_layout(geometry):
    """Return wheel names, x, y and rolling-direction yaw (rad).

    Each wheel sits at center_distance along its position angle. Its axle is
    radial, so the rolling direction is the position angle minus 90 deg.
    """
    wheels = geometry["drive_wheels"]
    distance = float(wheels["center_distance"])
    names, xs, ys, yaws = [], [], [], []
    for name, angle_deg in zip(wheels["names"], wheels["position_angles_deg"]):
        angle = math.radians(angle_deg)
        names.append(f"{name}_wheel_joint")
        xs.append(distance * math.cos(angle))
        ys.append(distance * math.sin(angle))
        yaws.append(math.atan2(math.sin(angle - math.pi / 2),
                               math.cos(angle - math.pi / 2)))
    return names, xs, ys, yaws


def drive_wheel_radius(geometry):
    return 0.5 * float(geometry["drive_wheels"]["diameter"])


def tracking_wheel_layout(geometry):
    """Return joint names, radius, x, y and measuring direction (rad)."""
    tracking = geometry["tracking_wheels"]
    names, xs, ys, angles = [], [], [], []
    for wheel in tracking["wheels"]:
        names.append(f"{wheel['name']}_wheel_joint")
        xs.append(float(wheel["xy"][0]))
        ys.append(float(wheel["xy"][1]))
        angles.append(math.radians(float(wheel["rolling_deg"])))
    return names, 0.5 * float(tracking["diameter"]), xs, ys, angles


FACE_YAW = {"+x": 0.0, "+y": math.pi / 2, "-x": math.pi, "-y": -math.pi / 2}


def lidar_mount_pose(geometry):
    """Nominal scan origin in base_footprint: (x, y, z, yaw, upside_down)."""
    chassis = geometry["chassis"]
    mount = geometry["lidar"]["mount"]
    yaw = FACE_YAW[mount["face"]]
    half = 0.5 * float(chassis["length"] if mount["face"] in ("+x", "-x")
                       else chassis["width"])
    out = half + float(mount["outward_offset"])
    side = float(mount["lateral_offset"])
    return (out * math.cos(yaw) - side * math.sin(yaw),
            out * math.sin(yaw) + side * math.cos(yaw),
            float(mount["scan_height"]), yaw, bool(mount["upside_down"]))


def outline_extents(geometry):
    """Outline used for collision checks: distances from the robot center to
    the front (+x), back (-x), left (+y) and right (-y) edges.

    The maximum outline, enlarged where the LiDAR body sticks out further.
    """
    front = back = 0.5 * float(geometry["max_outline"]["length"])
    left = right = 0.5 * float(geometry["max_outline"]["width"])
    extents = {"+x": front, "-x": back, "+y": left, "-y": right}
    lidar = geometry["lidar"]
    x, y, _, yaw, _ = lidar_mount_pose(geometry)
    depth = 0.5 * float(lidar["body_size"][0])
    reach = abs(x * math.cos(yaw) + y * math.sin(yaw)) + depth
    face = lidar["mount"]["face"]
    extents[face] = max(extents[face], reach)
    return extents


def outline_polygon(geometry):
    """Outline rectangle corners in base_footprint (counter-clockwise)."""
    e = outline_extents(geometry)
    return [(e["+x"], e["+y"]), (-e["-x"], e["+y"]),
            (-e["-x"], -e["-y"]), (e["+x"], -e["-y"])]


def footprint_string(geometry):
    """Nav2 footprint polygon from the outline (maximum outline + LiDAR)."""
    return "[" + ", ".join(f"[{x:.4f}, {y:.4f}]"
                           for x, y in outline_polygon(geometry)) + "]"


def _rectangle(cx, cy, half_along, half_across, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * a - s * b, cy + s * a + c * b)
            for a, b in ((half_along, half_across), (-half_along, half_across),
                         (-half_along, -half_across), (half_along, -half_across))]


def _wheel_section(cx, cy, radius, width, rolling_yaw, z):
    """Horizontal cross-section at height z of a wheel touching the floor."""
    dz = z - radius
    if abs(dz) >= radius:
        return None
    return _rectangle(cx, cy, math.sqrt(radius ** 2 - dz ** 2), 0.5 * width,
                      rolling_yaw)


def robot_sections_at_height(geometry, z):
    """Robot parts cut by the horizontal plane at height z (polygons)."""
    sections = []
    chassis = geometry["chassis"]
    bottom = float(chassis["ground_clearance"])
    if bottom <= z <= bottom + float(chassis["height"]):
        sections.append(_rectangle(0.0, 0.0, 0.5 * float(chassis["length"]),
                                   0.5 * float(chassis["width"]), 0.0))
    _, xs, ys, yaws = drive_wheel_layout(geometry)
    radius = drive_wheel_radius(geometry)
    width = float(geometry["drive_wheels"]["width"])
    for x, y, yaw in zip(xs, ys, yaws):
        section = _wheel_section(x, y, radius, width, yaw, z)
        if section:
            sections.append(section)
    _, radius, xs, ys, angles = tracking_wheel_layout(geometry)
    width = float(geometry["tracking_wheels"]["width"])
    for x, y, angle in zip(xs, ys, angles):
        section = _wheel_section(x, y, radius, width, angle, z)
        if section:
            sections.append(section)
    return sections


def ray_polygon_overlap(ox, oy, dx, dy, polygon, reach):
    """Length of the ray segment [0, reach] inside a convex polygon.

    Cyrus-Beck clipping; a ray that only grazes an edge or corner gives 0.
    """
    t_in, t_out = 0.0, reach
    area = sum(polygon[i][0] * polygon[(i + 1) % len(polygon)][1]
               - polygon[(i + 1) % len(polygon)][0] * polygon[i][1]
               for i in range(len(polygon)))
    orientation = 1.0 if area > 0 else -1.0
    for i in range(len(polygon)):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % len(polygon)]
        # inward normal of edge a->b
        nx, ny = -orientation * (by - ay), orientation * (bx - ax)
        numerator = nx * (ox - ax) + ny * (oy - ay)   # >0 means origin inside
        denominator = nx * dx + ny * dy
        if abs(denominator) < 1e-12:
            if numerator <= 0.0:
                return 0.0
            continue
        t = -numerator / denominator
        if denominator > 0:
            t_in = max(t_in, t)
        else:
            t_out = min(t_out, t)
        if t_in >= t_out:
            return 0.0
    return t_out - t_in


def lidar_self_occlusion(geometry, step_deg=0.05, reach=1.0):
    """Scan angle intervals (rad, in lidar_scan_link) blocked by the robot.

    A ray at the nominal scan height is blocked when it passes through the
    chassis or a wheel cross-section within `reach` meters. Intervals are
    widened by lidar.self_mask_margin_deg and clipped to nothing outside the
    field of view.
    """
    lidar = geometry["lidar"]
    sx, sy, sz, yaw, upside_down = lidar_mount_pose(geometry)
    sections = robot_sections_at_height(geometry, sz)
    half_fov = math.radians(float(lidar["fov_deg"])) / 2
    margin = math.radians(float(lidar["self_mask_margin_deg"]))
    direction = -1.0 if upside_down else 1.0

    blocked = []
    count = int(round(2 * half_fov / math.radians(step_deg)))
    for k in range(count + 1):
        angle = -half_fov + k * 2 * half_fov / count
        heading = yaw + direction * angle
        c, s = math.cos(heading), math.sin(heading)
        hit = any(ray_polygon_overlap(sx, sy, c, s, polygon, reach) > 1e-4
                  for polygon in sections)
        blocked.append((angle, hit))

    intervals = []
    start = None
    for angle, hit in blocked:
        if hit and start is None:
            start = angle
        if not hit and start is not None:
            intervals.append([start, previous])
            start = None
        previous = angle
    if start is not None:
        intervals.append([start, previous])
    return [(lo - margin, hi + margin) for lo, hi in intervals]
