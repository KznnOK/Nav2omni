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


def footprint_string(geometry):
    """Nav2 footprint polygon from the maximum outline."""
    half_l = 0.5 * float(geometry["max_outline"]["length"])
    half_w = 0.5 * float(geometry["max_outline"]["width"])
    return (f"[[{half_l}, {half_w}], [{half_l}, {-half_w}], "
            f"[{-half_l}, {-half_w}], [{-half_l}, {half_w}]]")
