"""Write one sim_conditions yaml per single-error condition (all values 仮値)."""
import copy
import os
import sys

import yaml

BASE = {
    "lidar_mount_error": {"x": 0.0, "y": 0.0, "z": 0.0, "roll_deg": 0.0,
                          "pitch_deg": 0.0, "yaw_deg": 0.0},
    "imu_error": {"gyro_bias_z_deg_s": 0.0,
                  "gyro_bias_random_walk_deg_s_sqrt_s": 0.0, "seed": 1},
    "tracking_wheel_error": {
        "diameter_scale": 1.0,
        "wheels": [{"dx": 0.0, "dy": 0.0, "angle_deg": 0.0},
                   {"dx": 0.0, "dy": 0.0, "angle_deg": 0.0}],
        "quantize": False, "lift_rate_hz": 0.0, "lift_duration_s": 0.02,
        "seed": 1},
    "drive_wheel_error": {"mu2_scale": 1.0},
}

# name: (section, changes). Values are 仮値 (see README).
CONDITIONS = {
    "baseline": [],
    "imu_bias": [("imu_error", {"gyro_bias_z_deg_s": 0.1})],
    "imu_drift": [("imu_error", {"gyro_bias_random_walk_deg_s_sqrt_s": 0.01})],
    "trk_diam_p1": [("tracking_wheel_error", {"diameter_scale": 1.01})],
    "trk_diam_m1": [("tracking_wheel_error", {"diameter_scale": 0.99})],
    "trk_angle_1deg": [("tracking_wheel_error", {"wheels": [
        {"dx": 0.0, "dy": 0.0, "angle_deg": 1.0},
        {"dx": 0.0, "dy": 0.0, "angle_deg": 1.0}]})],
    "trk_pos_3mm": [("tracking_wheel_error", {"wheels": [
        {"dx": 0.003, "dy": 0.003, "angle_deg": 0.0},
        {"dx": 0.003, "dy": 0.003, "angle_deg": 0.0}]})],
    "trk_lift": [("tracking_wheel_error", {"lift_rate_hz": 0.5,
                                           "lift_duration_s": 0.02})],
    "trk_quant": [("tracking_wheel_error", {"quantize": True})],
    "lidar_pitch_0.5": [("lidar_mount_error", {"pitch_deg": 0.5})],
    "lidar_pitch_1": [("lidar_mount_error", {"pitch_deg": 1.0})],
    "lidar_roll_0.5": [("lidar_mount_error", {"roll_deg": 0.5})],
    "lidar_roll_1": [("lidar_mount_error", {"roll_deg": 1.0})],
    "lidar_yaw_1": [("lidar_mount_error", {"yaw_deg": 1.0})],
    "lidar_pos_5mm": [("lidar_mount_error", {"x": 0.005, "y": 0.005})],
    "drive_mu2_0.4": [("drive_wheel_error", {"mu2_scale": 0.4})],
    "drive_mu2_0.6": [("drive_wheel_error", {"mu2_scale": 0.6})],
    "drive_mu2_0.8": [("drive_wheel_error", {"mu2_scale": 0.8})],
}

if __name__ == "__main__":
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for name, changes in CONDITIONS.items():
        cond = copy.deepcopy(BASE)
        for section, values in changes:
            cond[section].update(values)
        with open(os.path.join(out, f"{name}.yaml"), "w") as f:
            yaml.safe_dump(cond, f, sort_keys=False)
    print(" ".join(CONDITIONS))
