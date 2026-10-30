# LiDAR mounting face comparison

Candidates: center of the +x, +y, -x, -y chassis face, scan height 60 mm,
sensor body 60 mm deep with the scan origin on the chassis face (protrudes
30 mm, included in the outline). Default in robot_geometry.yaml is still -y
(decision pending).

## How it was produced

- Self mask, outline, start region, route collisions (geometry only):
  `ros2 run omni_robot_gazebo lidar_mount_study --geometry <file> --route src/omni_robot_gazebo/config/route_check_points.yaml`
  -> `start_region_<face>.txt`
- Wall visibility (simulation, same procedure as phase 2):
  `sim_check.launch.py gui:=false rviz:=false geometry_file:=<file>` then
  `ros2 run omni_robot_gazebo scan_visibility_check` -> `visibility_<face>.csv`
  Route points are the provisional ones in route_check_points.yaml
  (start at (-1.025, -3.112, 0) for every face so the tables are comparable).

## Start region rules

Nominal center: right edge 0.05 m (仮値) left of the dice-zone line, centered
in area 1. A pose is allowed when the outline (max outline + LiDAR body)
stays inside area 1, touches no frame or the dice marker, and no frame
inside the unmasked field of view is closer than range_min = 0.1 m.
Searched: dx +-0.30 m, dy +-0.15 m, yaw +-10 deg.
`start_feasible_fraction`: share of that box (grid) that is allowed.
