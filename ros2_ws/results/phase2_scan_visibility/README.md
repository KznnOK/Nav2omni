# Phase 2: walls seen by the LiDAR along the provisional route

- Command: `ros2 run omni_robot_gazebo scan_visibility_check`
  (points: omni_robot_gazebo/config/route_check_points.yaml, 仮値 route)
- LiDAR at the -y face center, scan height 60 mm, mask beyond +-88 deg.
- x-wall: points on frame faces whose normal is along x (fix x);
  y-wall: normal along y (fix y); unmatched: > 5 cm from any frame face.
