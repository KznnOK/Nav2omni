# Phase 3: tracking wheel odometry vs drive wheel odometry

- Command: `ros2 run omni_robot_gazebo odometry_comparison --cycles 3`
  (simulation started with sim_check.launch.py x:=-1.156 y:=0.5)
- Pattern per cycle: step /cmd_vel without acceleration limit (1.0 m/s in x,
  y, diagonal with instant reversal; 3 rad/s spins; 0.6 m/s + 2 rad/s arcs),
  3 cycles = about 23 s. Drive wheels slip on every step.
- Errors are displacements from the test start compared with Gazebo ground
  truth. "max" values include up to about 10 ms timing mismatch between the
  topics (1 cm at 1 m/s, 1.7 deg at 3 rad/s); "final" is taken at rest.
- No sensor noise beyond the Gazebo IMU noise (phase 5 adds noise models).
