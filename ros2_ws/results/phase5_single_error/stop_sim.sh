#!/bin/bash
# Stop all simulation-related processes by PID (avoids pkill self-match).
pids=$(ps -eo pid,args | grep -E "ros2 launch|gzserver|gzclient|/opt/ros/humble/lib/|omni_robot_ws|ros2_ws/install|rviz2" | grep -v grep | awk '{print $1}')
[ -n "$pids" ] && kill -INT $pids 2>/dev/null
for i in 1 2 3 4 5 6 7 8; do
  pids=$(ps -eo pid,args | grep -E "ros2 launch|gzserver|gzclient|/opt/ros/humble/lib/|ros2_ws/install|rviz2" | grep -v grep | awk '{print $1}')
  [ -z "$pids" ] && break
  sleep 1
done
[ -n "$pids" ] && kill -9 $pids 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
echo "remaining: $(ps -eo args | grep -E 'gzserver|ros2_ws/install|/opt/ros/humble/lib/' | grep -vc grep)"
