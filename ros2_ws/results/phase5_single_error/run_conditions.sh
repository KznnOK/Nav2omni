#!/bin/bash
# Phase 5 single errors (CONDS="..." to choose conditions; LIDARS="true" for correction only; SUFFIX=_x appended to labels; EXTRA_SET="--set max_speed=2.0 ..." extra mission parameters): per condition one launch, 3 runs with LiDAR correction, 3 without.
W=/home/ubuntu/robocon_study/ros2/omni_nav2/ros2_ws
R=$W/results/phase5_single_error
S=${LOGDIR:-/tmp/phase5_logs}; mkdir -p $S
C=$W/results/phase5_single_errors/conditions
OUT=${OUT:-$R/runs_resume.csv}; mkdir -p $R/timeseries
source /opt/ros/humble/setup.bash
cd $W && source install/setup.bash
for cond in ${CONDS:-baseline imu_bias imu_drift trk_diam_p1 trk_diam_m1 trk_angle_1deg trk_pos_3mm trk_lift trk_quant lidar_pitch_0.5 lidar_pitch_1 lidar_roll_0.5 lidar_roll_1 lidar_yaw_1 lidar_pos_5mm drive_mu2_0.4}; do
  ok=0
  for attempt in 1 2 3; do
    $R/stop_sim.sh >/dev/null
    LOG=$S/p5_$cond.launch.log
    (setsid ros2 launch omni_robot_bringup sim_mission.launch.py gui:=false rviz:=false sim_conditions:=$C/$cond.yaml > $LOG 2>&1 &)
    for i in $(seq 1 90); do grep -q "map received" $LOG && grep -q "Managed nodes are active" $LOG && break; sleep 1; done
    if grep -q "Managed nodes are active" $LOG && grep -q "Successfully spawned" $LOG; then ok=1; break; fi
    echo "$cond start failed (attempt $attempt)"
  done
  [ $ok = 1 ] || { echo "$cond SKIPPED"; continue; }
  sleep 3
  for lidar in ${LIDARS:-true false}; do for k in 1 2 3; do
    L=${cond}${SUFFIX}_lidar${lidar}_$k
    echo "run $L $(date +%T)"
    timeout 400 ros2 run omni_robot_gazebo mission_trial --trials 1 --label $L --timeout 60 \
      --set use_lidar_correction=$lidar $EXTRA_SET \
      --timeseries $R/timeseries/$L.csv --csv $OUT 2>&1 | grep -oE "Traceback|Error|time_total_s=[0-9.]*|success=[A-Za-z]*" | tr '\n' ' '; echo
  done; done
done
$R/stop_sim.sh >/dev/null
echo ALLDONE
