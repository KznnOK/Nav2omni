#!/bin/bash
# Phase 5 combined errors. Per condition one launch: 3 standard-start trials and
# 2 random-start trials (seeds RSEEDS), LiDAR correction on, friction limit on
# with the planning mu MU_PLAN. Per trial the number of stall-watchdog
# interventions is counted from the launch log into stalls.csv.
W=/home/ubuntu/robocon_study/ros2/omni_nav2/ros2_ws
R=$W/results/phase5_combo
C=$W/results/phase5_single_errors/conditions
S=${LOGDIR:-/tmp/phase5_combo_logs}; mkdir -p $S $R/timeseries
MU_PLAN=${MU_PLAN:-0.5}
RSEEDS=${RSEEDS:-"11 12"}
OUT=$R/runs.csv
source /opt/ros/humble/setup.bash
cd $W && source install/setup.bash
SETS="--set use_lidar_correction=true --set friction_limit_enabled=true --set mu_drive=$MU_PLAN"
for cond in ${CONDS:-combo_base combo_real combo_severe}; do
  ok=0
  for attempt in 1 2 3; do
    $W/results/phase5_single_error/stop_sim.sh >/dev/null
    LOG=$S/$cond.launch.log
    (setsid ros2 launch omni_robot_bringup sim_mission.launch.py gui:=false rviz:=false sim_conditions:=$C/$cond.yaml > $LOG 2>&1 &)
    for i in $(seq 1 90); do grep -q "map received" $LOG && grep -q "Managed nodes are active" $LOG && break; sleep 1; done
    if grep -q "Managed nodes are active" $LOG && grep -q "Successfully spawned" $LOG; then ok=1; break; fi
    echo "$cond start failed (attempt $attempt)"
  done
  [ $ok = 1 ] || { echo "$cond SKIPPED"; continue; }
  sleep 3
  trial() {  # label, extra args
    L=$1; shift
    before=$(wc -l < $LOG)
    echo "run $L $(date +%T)"
    timeout 400 ros2 run omni_robot_gazebo mission_trial --trials 1 --label $L --timeout 60 \
      $SETS "$@" --timeseries $R/timeseries/$L.csv --csv $OUT 2>&1 | grep -oE "Traceback|Error|time_total_s=[0-9.]*|success=[A-Za-z]*" | tr '\n' ' '; echo
    n=$(tail -n +$((before + 1)) $LOG | grep -c "stall [0-9]/[0-9]*:")
    echo "$L,$n" >> $R/stalls.csv
  }
  for k in 1 2 3; do trial ${cond}_std_lidartrue_$k; done
  k=0; for seed in $RSEEDS; do k=$((k + 1)); trial ${cond}_rnd_lidartrue_$k --random-start --seed $seed; done
done
$W/results/phase5_single_error/stop_sim.sh >/dev/null
echo FINALDONE
