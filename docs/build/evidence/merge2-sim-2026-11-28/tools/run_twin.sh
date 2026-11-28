#!/bin/bash
# usage: run_twin.sh <vehicle> <outdir>
# Headless twin + SLAM + Nav2, a short mapping drive, two goals, everything recorded.
V=$1; O=$2; DRIVE=${3:-8}; mkdir -p "$O"
HERE=$(dirname "$(readlink -f "$0")")
cd "$(git -C "$HERE" rev-parse --show-toplevel)/ros2_ws" || exit 1
set +u
source /opt/ros/humble/setup.bash
source setup_env.sh >/dev/null 2>&1
source install/setup.bash

stop_all() {
  pkill -INT -f "[r]os2 launch mitt_"; sleep 6
  for p in "[g]z sim" "[s]lam_toolbox" "[e]kf_node" "[r]obot_state_publisher" \
           "[p]arameter_bridge" "[t]wist_mux" "[c]ontroller_server" "[p]lanner_server" \
           "[b]t_navigator" "[l]ifecycle_manager" "[b]ehavior_server" \
           "[v]elocity_smoother" "[v]ehicle_config.py" "[s]moother_server" "[w]aypoint_follower"; do
    pkill -9 -f "$p"
  done
}
stop_all; sleep 2

ros2 launch mitt_bringup sim.launch.py vehicle:=$V gz_args:="-r -s" > "$O/sim.log" 2>&1 &
timeout 150 bash -c "until grep -q 'activated .*ackermann_steering_controller' '$O/sim.log'; do sleep 2; done" \
  || { echo "sim did not come up"; stop_all; exit 1; }
ros2 launch mitt_navigation slam.launch.py > "$O/slam.log" 2>&1 &
sleep 20
ros2 launch mitt_navigation nav2.launch.py vehicle:=$V > "$O/nav2.log" 2>&1 &
timeout 120 bash -c "until grep -q 'Managed nodes are active' '$O/nav2.log'; do sleep 2; done" \
  || { echo "nav2 did not come up"; stop_all; exit 1; }

python3 "$HERE/recorder.py" "$O/run.pkl" 110 > "$O/rec.log" 2>&1 &
sleep 4
timeout $DRIVE ros2 topic pub -r 20 /cmd_vel_joy geometry_msgs/msg/Twist "{linear: {x: 0.5}}" > /dev/null 2>&1
sleep 2

goal() {  # x y qz qw
  local n=$(( $(wc -l < "$O/nav2.log") + 1 ))
  timeout 6 ros2 topic pub -r 2 /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {z: $3, w: $4}}}" > /dev/null 2>&1
  timeout 60 bash -c "until tail -n +$n '$O/nav2.log' | grep -q -i 'Goal succeeded\|Goal failed\|Goal was aborted\|Aborting'; do sleep 2; done"
  echo "vehicle $V goal ($1, $2): $(tail -n +$n "$O/nav2.log" | grep -i -o 'Goal succeeded\|Goal failed\|Goal was aborted\|Aborting' | tail -1)" | tee -a "$O/goals.txt"
}
goal 7.9 1.2 0.0 1.0
goal 2.0 4.5 1.0 0.0

timeout 100 bash -c "until grep -q saved '$O/rec.log'; do sleep 2; done"
tail -1 "$O/rec.log"
stop_all
exit 0
