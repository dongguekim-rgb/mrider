#!/usr/bin/env bash
set -e
source /root/mrider-team/ros2_ws/setup_env.sh
source /root/mrider-team/ros2_ws/install-docker/setup.bash
export ROS_DOMAIN_ID=192 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export MRIDER_SIM_REPORT_DIR=/tmp/mrider-registration-recovery/live-rviz
mkdir -p "$MRIDER_SIM_REPORT_DIR"
cd /root/mrider-team
python3 - <<'PY'
import json,pathlib,time
path=pathlib.Path('/tmp/mrider-registration-recovery/live-rviz')
assert pathlib.Path('/proc/87150/comm').read_text().strip()=='rviz2'
(path/'environment.json').write_text(json.dumps(dict(started_epoch=time.time(),ros_domain_id=192,rviz_pid_before=87150,rviz_restarted=False,actor_time_offset_s=227.0,vehicle=1,launch='sensor_only_demo.launch.py scenario:=large_office vehicle:=1 goal_x:=24.0 auto_resume:=true actor_time_offset_s:=227.0 diagnostics:=true rviz:=false'),indent=2)+'\n')
PY
ros2 launch mrider_navigation_sim sensor_only_demo.launch.py \
  scenario:=large_office vehicle:=1 goal_x:=24.0 auto_resume:=true \
  actor_time_offset_s:=227.0 diagnostics:=true rviz:=false > "$MRIDER_SIM_REPORT_DIR/backend.log" 2>&1 &
mrider_backend_pid=$!
printf '%s\n' "$mrider_backend_pid" > "$MRIDER_SIM_REPORT_DIR/backend.pid"
cleanup() {
  kill -INT "$mrider_backend_pid" 2>/dev/null || true
  wait "$mrider_backend_pid" || true
}
trap cleanup EXIT
set +e
python3 /tmp/mrider-run-live-validation.py /root/mrider-team > "$MRIDER_SIM_REPORT_DIR/validation.log" 2>&1
mrider_validation_code=$?
set -e
printf '%s\n' "$mrider_validation_code" > "$MRIDER_SIM_REPORT_DIR/validation-exit-code.txt"
# Allow one display refresh, then stop only this owned backend.
# The evaluator has already revoked driving permission.
python3 - <<'PY'
import json,pathlib,time
p=pathlib.Path('/tmp/mrider-registration-recovery/live-rviz/environment.json')
d=json.loads(p.read_text());d['rviz_pid_after']=87150 if pathlib.Path('/proc/87150/comm').read_text().strip()=='rviz2' else None
d['finished_epoch']=time.time();p.write_text(json.dumps(d,indent=2)+'\n')
time.sleep(1.)
PY
exit "$mrider_validation_code"
