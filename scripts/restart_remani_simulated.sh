#!/usr/bin/env bash
# Restart REMANI only (keep Gazebo). environment_mode=simulated.
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/ws_fkie/devel/setup.bash 2>/dev/null || true
# remani overlay last so $(find remani_planner) resolves
source /workspace/third_party/wbc/remani_planner/devel/setup.bash
export ROS_PACKAGE_PATH="/workspace/third_party/wbc/remani_planner/src/REMANI-Planner:${ROS_PACKAGE_PATH:-}"
rospack find remani_planner
rospack find mm_controller

LOG=/workspace/logs/fkie_agx_warehouse_mode_b/remani_simulated.log
mkdir -p /workspace/logs/fkie_agx_warehouse_mode_b

echo "[restart] stopping prior remani binaries..."
# Match executables only (avoid matching this script's argv)
for pat in 'remani_planner/remani_planner_node' 'mm_controller/mm_controller_node'; do
  pids=$(ps -eo pid,cmd | awk -v p="$pat" '$0 ~ p && $0 !~ /awk/ && $0 !~ /restart_remani/ {print $1}')
  if [ -n "${pids}" ]; then
    echo "kill $pat -> $pids"
    # shellcheck disable=SC2086
    kill $pids 2>/dev/null || true
  fi
done
# roslaunch parents for warehouse remani
ps -eo pid,cmd | awk '/remani_agx_warehouse.launch/ && !/awk/ && !/restart_remani/ {print $1}' | while read -r pid; do
  echo "kill launch $pid"
  kill "$pid" 2>/dev/null || true
done
sleep 3

if ! timeout 5 rosservice list 2>/dev/null | grep -q /gazebo/spawn_sdf_model; then
  echo "FAIL: Gazebo spawn service missing — abort (will not recreate world)"
  exit 3
fi
echo "[restart] Gazebo still up"

if ! pgrep -f 'python3 /workspace/scripts/warehouse_cloud_to_remani.py' >/dev/null; then
  echo "[restart] starting feeder"
  python3 /workspace/scripts/warehouse_cloud_to_remani.py --skip 6 \
    >/workspace/logs/fkie_agx_warehouse_mode_b/warehouse_cloud_feeder.log 2>&1 &
  sleep 5
fi

python3 - <<'PY'
import json, rospy, time
from std_msgs.msg import String
rospy.init_node("wait_feeder_restart", anonymous=True)
t0 = time.time()
while time.time() - t0 < 45:
    s = json.loads(rospy.wait_for_message("/warehouse_map/self_filter_stats", String, timeout=3).data)
    print("feeder", s.get("ready"), s.get("accum_n"), s.get("last_removed"))
    if s.get("ready"):
        break
else:
    raise SystemExit("feeder not ready")
PY

echo "[restart] launching environment_mode:=simulated"
# Clear old log
: > "$LOG"
roslaunch /workspace/configs/fkie/agx_adapt/launch/remani_agx_warehouse.launch \
  environment_mode:=simulated >>"$LOG" 2>&1 &
LPID=$!
echo "roslaunch_pid=$LPID"
sleep 15

ps -eo pid,cmd | awk '/remani_planner_node|mm_controller_node/ && !/awk/ {print}'
echo "--- log head ---"
head -80 "$LOG" || true
echo "--- collisions ---"
grep -E "BACKEND COLLISION|environment_mode|ERROR|use_global" "$LOG" | head -40 || true
# param
rosparam get /environment_mode 2>/dev/null || rosparam get /remani_planner_node/environment_mode 2>/dev/null || true
echo "REMANI_SIM_RESTART_DONE"
