#!/usr/bin/env bash
# Start REMANI against Mode B Gazebo with environment_mode=simulated.
# Requires: Mode B up, filtered /map_generator/global_cloud feeder ready.
# Does NOT fall back to static_empty on collision — fail and inspect.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR="${ROOT}/logs/fkie_agx_warehouse_mode_b"
mkdir -p "$LOGDIR"

CONTAINER="${FKIE_CONTAINER:-nbv_fkie}"
ENV_MODE="${ENV_MODE:-simulated}"

run() {
  if [ -f /.dockerenv ] || [ -n "${INSIDE_NBV_FKIE:-}" ]; then
    bash -lc "$1"
  else
    docker exec "${CONTAINER}" bash -lc "$1"
  fi
}

echo "[remani-sim] ensuring cloud feeder..."
run 'pgrep -af warehouse_cloud_to_remani.py >/dev/null || \
  (nohup python3 /workspace/scripts/warehouse_cloud_to_remani.py \
    >/workspace/logs/fkie_agx_warehouse_mode_b/warehouse_cloud_feeder.log 2>&1 & sleep 3)'

echo "[remani-sim] waiting for filter ready..."
run 'python3 - <<'"'"'PY'"'"'
import json, rospy, time
from std_msgs.msg import String
rospy.init_node("wait_feeder", anonymous=True)
t0=time.time()
while time.time()-t0 < 60:
  try:
    s=json.loads(rospy.wait_for_message("/warehouse_map/self_filter_stats", String, timeout=2).data)
    print(s.get("frames_ok"), s.get("accum_n"), s.get("ready"), s.get("last_skip_reason"))
    if s.get("ready"):
      print("FEEDER_READY"); raise SystemExit(0)
  except SystemExit: raise
  except Exception as e:
    print("wait", e)
raise SystemExit("FEEDER_NOT_READY")
PY'

echo "[remani-sim] validate map C2..."
run 'python3 /workspace/scripts/validate_remani_warehouse_map.py \
  --out /workspace/data/metrics/fkie_agx_warehouse_mode_b/warehouse_map_c2.json'

echo "[remani-sim] launching REMANI environment_mode=${ENV_MODE} (no static_empty fallback)"
run "source /opt/ros/noetic/setup.bash; \
  source /workspace/third_party/wbc/remani_planner/devel/setup.bash; \
  source /workspace/ws_fkie/devel/setup.bash 2>/dev/null || true; \
  export ROS_MASTER_URI=\${ROS_MASTER_URI:-http://127.0.0.1:11311}; \
  roslaunch /workspace/configs/fkie/agx_adapt/launch/remani_agx_warehouse.launch \
    environment_mode:=${ENV_MODE} \
    > /workspace/logs/fkie_agx_warehouse_mode_b/remani_${ENV_MODE}.log 2>&1 & \
  echo REMANI_LAUNCHED"

echo "[remani-sim] logs: logs/fkie_agx_warehouse_mode_b/remani_${ENV_MODE}.log"
echo "REMANI_SIMULATED_START_OK"
