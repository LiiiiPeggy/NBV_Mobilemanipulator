#!/usr/bin/env bash
# F3/F4: Online FKIE → REMANI → sync RGB-D → GSAM → object memory loop.
# Fail-closed on empty FKIE NBV; never invent EE goals; never reuse RGB-D across steps.
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/third_party/wbc/remani_planner/devel/setup.bash 2>/dev/null || true
source /workspace/ws_fkie/devel/setup.bash 2>/dev/null || true
export PYTHONPATH="/workspace/third_party/wbc/remani_planner/devel/lib/python3/dist-packages:${PYTHONPATH:-}"

SEED="${SEED:-42}"
SCENE_KIND="${SCENE_KIND:-ycb}"
MAX_STEPS="${MAX_STEPS:-5}"
ENSURE_YCB="${ENSURE_YCB:-1}"
ENSURE_MAP="${ENSURE_MAP:-1}"
RUN_ID="${RUN_ID:-online_$(date +%Y%m%d_%H%M%S)_seed${SEED}}"
GT="${GT:-/workspace/data/scenes/indoor_objects/ycb/gt/seed${SEED}_gt.json}"
CLASSES="${CLASSES:-banana,apple,bottle,bowl,box,can,cup,cracker,mustard,mug,orange,peach,pear,plum,gelatin,meat,sugar,tomato,sponge}"
DEVICE="${DEVICE:-}"

RUN_DIR="/workspace/data/metrics/online_exploration/${RUN_ID}"
LOGDIR="/workspace/logs/online_exploration/${RUN_ID}"
mkdir -p "$RUN_DIR" "$LOGDIR" /workspace/logs/online_exploration
RESET_OCTOMAP="${RESET_OCTOMAP:-0}"
RESTART_FKIE="${RESTART_FKIE:-1}"

cat >"$RUN_DIR/config.json" <<EOF
{
  "run_id": "$RUN_ID",
  "seed": $SEED,
  "scene_kind": "$SCENE_KIND",
  "max_steps": $MAX_STEPS,
  "gt": "$GT",
  "classes": "$CLASSES",
  "mode": "online_fkie_remani_gsam",
  "started_utc": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

echo "[explore] run_id=${RUN_ID} max_steps=${MAX_STEPS}"

if [ "${ENSURE_YCB}" = "1" ]; then
  export INSIDE_NBV_FKIE=1 ONLY_MISSING=1 SEED="${SEED}" SCENE_KIND="${SCENE_KIND}"
  bash /workspace/scripts/load_ycb_scene_into_gazebo.sh \
    || { echo "[explore] FAIL: YCB scene"; exit 4; }
fi

python3 - <<'PY'
import rospy
from gazebo_msgs.srv import GetWorldProperties
rospy.init_node("explore_scene_check", anonymous=True)
rospy.wait_for_service("/gazebo/get_world_properties", timeout=20)
w = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
names = set(w.model_names)
need = {"agx_ranger_cr10", "mo_table_01"}
missing = sorted(need - names)
n_mo = sum(1 for n in names if n.startswith("mo_obj_"))
print("missing", missing, "n_mo", n_mo)
if missing or n_mo < 8:
  raise SystemExit("SCENE_MISMATCH")
print("SCENE_OK")
PY

if [ "${ENSURE_MAP}" = "1" ]; then
  if ! pgrep -af warehouse_cloud_to_remani.py >/dev/null 2>&1; then
    nohup python3 /workspace/scripts/warehouse_cloud_to_remani.py \
      >"$LOGDIR/warehouse_cloud_feeder.log" 2>&1 &
    sleep 5
  fi
  python3 - <<'PY'
import json, rospy, time
from std_msgs.msg import String
rospy.init_node("wait_map_ready", anonymous=True)
t0 = time.time(); ready=False
# Accept ready flag OR sufficient accum (voxel scenes can plateau <500).
while time.time()-t0 < 60 and not rospy.is_shutdown():
  try:
    m = rospy.wait_for_message("/warehouse_map/self_filter_stats", String, timeout=2.0)
    s = json.loads(m.data)
    print("map_stats", {k:s.get(k) for k in ("frames_ok","accum_n","ready","last_skip_reason")})
    if s.get("ready") or (int(s.get("accum_n") or 0) >= 250 and int(s.get("frames_ok") or 0) >= 5):
      ready=True; break
  except Exception as e:
    print("wait_map", e)
if not ready:
  raise SystemExit("MAP_NOT_READY")
print("MAP_READY")
PY
fi

if [ "${RESET_OCTOMAP}" = "1" ]; then
  echo "[explore] resetting OctoMap for fresh unknown space..."
  rosservice call /robot/octomap_server/reset "{}" || true
  sleep 8
fi
if [ "${RESTART_FKIE}" = "1" ]; then
  echo "[explore] restarting FKIE nbvplanner (clear visited)..."
  bash /workspace/scripts/restart_fkie_nbv_planner.sh | tee "$LOGDIR/fkie_restart.log"
  sleep 5
fi

# Step 0: initial observation (empty memory)
DEVICE_ARGS=()
if [ -n "$DEVICE" ]; then DEVICE_ARGS=(--device "$DEVICE"); fi

python3 /workspace/scripts/run_fkie_online_gsam_step.py \
  --run-dir "$RUN_DIR" --step 0 --max-steps "$MAX_STEPS" \
  --gt "$GT" --classes "$CLASSES" "${DEVICE_ARGS[@]}" \
  | tee "$LOGDIR/step000.log"

# Steps 1..MAX_STEPS: FKIE NBV + motion + fresh obs
TERMINATED=0
TERMINATE_REASON=""
for s in $(seq 1 "$MAX_STEPS"); do
  echo "[explore] === step $s / $MAX_STEPS ==="
  set +e
  python3 /workspace/scripts/run_fkie_online_gsam_step.py \
    --run-dir "$RUN_DIR" --step "$s" --max-steps "$MAX_STEPS" \
    --gt "$GT" --classes "$CLASSES" "${DEVICE_ARGS[@]}" \
    | tee "$LOGDIR/step$(printf '%03d' "$s").log"
  rc=${PIPESTATUS[0]}
  set -e
  if [ "$rc" -eq 10 ]; then
    TERMINATED=1
    TERMINATE_REASON="no_usable_nbv"
    echo "[explore] legal terminate: no usable FKIE NBV at step $s"
    break
  elif [ "$rc" -eq 11 ]; then
    TERMINATED=1
    TERMINATE_REASON="remani_exec_fail"
    echo "[explore] terminate: REMANI exec fail at step $s"
    break
  elif [ "$rc" -ne 0 ]; then
    TERMINATED=1
    TERMINATE_REASON="step_error_$rc"
    echo "[explore] terminate: step error rc=$rc"
    break
  fi
done

python3 /workspace/scripts/summarize_online_exploration.py \
  --run-dir "$RUN_DIR" \
  --terminated "$TERMINATED" \
  --terminate-reason "$TERMINATE_REASON" \
  | tee "$LOGDIR/summary.log"

echo "[explore] done run_id=${RUN_ID} dir=${RUN_DIR}"
