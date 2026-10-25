#!/usr/bin/env bash
# C3/C4: FKIE Action → REMANI EE → Gazebo motion → perception delta on the SAME Mode B scene.
# Fail-closed: no regex goals, no default RPY overlay, no || true on critical steps,
# no second /ee_goal overwrite after FKIE conversion publish.
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/third_party/wbc/remani_planner/devel/setup.bash 2>/dev/null || true
source /workspace/ws_fkie/devel/setup.bash 2>/dev/null || true
# Keep remani Python msgs after ws_fkie overlay
export PYTHONPATH="/workspace/third_party/wbc/remani_planner/devel/lib/python3/dist-packages:${PYTHONPATH:-}"

OUTDIR=/workspace/data/metrics/fkie_agx_warehouse_mode_b
LOGDIR=/workspace/logs/fkie_agx_warehouse_mode_b
REPDIR=/workspace/reports
SHOTDIR=/workspace/reports/fkie_remani_one_step_shots
mkdir -p "$OUTDIR" "$LOGDIR" "$SHOTDIR"

SEED="${SEED:-42}"
SCENE_KIND="${SCENE_KIND:-ycb}"
ENSURE_YCB="${ENSURE_YCB:-1}"
ENSURE_MAP="${ENSURE_MAP:-1}"
ENV_MODE="${ENV_MODE:-simulated}"
RUN_PERCEPTION="${RUN_PERCEPTION:-1}"
GOAL_ID="fkie_$(date +%Y%m%d_%H%M%S)"

echo "[one-step] goal_id=${GOAL_ID} ENV_MODE=${ENV_MODE}"

if [ "${ENSURE_YCB}" = "1" ]; then
  echo "[one-step] ensuring YCB seed${SEED} in current Gazebo (only-missing)..."
  export INSIDE_NBV_FKIE=1 ONLY_MISSING=1 SEED="${SEED}" SCENE_KIND="${SCENE_KIND}"
  bash /workspace/scripts/load_ycb_scene_into_gazebo.sh \
    || { echo "[one-step] FAIL: YCB scene not verified"; exit 4; }
fi

python3 - <<'PY'
import rospy
from gazebo_msgs.srv import GetWorldProperties
rospy.init_node("one_step_scene_check", anonymous=True)
rospy.wait_for_service("/gazebo/get_world_properties", timeout=20)
w = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
names = set(w.model_names)
need = {"agx_ranger_cr10", "mo_table_01"}
missing = sorted(need - names)
print("world_models", len(names), "missing", missing)
if missing:
  raise SystemExit("S5B_SCENE_MISMATCH: missing %s" % missing)
n_mo = sum(1 for n in names if n.startswith("mo_obj_"))
if n_mo < 8:
  raise SystemExit("S5B_SCENE_MISMATCH: expected >=8 mo_obj_* got %d" % n_mo)
print("S5B_SCENE_OK")
PY

if [ "${ENSURE_MAP}" = "1" ]; then
  echo "[one-step] validating filtered warehouse map (C2)..."
  # Start feeder if not running
  if ! pgrep -af warehouse_cloud_to_remani.py >/dev/null 2>&1; then
    echo "[one-step] starting warehouse_cloud_to_remani.py"
    nohup python3 /workspace/scripts/warehouse_cloud_to_remani.py \
      >"$LOGDIR/warehouse_cloud_feeder.log" 2>&1 &
    sleep 5
  fi
  # Wait for ready stats
  python3 - <<'PY'
import json, rospy, time
from std_msgs.msg import String
rospy.init_node("wait_map_ready", anonymous=True)
t0 = time.time()
ready = False
while time.time() - t0 < 45 and not rospy.is_shutdown():
  try:
    m = rospy.wait_for_message("/warehouse_map/self_filter_stats", String, timeout=2.0)
    s = json.loads(m.data)
    print("map_stats", {k: s.get(k) for k in ("frames_ok","accum_n","last_removed","ready","last_skip_reason")})
    if s.get("ready"):
      ready = True
      break
  except Exception as e:
    print("wait_map", e)
print("MAP_READY" if ready else "MAP_NOT_READY")
if not ready:
  raise SystemExit(6)
PY
  python3 /workspace/scripts/validate_remani_warehouse_map.py \
    --out "$OUTDIR/warehouse_map_c2.json" \
    | tee "$LOGDIR/warehouse_map_c2.log"
fi

# Pre snapshot
python3 /workspace/scripts/capture_nbv_loop_snapshot.py \
  --tag before --goal-id "$GOAL_ID" \
  --out-dir "$OUTDIR/snap_before" \
  --shot-dir "$SHOTDIR" \
  | tee "$LOGDIR/snap_before.log"

echo "[one-step] FKIE action (structured)..."
python3 /workspace/scripts/save_fkie_action_result.py \
  --out "$OUTDIR/fkie_action_result.json" \
  --timeout 90 \
  --boundary-xy '[[-8.3603,-2.6852],[-1.80,-2.6852],[-1.80,-8.6852],[-8.3603,-8.6852]]' \
  | tee "$LOGDIR/fkie_action.log"

echo "[one-step] convert FKIE → REMANI EE (publish once)..."
python3 /workspace/scripts/nbv_to_remani_ee_goal.py \
  --from-fkie-json "$OUTDIR/fkie_action_result.json" \
  --align-remani-ee --publish \
  --out "$OUTDIR/ee_goal_one_step.json" \
  | tee "$LOGDIR/one_step_convert.log"

# Record goal_id into ee json
python3 - <<PY
import json
from pathlib import Path
p = Path("$OUTDIR/ee_goal_one_step.json")
d = json.loads(p.read_text())
d["goal_id"] = "$GOAL_ID"
d["environment_mode_requested"] = "$ENV_MODE"
p.write_text(json.dumps(d, indent=2))
print("goal_id_recorded", d["goal_id"])
PY

echo "[one-step] monitor REMANI execution (no second publish)..."
python3 /workspace/scripts/execute_single_ee_goal.py \
  --from-json "$OUTDIR/ee_goal_one_step.json" \
  --monitor-only \
  --goal-id "$GOAL_ID" \
  --timeout 120 \
  --out "$OUTDIR/one_step_exec.json" \
  | tee "$LOGDIR/one_step_exec.log"

# Post snapshot after settle
sleep 2
python3 /workspace/scripts/capture_nbv_loop_snapshot.py \
  --tag after --goal-id "$GOAL_ID" \
  --out-dir "$OUTDIR/snap_after" \
  --shot-dir "$SHOTDIR" \
  | tee "$LOGDIR/snap_after.log"

if [ "${RUN_PERCEPTION}" = "1" ]; then
  echo "[one-step] perception delta..."
  python3 /workspace/scripts/compute_nbv_perception_delta.py \
    --before "$OUTDIR/snap_before" \
    --after "$OUTDIR/snap_after" \
    --layout "/workspace/data/scenes/indoor_objects/${SCENE_KIND}/layouts/seed${SEED}.json" \
    --out "$OUTDIR/perception_delta.json" \
    --report "$REPDIR/fkie_remani_perception_delta.md" \
    | tee "$LOGDIR/perception_delta.log"
fi

python3 - <<'PY'
import json
from pathlib import Path
from datetime import datetime
outdir = Path("/workspace/data/metrics/fkie_agx_warehouse_mode_b")
rep = Path("/workspace/reports/fkie_remani_one_step_execution.md")
fkie = json.loads((outdir / "fkie_action_result.json").read_text())
ee = json.loads((outdir / "ee_goal_one_step.json").read_text())
ex = json.loads((outdir / "one_step_exec.json").read_text())
lines = [
  "# FKIE → REMANI one-step execution",
  "",
  "Date: %s" % datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S UTC"),
  "goal_id: `%s`" % ee.get("goal_id"),
  "",
  "## FKIE Action",
  "- state: **%s**" % fkie.get("action_state_name"),
  "- n_goals: **%d**" % fkie.get("n_goals"),
  "- camera_goal_xyz: %s" % fkie.get("camera_goal_xyz"),
  "- orientation_source: %s" % fkie.get("orientation_source"),
  "",
  "## REMANI EE",
  "- ee_goal_xyz: %s" % ee.get("ee_goal_xyz"),
  "- align: %s" % ee.get("align_remani_ee"),
  "- orientation_source: %s" % ee.get("orientation_source"),
  "",
  "## Execution",
  "- ok: **%s**" % ex.get("ok"),
  "- stages: `%s`" % json.dumps(ex.get("stages"), ensure_ascii=False),
  "",
  "Artifacts: `data/metrics/fkie_agx_warehouse_mode_b/`, `reports/fkie_remani_one_step_shots/`",
  "",
  "ONE_STEP_OK" if ex.get("ok") else "ONE_STEP_FAIL",
]
rep.write_text("\n".join(lines) + "\n")
print(lines[-1])
if not ex.get("ok"):
  raise SystemExit(7)
PY

echo "[one-step] done goal_id=${GOAL_ID}"
