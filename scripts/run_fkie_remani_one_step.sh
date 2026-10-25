#!/usr/bin/env bash
# S5-B one step on the *same* Mode B Gazebo scene (FKIE + REMANI + D455 + YCB).
# Ensures validated YCB layout is present (fill-missing only; never regenerates).
# Does NOT restart Gazebo/AGX/REMANI.
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/third_party/wbc/remani_planner/devel/setup.bash 2>/dev/null || true

OUTDIR=/workspace/data/metrics/fkie_agx_warehouse_mode_b
mkdir -p "$OUTDIR" /workspace/logs/fkie_agx_warehouse_mode_b

SEED="${SEED:-42}"
SCENE_KIND="${SCENE_KIND:-ycb}"
ENSURE_YCB="${ENSURE_YCB:-1}"

if [ "${ENSURE_YCB}" = "1" ]; then
  echo "[one-step] ensuring YCB seed${SEED} in current Gazebo (only-missing)..."
  export INSIDE_NBV_FKIE=1 ONLY_MISSING=1 SEED="${SEED}" SCENE_KIND="${SCENE_KIND}"
  bash /workspace/scripts/load_ycb_scene_into_gazebo.sh \
    || { echo "[one-step] FAIL: YCB scene not verified in this Gazebo"; exit 4; }
fi

# Confirm AGX + a layout object share the same world
python3 - <<'PY'
import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelState
rospy.init_node("one_step_scene_check", anonymous=True)
rospy.wait_for_service("/gazebo/get_world_properties", timeout=20)
w = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
names = set(w.model_names)
need = {"agx_ranger_cr10", "mo_table_01"}
missing = sorted(need - names)
print("world_models", len(names), "missing", missing)
if missing:
  raise SystemExit("S5B_SCENE_MISMATCH: missing %s — FKIE/REMANI not on YCB Gazebo" % missing)
# at least a few YCB targets
n_mo = sum(1 for n in names if n.startswith("mo_obj_"))
print("n_mo_obj", n_mo)
if n_mo < 8:
  raise SystemExit("S5B_SCENE_MISMATCH: expected >=8 mo_obj_* got %d" % n_mo)
print("S5B_SCENE_OK")
PY

echo "[one-step] FKIE action..."
python3 /workspace/scripts/verify_fkie_action.py _timeout:=90.0 \
  _boundary_xy:='[[-8.3603,-2.6852],[-1.80,-2.6852],[-1.80,-8.6852],[-8.3603,-8.6852]]' \
  2>&1 | tee "$OUTDIR/action_one_step.txt"

python3 - <<'PY'
import re, json
from pathlib import Path
text = Path("/workspace/data/metrics/fkie_agx_warehouse_mode_b/action_one_step.txt").read_text()
m = re.search(r"goal[^\n]*\(([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)\)", text)
out = {"source": "parse_fail"}
if m:
  out = {"camera_goal_xyz": [float(m.group(1)), float(m.group(2)), float(m.group(3))],
         "camera_goal_rpy": [0.0, 0.35, 0.0], "source": "regex",
         "note": "rpy adaptation — FKIE orientation not parsed"}
Path("/workspace/data/metrics/fkie_agx_warehouse_mode_b/fkie_goal_one_step.json").write_text(json.dumps(out, indent=2))
print(out)
PY

python3 /workspace/scripts/nbv_to_remani_ee_goal.py \
  --from-fkie-json /workspace/data/metrics/fkie_agx_warehouse_mode_b/fkie_goal_one_step.json \
  --align-remani-ee --publish \
  --out /workspace/data/metrics/fkie_agx_warehouse_mode_b/ee_goal_one_step.json \
  2>&1 | tee /workspace/logs/fkie_agx_warehouse_mode_b/one_step_convert.log || true

python3 /workspace/scripts/execute_single_ee_goal.py --timeout 90 \
  --delta-xyz 0 0 0 \
  --out /workspace/data/metrics/fkie_agx_warehouse_mode_b/one_step_exec.json \
  2>&1 | tee /workspace/logs/fkie_agx_warehouse_mode_b/one_step_exec.log || true

echo "[one-step] done. See $OUTDIR (same Gazebo — YCB not cleared)"
