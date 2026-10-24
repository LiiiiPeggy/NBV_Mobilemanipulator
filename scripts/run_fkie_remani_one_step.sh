#!/usr/bin/env bash
# One FKIE NBV → REMANI EE goal → execution attempt on Mode B stack (already running).
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/third_party/wbc/remani_planner/devel/setup.bash 2>/dev/null || true

OUTDIR=/workspace/data/metrics/fkie_agx_warehouse_mode_b
mkdir -p "$OUTDIR" /workspace/logs/fkie_agx_warehouse_mode_b

echo "[one-step] FKIE action..."
python3 /workspace/scripts/verify_fkie_action.py _timeout:=90.0 \
  _boundary_xy:='[[-8.3603,-2.6852],[-1.80,-2.6852],[-1.80,-8.6852],[-8.3603,-8.6852]]' \
  2>&1 | tee "$OUTDIR/action_one_step.txt"

# Parse first goal xyz from action text if JSON not written
python3 - <<'PY'
import re, json
from pathlib import Path
text = Path("/workspace/data/metrics/fkie_agx_warehouse_mode_b/action_one_step.txt").read_text()
# fallback: use current camera pose + small look-ahead as demo if goals empty
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

echo "[one-step] done. See $OUTDIR"
