#!/usr/bin/env bash
# Minimal P2→P3→P5(+P4-A dry) loop against AGX Mode A warehouse Gazebo.
# Does NOT rewrite verified FKIE planner/pose configs. Does not touch other containers.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SEED="${SEED:-42}"
BACKEND="${BACKEND:-color}"
START_AGX="${START_AGX:-0}"

LAYOUT="data/scenes/indoor_objects/layouts/seed${SEED}.json"
GT="data/scenes/indoor_objects/gt/seed${SEED}_gt.json"
PRED="data/metrics/indoor_objects/seed${SEED}_pred.json"
EVAL="data/metrics/indoor_objects/seed${SEED}_eval.json"
EVAL_CSV="data/metrics/indoor_objects/seed${SEED}_eval.csv"
EE_OUT="data/metrics/nbv_remani/ee_goal_seed${SEED}.json"
REPORT="reports/indoor_objects_min_loop.md"

mkdir -p "$(dirname "${ROOT}/${LAYOUT}")" "$(dirname "${ROOT}/${GT}")" \
  "$(dirname "${ROOT}/${PRED}")" "$(dirname "${ROOT}/${EE_OUT}")" "${ROOT}/reports"
chmod +x "${ROOT}/scripts/"*.sh 2>/dev/null || true

python3 "${ROOT}/scripts/generate_object_scene.py" --seed "${SEED}" --out "${ROOT}/${LAYOUT}"

run_in() {
  docker exec -e NBV_ROOT=/workspace -e DISPLAY="${DISPLAY:-:1}" \
    -e GAZEBO_MASTER_URI=http://127.0.0.1:11355 \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e GAZEBO_MODEL_PATH="/workspace/third_party/gazebo_objects/gazebo_ycb/models:/workspace/configs/fkie/agx_adapt/models:/workspace/third_party/gazebo_models:/usr/share/gazebo-11/models" \
    nbv_fkie bash -lc "source /opt/ros/noetic/setup.bash; if [ -f /workspace/ws_fkie/devel/setup.bash ]; then source /workspace/ws_fkie/devel/setup.bash; fi; $*"
}

if [ "${START_AGX}" = "1" ]; then
  echo "[min-loop] stopping previous Gazebo in nbv_fkie..."
  docker exec nbv_fkie bash -lc 'STOP_SIM_KEEP_PID=$$ bash /workspace/scripts/fkie_stop_sim.sh || true'
  sleep 3
  bash "${ROOT}/scripts/setup_x11.sh" || true
  echo "[min-loop] starting AGX warehouse Mode A (background)..."
  docker exec -d nbv_fkie bash -lc '
    export DISPLAY=:1 QT_X11_NO_MITSHM=1
    export GAZEBO_MASTER_URI=http://127.0.0.1:11355
    export ROS_MASTER_URI=http://localhost:11311
    export GAZEBO_MODEL_PATH="/workspace/third_party/gazebo_objects/gazebo_ycb/models:/workspace/configs/fkie/agx_adapt/models:/workspace/third_party/gazebo_models:${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}"
    mkdir -p /workspace/logs/fkie_agx_warehouse_nbv
    nohup bash /workspace/scripts/fkie_agx_warehouse_nbv.sh >> /workspace/logs/fkie_agx_warehouse_nbv/console.log 2>&1 &
  '
fi

echo "[min-loop] waiting for Gazebo spawn API + RGB..."
for i in $(seq 1 150); do
  if run_in 'timeout 2 rosservice list 2>/dev/null | grep -q /gazebo/spawn_sdf_model' \
     && run_in 'timeout 3 rostopic echo -n1 /realsense/rgb/image_raw >/dev/null 2>&1'; then
    echo "[min-loop] ready (${i}x2s)"
    break
  fi
  sleep 2
  if [ "$i" -eq 150 ]; then
    echo "[min-loop] FAIL: gazebo/rgb not ready"
    run_in 'tail -40 /workspace/logs/fkie_agx_warehouse_nbv/console.log || true'
    exit 2
  fi
done

echo "[min-loop] spawn"
run_in "python3 /workspace/scripts/spawn_object_scene.py --layout /workspace/${LAYOUT}"

echo "[min-loop] export GT"
run_in "python3 /workspace/scripts/export_object_gt.py --layout /workspace/${LAYOUT} --out /workspace/${GT}"

echo "[min-loop] perceive backend=${BACKEND}"
run_in "python3 /workspace/scripts/perceive_rgbd_objects.py --backend ${BACKEND} --out /workspace/${PRED} --views 1"

echo "[min-loop] evaluate"
python3 "${ROOT}/scripts/evaluate_object_discovery.py" \
  --gt "${ROOT}/${GT}" --pred "${ROOT}/${PRED}" \
  --dist-thresh 0.15 \
  --out-json "${ROOT}/${EVAL}" --out-csv "${ROOT}/${EVAL_CSV}"

echo "[min-loop] NBV camera→EE dry-run"
run_in "python3 /workspace/scripts/nbv_to_remani_ee_goal.py --cam-xyz -4.419 -6.198 1.609 --cam-rpy 0 0.35 0 --out /workspace/${EE_OUT}" || \
  echo "[min-loop] WARN: ee transform failed (TF/camera frames?)"

# Compact report
python3 - <<PY
import json, pathlib, datetime
root = pathlib.Path("${ROOT}")
gt = json.loads((root/"${GT}").read_text()) if (root/"${GT}").is_file() else {}
pred = json.loads((root/"${PRED}").read_text()) if (root/"${PRED}").is_file() else {}
ev = json.loads((root/"${EVAL}").read_text()) if (root/"${EVAL}").is_file() else {}
ee = json.loads((root/"${EE_OUT}").read_text()) if (root/"${EE_OUT}").is_file() else {}
lines = [
  "# Indoor objects min loop",
  "",
  "time: %s" % datetime.datetime.now().isoformat(timespec="seconds"),
  "seed: ${SEED}",
  "backend: ${BACKEND}",
  "n_gt: %s" % gt.get("n_objects"),
  "n_pred_localized: %s" % (pred.get("views",[{}])[0].get("n_localized") if pred.get("views") else None),
  "precision: %s" % ev.get("precision"),
  "unique_recall: %s" % ev.get("unique_recall"),
  "mean_center_error_m: %s" % ev.get("mean_center_error_m"),
  "ee_goal_xyz: %s" % ee.get("ee_goal_xyz"),
  "",
  "artifacts:",
  "- ${LAYOUT}",
  "- ${GT}",
  "- ${PRED}",
  "- ${EVAL}",
  "- ${EE_OUT}",
]
(root/"${REPORT}").write_text("\\n".join(lines)+"\\n")
print("Wrote", root/"${REPORT}")
PY

echo "[min-loop] OK — see ${REPORT}"
