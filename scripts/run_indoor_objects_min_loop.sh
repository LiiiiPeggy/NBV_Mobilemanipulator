#!/usr/bin/env bash
# Minimal indoor-objects loop against AGX Mode A warehouse Gazebo.
# SCENE_KIND=primitive_debug|ycb — layouts/GT/metrics stay in separate trees.
# Does NOT rewrite verified FKIE planner/pose configs. Does not touch other containers.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SEED="${SEED:-42}"
BACKEND="${BACKEND:-color}"
START_AGX="${START_AGX:-0}"
SCENE_KIND="${SCENE_KIND:-primitive_debug}"
DIST_THRESH="${DIST_THRESH:-0.15}"
WORLD_TF_MODE="${WORLD_TF_MODE:-tf}"
SPAWN_PROFILE="${SPAWN_PROFILE:-aisle_safe}"

if [ "${SCENE_KIND}" = "ycb" ]; then
  LAYOUT="data/scenes/indoor_objects/ycb/layouts/seed${SEED}.json"
  GT="data/scenes/indoor_objects/ycb/gt/seed${SEED}_gt.json"
  PRED="data/metrics/indoor_objects/ycb/seed${SEED}_pred.json"
  EVAL="data/metrics/indoor_objects/ycb/seed${SEED}_eval.json"
  EVAL_CSV="data/metrics/indoor_objects/ycb/seed${SEED}_eval.csv"
  REPORT="reports/indoor_objects_ycb_min_loop.md"
elif [ "${SCENE_KIND}" = "primitive_debug" ]; then
  LAYOUT="data/scenes/indoor_objects/primitive_debug/layouts/seed${SEED}.json"
  GT="data/scenes/indoor_objects/primitive_debug/gt/seed${SEED}_gt.json"
  PRED="data/metrics/indoor_objects/primitive_debug/seed${SEED}_pred.json"
  EVAL="data/metrics/indoor_objects/primitive_debug/seed${SEED}_eval.json"
  EVAL_CSV="data/metrics/indoor_objects/primitive_debug/seed${SEED}_eval.csv"
  REPORT="reports/indoor_objects_primitive_debug_min_loop.md"
else
  echo "Unknown SCENE_KIND=${SCENE_KIND} (use primitive_debug|ycb)"; exit 1
fi

EE_OUT="data/metrics/nbv_remani/ee_goal_seed${SEED}_${SCENE_KIND}.json"

mkdir -p "$(dirname "${ROOT}/${LAYOUT}")" "$(dirname "${ROOT}/${GT}")" \
  "$(dirname "${ROOT}/${PRED}")" "$(dirname "${ROOT}/${EE_OUT}")" "${ROOT}/reports"
chmod +x "${ROOT}/scripts/"*.sh 2>/dev/null || true

# YCB: generate layout if missing. Never overwrite primitive_debug with generate_object_scene.
if [ "${SCENE_KIND}" = "ycb" ]; then
  if [ ! -f "${ROOT}/${LAYOUT}" ]; then
    python3 "${ROOT}/scripts/generate_object_scene.py" --seed "${SEED}" --out "${ROOT}/${LAYOUT}"
  fi
elif [ ! -f "${ROOT}/${LAYOUT}" ]; then
  echo "[min-loop] FAIL: missing primitive layout ${LAYOUT}; run spawn_visible_primitives.py first"
  exit 2
fi

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
  echo "[min-loop] starting AGX warehouse Mode A profile=${SPAWN_PROFILE}..."
  docker exec -d nbv_fkie bash -lc "
    export DISPLAY=:1 QT_X11_NO_MITSHM=1
    export GAZEBO_MASTER_URI=http://127.0.0.1:11355
    export ROS_MASTER_URI=http://localhost:11311
    export SPAWN_PROFILE=${SPAWN_PROFILE}
    export GAZEBO_MODEL_PATH=\"/workspace/third_party/gazebo_objects/gazebo_ycb/models:/workspace/configs/fkie/agx_adapt/models:/workspace/third_party/gazebo_models:\${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}\"
    mkdir -p /workspace/logs/fkie_agx_warehouse_nbv
    nohup bash /workspace/scripts/fkie_agx_warehouse_nbv.sh >> /workspace/logs/fkie_agx_warehouse_nbv/console.log 2>&1 &
  "
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

echo "[min-loop] scene_kind=${SCENE_KIND} layout=${LAYOUT} dist_thresh=${DIST_THRESH} world_tf=${WORLD_TF_MODE}"

if [ "${SCENE_KIND}" = "ycb" ]; then
  echo "[min-loop] spawn YCB layout"
  run_in "python3 /workspace/scripts/spawn_object_scene.py --layout /workspace/${LAYOUT}"
elif [ "${SCENE_KIND}" = "primitive_debug" ]; then
  echo "[min-loop] spawn uses existing primitive layout (no generate overwrite)"
  run_in "python3 /workspace/scripts/spawn_object_scene.py --layout /workspace/${LAYOUT}" || \
    echo "[min-loop] WARN: spawn_object_scene failed for primitives; try spawn_visible_primitives.py"
fi

echo "[min-loop] export GT"
run_in "python3 /workspace/scripts/export_object_gt.py --layout /workspace/${LAYOUT} --out /workspace/${GT}"

echo "[min-loop] perceive backend=${BACKEND} world_tf=${WORLD_TF_MODE}"
run_in "python3 /workspace/scripts/perceive_rgbd_objects.py --backend ${BACKEND} --world-tf-mode ${WORLD_TF_MODE} --out /workspace/${PRED} --views 1"

echo "[min-loop] evaluate dist_thresh=${DIST_THRESH} (+ diagnostic 0.10/0.35)"
python3 "${ROOT}/scripts/evaluate_object_discovery.py" \
  --gt "${ROOT}/${GT}" --pred "${ROOT}/${PRED}" \
  --dist-thresh "${DIST_THRESH}" \
  --out-json "${ROOT}/${EVAL}" --out-csv "${ROOT}/${EVAL_CSV}"
for t in 0.10 0.35; do
  python3 "${ROOT}/scripts/evaluate_object_discovery.py" \
    --gt "${ROOT}/${GT}" --pred "${ROOT}/${PRED}" \
    --dist-thresh "$t" \
    --out-json "${ROOT}/data/metrics/indoor_objects/${SCENE_KIND}/seed${SEED}_eval_t${t}.json" \
    --out-csv "${ROOT}/data/metrics/indoor_objects/${SCENE_KIND}/seed${SEED}_eval_t${t}.csv" || true
done

echo "[min-loop] NBV camera→EE dry-run"
run_in "python3 /workspace/scripts/nbv_to_remani_ee_goal.py --cam-xyz -4.419 -6.198 1.609 --cam-rpy 0 0.35 0 --out /workspace/${EE_OUT}" || \
  echo "[min-loop] WARN: ee transform failed (TF/camera frames?)"

python3 - <<PY
import json, pathlib, datetime, hashlib
root = pathlib.Path("${ROOT}")
layout_p = root/"${LAYOUT}"
gt = json.loads((root/"${GT}").read_text()) if (root/"${GT}").is_file() else {}
pred = json.loads((root/"${PRED}").read_text()) if (root/"${PRED}").is_file() else {}
ev = json.loads((root/"${EVAL}").read_text()) if (root/"${EVAL}").is_file() else {}
ee = json.loads((root/"${EE_OUT}").read_text()) if (root/"${EE_OUT}").is_file() else {}
layout_hash = hashlib.sha256(layout_p.read_bytes()).hexdigest()[:12] if layout_p.is_file() else None
layout = json.loads(layout_p.read_text()) if layout_p.is_file() else {}
diag = {}
for t in ("0.10", "0.35"):
  p = root / f"data/metrics/indoor_objects/${SCENE_KIND}/seed${SEED}_eval_t{t}.json"
  if p.is_file():
    d = json.loads(p.read_text())
    diag[t] = {"precision": d.get("precision"), "unique_recall": d.get("unique_recall"), "tp": d.get("tp")}
lines = [
  "# Indoor objects min loop",
  "",
  "time: %s" % datetime.datetime.now().isoformat(timespec="seconds"),
  "scene_kind: ${SCENE_KIND}",
  "scene_id: %s" % layout.get("scene_id", layout.get("notes")),
  "layout_hash: %s" % layout_hash,
  "seed: ${SEED}",
  "backend: ${BACKEND}",
  "world_tf_mode: ${WORLD_TF_MODE}",
  "dist_thresh_m (formal): ${DIST_THRESH}",
  "spawn_profile: ${SPAWN_PROFILE}",
  "n_gt: %s" % gt.get("n_objects"),
  "n_pred_localized: %s" % (pred.get("views",[{}])[0].get("n_localized") if pred.get("views") else None),
  "precision: %s" % ev.get("precision"),
  "unique_recall: %s" % ev.get("unique_recall"),
  "mean_center_error_m: %s" % ev.get("mean_center_error_m"),
  "diag_thresh: %s" % json.dumps(diag),
  "ee_goal_xyz: %s" % ee.get("ee_goal_xyz"),
  "",
  "note: color/heuristic backend ≠ Grounded-SAM recognition.",
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
