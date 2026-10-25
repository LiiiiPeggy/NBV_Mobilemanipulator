#!/usr/bin/env bash
# Load validated YCB layout into a *running* Gazebo (Mode A or Mode B).
# Does NOT regenerate layout. Does NOT restart AGX/REMANI/Gazebo.
# Default: seed42 under data/scenes/indoor_objects/ycb/layouts/seed42.json
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SCENE_KIND="${SCENE_KIND:-ycb}"
SEED="${SEED:-42}"
LAYOUT="${LAYOUT:-data/scenes/indoor_objects/${SCENE_KIND}/layouts/seed${SEED}.json}"
SPAWNED_OUT="${SPAWNED_OUT:-data/scenes/indoor_objects/${SCENE_KIND}/layouts/seed${SEED}.spawned.json}"
VERIFY_OUT="${VERIFY_OUT:-data/metrics/fkie_agx_warehouse_mode_b/ycb_seed${SEED}_verify.json}"
REPORT="${REPORT:-reports/mode_b_ycb_scene.md}"
SHOTDIR="${SHOTDIR:-reports/mode_b_ycb_shots}"
LOGDIR="${LOGDIR:-logs/fkie_agx_warehouse_mode_b}"
ONLY_MISSING="${ONLY_MISSING:-1}"
CONTAINER="${FKIE_CONTAINER:-nbv_fkie}"
INSIDE=0
if [ -f /.dockerenv ] || [ -n "${INSIDE_NBV_FKIE:-}" ]; then
  INSIDE=1
fi

mkdir -p "${ROOT}/${SHOTDIR}" "${ROOT}/${LOGDIR}" "$(dirname "${ROOT}/${VERIFY_OUT}")" "$(dirname "${ROOT}/${REPORT}")"

if [ ! -f "${ROOT}/${LAYOUT}" ]; then
  echo "[ycb-load] FAIL: missing layout ${LAYOUT} (will not regenerate)"
  exit 2
fi

echo "[ycb-load] layout=${LAYOUT} only_missing=${ONLY_MISSING} (no regenerate)"

run() {
  local cmd="$1"
  if [ "${INSIDE}" = "1" ]; then
    bash -lc "source /opt/ros/noetic/setup.bash; \
      source /workspace/ws_fkie/devel/setup.bash 2>/dev/null || true; \
      export NBV_ROOT=/workspace; \
      export GAZEBO_MODEL_PATH=/workspace/third_party/gazebo_objects/gazebo_ycb/models:/workspace/configs/fkie/agx_adapt/models:/workspace/third_party/gazebo_models:\${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}; \
      ${cmd}"
  else
    docker exec "${CONTAINER}" bash -lc "source /opt/ros/noetic/setup.bash; \
      source /workspace/ws_fkie/devel/setup.bash 2>/dev/null || true; \
      export NBV_ROOT=/workspace; \
      export GAZEBO_MODEL_PATH=/workspace/third_party/gazebo_objects/gazebo_ycb/models:/workspace/configs/fkie/agx_adapt/models:/workspace/third_party/gazebo_models:\${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}; \
      ${cmd}"
  fi
}

echo "[ycb-load] waiting for /gazebo/spawn_sdf_model..."
for i in $(seq 1 90); do
  if run 'rosservice list 2>/dev/null | grep -q /gazebo/spawn_sdf_model'; then
    echo "[ycb-load] spawn service ready (${i}s)"
    break
  fi
  sleep 2
  if [ "$i" -eq 90 ]; then
    echo "[ycb-load] FAIL: Gazebo spawn service not ready"
    exit 3
  fi
done

OM_FLAG=""
if [ "${ONLY_MISSING}" = "1" ]; then
  OM_FLAG="--only-missing"
fi

echo "[ycb-load] spawning..."
run "python3 /workspace/scripts/spawn_object_scene.py \
  --layout /workspace/${LAYOUT} \
  --out-json /workspace/${SPAWNED_OUT} \
  ${OM_FLAG}" | tee "${ROOT}/${LOGDIR}/ycb_spawn.log"

echo "[ycb-load] verifying..."
run "python3 /workspace/scripts/verify_gazebo_object_scene.py \
  --layout /workspace/${LAYOUT} \
  --spawned /workspace/${SPAWNED_OUT} \
  --out /workspace/${VERIFY_OUT}" | tee "${ROOT}/${LOGDIR}/ycb_verify.log"

# Inventory + GUI diagnostics
run "python3 /workspace/scripts/diagnose_ycb_gazebo_visibility.py \
  --layout /workspace/${LAYOUT} \
  --out /workspace/data/metrics/fkie_agx_warehouse_mode_b/ycb_seed${SEED}_visibility.json" \
  | tee "${ROOT}/${LOGDIR}/ycb_visibility.log" || true

# Aim Gazebo GUI near table
run 'gz camera -c gzclient_camera -p "-1.6 -5.6 2.2" -t "-3.5 -7.2 0.9" 2>/dev/null || true' || true

if command -v import >/dev/null 2>&1; then
  DISPLAY="${DISPLAY:-:1}" import -window root "${ROOT}/${SHOTDIR}/desktop_$(date +%H%M%S).png" || true
fi
run "python3 /workspace/scripts/capture_ycb_scene_evidence.py \
  --out-dir /workspace/${SHOTDIR} --seed ${SEED}" 2>/dev/null || true

# Markdown report
python3 - <<PY
import json, time
from pathlib import Path
root = Path("${ROOT}")
verify = json.loads((root / "${VERIFY_OUT}").read_text()) if (root / "${VERIFY_OUT}").is_file() else {}
spawned = json.loads((root / "${SPAWNED_OUT}").read_text()) if (root / "${SPAWNED_OUT}").is_file() else {}
vis_p = root / "data/metrics/fkie_agx_warehouse_mode_b/ycb_seed${SEED}_visibility.json"
vis = json.loads(vis_p.read_text()) if vis_p.is_file() else {}
ok = bool(verify.get("ok"))
lines = [
  "# Mode B YCB scene load",
  "",
  "Date: " + time.strftime("%Y-%m-%d %H:%M:%S"),
  "Layout: \`${LAYOUT}\` (not regenerated)",
  "only_missing: ${ONLY_MISSING}",
  "",
  "## Verify",
  "- ok: **%s**" % ok,
  "- n_exist: %s / %s" % (verify.get("n_exist"), verify.get("n_expected_total")),
  "- n_targets_exist: %s / %s" % (verify.get("n_targets_exist"), verify.get("n_expected_targets")),
  "- n_pose_ok: %s" % verify.get("n_pose_ok"),
  "",
  "## Spawn",
  "- n_ok: %s / %s" % (spawned.get("n_ok"), spawned.get("n_requested")),
  "- n_fail: %s" % spawned.get("n_fail"),
  "",
  "## Visibility diag",
  "\`\`\`json",
  json.dumps(vis, indent=2)[:4000],
  "\`\`\`",
  "",
  "## Shots",
  "- \`reports/mode_b_ycb_shots/\`",
  "",
  "LOAD_YCB_SCENE_OK" if ok else "LOAD_YCB_SCENE_FAIL",
]
(root / "${REPORT}").write_text("\n".join(lines) + "\n")
print("Wrote", root / "${REPORT}")
print("LOAD_YCB_SCENE_OK" if ok else "LOAD_YCB_SCENE_FAIL")
raise SystemExit(0 if ok else 1)
PY
