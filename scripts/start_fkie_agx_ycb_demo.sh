#!/usr/bin/env bash
# One-click: AGX aisle_safe warehouse + real YCB multi-object scene (Mode A).
# Does NOT hardcode objects into the FKIE world; spawns after Gazebo is up.
# Default SCENE_KIND=ycb (not primitive_debug).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SCENE_KIND="${SCENE_KIND:-ycb}"
SEED="${SEED:-42}"
SPAWN_PROFILE="${SPAWN_PROFILE:-aisle_safe_ycb}"
AGX_MODE="${AGX_MODE:-static}"
FORCE_RESTART="${FORCE_RESTART:-0}"
CONTAINER="${FKIE_CONTAINER:-nbv_fkie}"
LAYOUT="${LAYOUT:-data/scenes/indoor_objects/${SCENE_KIND}/layouts/seed${SEED}.json}"
GT_OUT="${GT_OUT:-data/scenes/indoor_objects/${SCENE_KIND}/gt/seed${SEED}_gt.json}"
VERIFY_OUT="${VERIFY_OUT:-data/metrics/indoor_objects/${SCENE_KIND}/seed${SEED}_verify.json}"
SPAWNED_OUT="${SPAWNED_OUT:-data/scenes/indoor_objects/${SCENE_KIND}/layouts/seed${SEED}.spawned.json}"
SHOTDIR="${SHOTDIR:-reports/ycb_gazebo_shots}"
LOGDIR="${LOGDIR:-logs/fkie_agx_ycb_demo}"
mkdir -p "${ROOT}/${SHOTDIR}" "${ROOT}/${LOGDIR}" "$(dirname "${ROOT}/${GT_OUT}")" "$(dirname "${ROOT}/${VERIFY_OUT}")"

echo "[ycb-demo] SCENE_KIND=${SCENE_KIND} SEED=${SEED} profile=${SPAWN_PROFILE} mode=${AGX_MODE}"
echo "[ycb-demo] layout=${LAYOUT}"

if [[ "${SCENE_KIND}" != "ycb" ]]; then
  echo "[ycb-demo] WARN: this entry is intended for SCENE_KIND=ycb (got ${SCENE_KIND})"
fi

# Ensure layout exists / refresh FOV-aware YCB layout
if [[ "${SCENE_KIND}" = "ycb" ]]; then
  python3 "${ROOT}/scripts/generate_object_scene.py" --seed "${SEED}" --n-objects 12 --out "${ROOT}/${LAYOUT}"
fi
test -f "${ROOT}/${LAYOUT}" || { echo "missing layout ${LAYOUT}"; exit 2; }

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[ycb-demo] starting compose service fkie..."
  docker compose -f "${ROOT}/docker-compose.fkie.yml" up -d fkie
  sleep 3
fi

run_in() {
  docker exec "${CONTAINER}" bash -lc "source /opt/ros/noetic/setup.bash; \
    source /workspace/ws_fkie/devel/setup.bash; \
    export NBV_ROOT=/workspace; \
    export GAZEBO_MODEL_PATH=/workspace/third_party/gazebo_objects/gazebo_ycb/models:/workspace/configs/fkie/agx_adapt/models:/workspace/third_party/gazebo_models:\${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}; \
    $1"
}

gazebo_up() {
  run_in 'rosservice list 2>/dev/null | grep -q /gazebo/spawn_sdf_model'
}

echo "[ycb-demo] checking Gazebo..."
NEED_BRINGUP=1
if [[ "${FORCE_RESTART}" != "1" ]] && gazebo_up; then
  # Reuse only if AGX already uses ycb observe profile (heuristic: skip if FORCE_RESTART)
  echo "[ycb-demo] Gazebo up; set FORCE_RESTART=1 to rebake arm to observe-table pose"
  NEED_BRINGUP=0
fi
if [[ "${NEED_BRINGUP}" = "1" ]] || [[ "${FORCE_RESTART}" = "1" ]]; then
  echo "[ycb-demo] bringing up AGX warehouse profile=${SPAWN_PROFILE} (SKIP_NBV_ACTION=1)..."
  docker exec -d "${CONTAINER}" bash -lc "source /opt/ros/noetic/setup.bash; \
    source /workspace/ws_fkie/devel/setup.bash; \
    export SPAWN_PROFILE=${SPAWN_PROFILE} AGX_MODE=${AGX_MODE} SKIP_NBV_ACTION=1 NBV_OCTOMAP_SETTLE_S=${NBV_OCTOMAP_SETTLE_S:-15}; \
    bash /workspace/scripts/fkie_agx_warehouse_nbv.sh > /workspace/${LOGDIR}/bringup.log 2>&1"
  echo -n "[ycb-demo] waiting for Gazebo"
  for i in $(seq 1 180); do
    if gazebo_up; then echo " ready (${i}s)"; break; fi
    echo -n "."
    sleep 2
    if [[ "$i" -eq 180 ]]; then
      echo
      echo "[ycb-demo] FAIL: Gazebo not ready"; tail -50 "${ROOT}/${LOGDIR}/bringup.log" || true
      exit 3
    fi
  done
fi

# Wait for D455 topics (optional but preferred)
echo "[ycb-demo] waiting for D455 rgb..."
run_in 'for i in $(seq 1 60); do timeout 2 rostopic echo -n 1 /realsense/rgb/image_raw >/dev/null 2>&1 && echo D455_OK && exit 0; sleep 1; done; echo D455_TIMEOUT; exit 0'

echo "[ycb-demo] spawning YCB layout..."
run_in "python3 /workspace/scripts/spawn_object_scene.py \
  --layout /workspace/${LAYOUT} \
  --out-json /workspace/${SPAWNED_OUT}" | tee "${ROOT}/${LOGDIR}/spawn.log"

echo "[ycb-demo] verifying scene..."
run_in "python3 /workspace/scripts/verify_gazebo_object_scene.py \
  --layout /workspace/${LAYOUT} \
  --spawned /workspace/${SPAWNED_OUT} \
  --out /workspace/${VERIFY_OUT}" | tee "${ROOT}/${LOGDIR}/verify.log"

echo "[ycb-demo] exporting GT..."
run_in "python3 /workspace/scripts/export_object_gt.py \
  --layout /workspace/${LAYOUT} \
  --spawned /workspace/${SPAWNED_OUT} \
  --out /workspace/${GT_OUT}" | tee "${ROOT}/${LOGDIR}/gt.log"

# Point Gazebo GUI camera near table/shelf (aisle_safe)
echo "[ycb-demo] aiming Gazebo GUI camera..."
run_in 'gz camera -c gzclient_camera -p \"-1.6 -5.6 2.2\" -t \"-2.6 -6.4 0.8\" 2>/dev/null || \
  timeout 5 gz topic -p /gazebo/default/pose/info -m permanent:false 2>/dev/null || true' || true

# Screenshots
if command -v import >/dev/null 2>&1; then
  DISPLAY="${DISPLAY:-:1}" import -window root "${ROOT}/${SHOTDIR}/desktop_overview_$(date +%H%M%S).png" || true
fi
run_in "python3 /workspace/scripts/capture_ycb_scene_evidence.py \
  --out-dir /workspace/${SHOTDIR} --seed ${SEED}" 2>/dev/null || \
  echo "[ycb-demo] capture helper optional; continuing"

echo "[ycb-demo] DONE"
echo "  layout=${LAYOUT}"
echo "  spawned=${SPAWNED_OUT}"
echo "  verify=${VERIFY_OUT}"
echo "  gt=${GT_OUT}"
echo "  shots=${SHOTDIR}"
echo "START_FKIE_AGX_YCB_DEMO_OK"
