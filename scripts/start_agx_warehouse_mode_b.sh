#!/usr/bin/env bash
# Mode B warehouse bringup: dynamic AGX + p3d TF + REMANI bridges.
# Optional YCB: LOAD_YCB_SCENE=1 loads validated seed42 layout (never regenerates).
# Pure motion (B4): LOAD_YCB_SCENE=0 (default when unset → 0 for regression safety;
#   set LOAD_YCB_SCENE=1 for perception/S5-B scene).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

export AGX_MODE=dynamic
export SPAWN_PROFILE="${SPAWN_PROFILE:-aisle_safe_ycb}"
export SKIP_NBV_ACTION="${SKIP_NBV_ACTION:-1}"
export NBV_OCTOMAP_SETTLE_S="${NBV_OCTOMAP_SETTLE_S:-12}"
export MODE_B_REMANI_BRIDGES="${MODE_B_REMANI_BRIDGES:-1}"
export FORCE_RESTART="${FORCE_RESTART:-1}"
export LOAD_YCB_SCENE="${LOAD_YCB_SCENE:-0}"
export SEED="${SEED:-42}"
export SCENE_KIND="${SCENE_KIND:-ycb}"
export ONLY_MISSING="${ONLY_MISSING:-1}"

echo "[mode-b] AGX_MODE=dynamic SPAWN_PROFILE=${SPAWN_PROFILE} SKIP_NBV_ACTION=${SKIP_NBV_ACTION} LOAD_YCB_SCENE=${LOAD_YCB_SCENE}"

# Prefer running inside nbv_fkie (same as Mode A demos).
if [ -f /.dockerenv ] || [ -n "${INSIDE_NBV_FKIE:-}" ]; then
  if [ "${FORCE_RESTART}" = "1" ]; then
    STOP_SIM_KEEP_PID=$$ bash /workspace/scripts/fkie_stop_sim.sh || true
    sleep 2
  fi
  # Bringup in background so we can load YCB after spawn service is up,
  # without blocking forever on the warehouse script's keep-alive loop.
  bash /workspace/scripts/fkie_agx_warehouse_nbv.sh \
    > /workspace/logs/fkie_agx_warehouse_mode_b/bringup_stdout.log 2>&1 &
  BRINGUP_PID=$!
  echo "[mode-b] warehouse bringup pid=${BRINGUP_PID}"

  echo "[mode-b] waiting for /gazebo/spawn_sdf_model..."
  for i in $(seq 1 120); do
    if timeout 2 rosservice list 2>/dev/null | grep -q /gazebo/spawn_sdf_model; then
      echo "[mode-b] gazebo spawn ready (${i}s)"
      break
    fi
    sleep 2
    if [ "$i" -eq 120 ]; then
      echo "[mode-b] FAIL: gazebo not ready"
      exit 3
    fi
  done

  # Wait until AGX model is present (Mode B spawn finished)
  for i in $(seq 1 60); do
    if timeout 3 rosservice call /gazebo/get_model_state "{model_name: agx_ranger_cr10, relative_entity_name: world}" 2>/dev/null | grep -q 'success: True'; then
      echo "[mode-b] AGX present (${i}s)"
      break
    fi
    sleep 2
  done

  if [ "${LOAD_YCB_SCENE}" = "1" ]; then
    export INSIDE_NBV_FKIE=1
    ONLY_MISSING="${ONLY_MISSING}" SEED="${SEED}" SCENE_KIND="${SCENE_KIND}" \
      bash /workspace/scripts/load_ycb_scene_into_gazebo.sh
  else
    echo "[mode-b] LOAD_YCB_SCENE=0 — pure motion / no YCB (B4 regression mode)"
  fi

  echo "[mode-b] ready. bringup keeps running (pid=${BRINGUP_PID})."
  wait "${BRINGUP_PID}" || true
  exit 0
fi

docker exec -d nbv_fkie bash -lc "
  export DISPLAY=\${DISPLAY:-:1} QT_X11_NO_MITSHM=1
  export AGX_MODE=dynamic SPAWN_PROFILE=${SPAWN_PROFILE}
  export SKIP_NBV_ACTION=${SKIP_NBV_ACTION} NBV_OCTOMAP_SETTLE_S=${NBV_OCTOMAP_SETTLE_S}
  export MODE_B_REMANI_BRIDGES=${MODE_B_REMANI_BRIDGES} FORCE_RESTART=${FORCE_RESTART}
  export LOAD_YCB_SCENE=${LOAD_YCB_SCENE} SEED=${SEED} SCENE_KIND=${SCENE_KIND} ONLY_MISSING=${ONLY_MISSING}
  export INSIDE_NBV_FKIE=1
  bash /workspace/scripts/start_agx_warehouse_mode_b.sh
"
echo "[mode-b] launched in nbv_fkie (detached). Logs: logs/fkie_agx_warehouse_mode_b/"
if [ "${LOAD_YCB_SCENE}" = "1" ]; then
  echo "[mode-b] YCB load enabled (seed=${SEED}). Check reports/mode_b_ycb_scene.md after ~90s"
else
  echo "[mode-b] wait ~60s then: docker exec nbv_fkie bash -lc 'python3 /workspace/scripts/check_agx_dynamic_stability.py'"
fi
