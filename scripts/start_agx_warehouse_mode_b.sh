#!/usr/bin/env bash
# Mode B warehouse bringup: dynamic AGX + ros_control + p3d TF + REMANI bridges.
# Does NOT overwrite Mode A static URDF. Keeps SKIP_NBV_ACTION=1 by default for physics tests.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

export AGX_MODE=dynamic
export SPAWN_PROFILE="${SPAWN_PROFILE:-aisle_safe_ycb}"
export SKIP_NBV_ACTION="${SKIP_NBV_ACTION:-1}"
export NBV_OCTOMAP_SETTLE_S="${NBV_OCTOMAP_SETTLE_S:-12}"
export MODE_B_REMANI_BRIDGES="${MODE_B_REMANI_BRIDGES:-1}"
export FORCE_RESTART="${FORCE_RESTART:-1}"

echo "[mode-b] AGX_MODE=dynamic SPAWN_PROFILE=${SPAWN_PROFILE} SKIP_NBV_ACTION=${SKIP_NBV_ACTION}"

# Prefer running inside nbv_fkie (same as Mode A demos).
if [ -f /.dockerenv ] || [ -n "${INSIDE_NBV_FKIE:-}" ]; then
  if [ "${FORCE_RESTART}" = "1" ]; then
    STOP_SIM_KEEP_PID=$$ bash /workspace/scripts/fkie_stop_sim.sh || true
    sleep 2
  fi
  exec bash /workspace/scripts/fkie_agx_warehouse_nbv.sh
fi

docker exec -d nbv_fkie bash -lc "
  export DISPLAY=\${DISPLAY:-:1} QT_X11_NO_MITSHM=1
  export AGX_MODE=dynamic SPAWN_PROFILE=${SPAWN_PROFILE}
  export SKIP_NBV_ACTION=${SKIP_NBV_ACTION} NBV_OCTOMAP_SETTLE_S=${NBV_OCTOMAP_SETTLE_S}
  export MODE_B_REMANI_BRIDGES=${MODE_B_REMANI_BRIDGES} FORCE_RESTART=${FORCE_RESTART}
  export INSIDE_NBV_FKIE=1
  bash /workspace/scripts/start_agx_warehouse_mode_b.sh
" 
echo "[mode-b] launched in nbv_fkie (detached). Logs: logs/fkie_agx_warehouse_mode_b/"
echo "[mode-b] wait ~60s then: docker exec nbv_fkie bash -lc 'python3 /workspace/scripts/check_agx_dynamic_stability.py'"
