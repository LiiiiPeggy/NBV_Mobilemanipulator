#!/usr/bin/env bash
# Run NBV-local Grounded-SAM perception (no DovSG path / env).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export NBV_ROOT="${NBV_ROOT:-$ROOT}"
export NBV_PERCEPTION_ROOT="${NBV_PERCEPTION_ROOT:-$ROOT}"
export TRANSFORMERS_OFFLINE="${TRANSFORMERS_OFFLINE:-1}"
export HF_HOME="${HF_HOME:-$NBV_ROOT/checkpoints}"
PY="${NBV_PERCEPTION_PYTHON:-$NBV_ROOT/.conda_envs/nbv_perception/bin/python}"
if [[ ! -x "$PY" ]]; then
  echo "[perception] missing python: $PY" >&2
  echo "Create env per configs/perception/README.md" >&2
  exit 2
fi
CONDA_SITE="$NBV_ROOT/.conda_envs/nbv_perception/lib/python3.9/site-packages"
VENDOR_GD="$NBV_ROOT/perception/vendor/GroundingDINO"
VENDOR_SAM="$NBV_ROOT/perception/vendor/segment-anything-2"
ROS_PY="/opt/ros/noetic/lib/python3/dist-packages"
# conda first (numpy/torch), then vendored GD/SAM2, then ROS rospy — never /usr/lib first
export PYTHONPATH="${NBV_ROOT}:${CONDA_SITE}:${VENDOR_GD}:${VENDOR_SAM}:${ROS_PY}${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$(dirname "$PY"):${PATH}"
# Refuse if DovSG leaked into PYTHONPATH
if [[ "${PYTHONPATH}" == *"/DovSG"* ]] || [[ "${PYTHONPATH}" == *"/locomani/DovSG"* ]]; then
  echo "[perception] PYTHONPATH must not include DovSG: $PYTHONPATH" >&2
  exit 3
fi
exec "$PY" "$NBV_ROOT/scripts/perceive_rgbd_objects.py" --backend grounded_sam "$@"
