#!/usr/bin/env bash
# Run NBV-local Grounded-SAM perception (no DovSG path / env).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export NBV_ROOT="${NBV_ROOT:-$ROOT}"
export NBV_PERCEPTION_ROOT="${NBV_PERCEPTION_ROOT:-$ROOT}"
export PYTHONPATH="${NBV_ROOT}${PYTHONPATH:+:$PYTHONPATH}"
export TRANSFORMERS_OFFLINE="${TRANSFORMERS_OFFLINE:-1}"
export HF_HOME="${HF_HOME:-$NBV_ROOT/checkpoints}"
PY="${NBV_PERCEPTION_PYTHON:-$NBV_ROOT/.conda_envs/nbv_perception/bin/python}"
if [[ ! -x "$PY" ]]; then
  echo "[perception] missing python: $PY" >&2
  echo "Create env per configs/perception/README.md" >&2
  exit 2
fi
# Refuse if DovSG leaked into PYTHONPATH
if [[ "${PYTHONPATH}" == *"/DovSG"* ]] || [[ "${PYTHONPATH}" == *"/locomani/DovSG"* ]]; then
  echo "[perception] PYTHONPATH must not include DovSG: $PYTHONPATH" >&2
  exit 3
fi
exec "$PY" "$NBV_ROOT/scripts/perceive_rgbd_objects.py" --backend grounded_sam "$@"
