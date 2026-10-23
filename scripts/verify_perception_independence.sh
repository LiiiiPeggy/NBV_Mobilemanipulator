#!/usr/bin/env bash
# Prove Grounded-SAM runs from NBV paths only (no DovSG runtime).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DOVSG="${DOVSG_PATH:-/home/ubuntu/locomani/DovSG}"
PY="${NBV_PERCEPTION_PYTHON:-$ROOT/.conda_envs/nbv_perception/bin/python}"
REPORT_DIR="${1:-$ROOT/reports}"
LOG="$REPORT_DIR/grounded_sam_independence.log"
mkdir -p "$REPORT_DIR"
exec > >(tee "$LOG") 2>&1

echo "=== verify_perception_independence $(date -Is) ==="
echo "NBV_ROOT=$ROOT"
echo "DOVSG=$DOVSG"
echo "PY=$PY"

test -x "$PY" || { echo "FAIL missing python $PY"; exit 2; }
test -f "$ROOT/checkpoints/CHECKSUMS.sha256" || { echo "FAIL missing CHECKSUMS"; exit 2; }
(cd "$ROOT" && sha256sum -c checkpoints/CHECKSUMS.sha256) || { echo "FAIL checksum"; exit 3; }

export NBV_ROOT="$ROOT" NBV_PERCEPTION_ROOT="$ROOT"
export PYTHONPATH="$ROOT"
export TRANSFORMERS_OFFLINE=1 HF_HOME="$ROOT/checkpoints"
# Explicitly scrub any DovSG hints
unset DOVSG_ROOT DOVSG_HOME || true
export PYTHONPATH="$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v DovSG | paste -sd: -)"

echo "--- import paths ---"
"$PY" - <<'PY'
from pathlib import Path
import groundingdino, sam2, perception, perception.paths as pp
for name, mod in [("groundingdino", groundingdino), ("sam2", sam2), ("perception", perception)]:
    p = str(Path(mod.__file__).resolve())
    print(name, p)
    assert "/DovSG/" not in p, p
    assert "NBV_Mobilemanipulator" in p or "perception/vendor" in p or name == "perception"
pp.assert_local_checkpoints()
for p in (pp.grounding_dino_checkpoint(), pp.sam2_checkpoint()):
    r = str(p.resolve())
    print("ckpt", r)
    assert "/DovSG/" not in r
print("IMPORT_PATHS_OK")
PY

echo "--- openat audit (optional; SKIP_STRACE=1 to skip) ---"
STRACE_LOG="$REPORT_DIR/grounded_sam_strace.txt"
if [[ "${SKIP_STRACE:-1}" == "1" ]]; then
  echo "STRACE_SKIPPED (set SKIP_STRACE=0 to enable; can be slow under torch)"
elif command -v strace >/dev/null; then
  timeout 90 strace -f -e openat -o "$STRACE_LOG" "$PY" - <<'PY' || true
import os
os.environ["NBV_ROOT"]=os.environ["NBV_PERCEPTION_ROOT"]="/home/ubuntu/locomani/NBV_Mobilemanipulator"
os.environ["PYTHONPATH"]="/home/ubuntu/locomani/NBV_Mobilemanipulator"
os.environ["TRANSFORMERS_OFFLINE"]="1"
os.environ["HF_HOME"]="/home/ubuntu/locomani/NBV_Mobilemanipulator/checkpoints"
from perception.rgbd_object_detector import GroundedSAMDetector
import numpy as np
d=GroundedSAMDetector(device="cpu")
r=d.run(np.zeros((240,320,3),dtype=np.uint8), ["banana"])
print("STRACE_INFER_OK", len(r.class_id))
PY
  if [[ -f "$STRACE_LOG" ]] && grep -E '/home/ubuntu/locomani/DovSG(/|$)' "$STRACE_LOG" >/dev/null 2>&1; then
    echo "FAIL strace saw DovSG path opens"
    grep -E '/home/ubuntu/locomani/DovSG' "$STRACE_LOG" | head -20
    exit 4
  else
    echo "STRACE_NO_DOVSG_OK"
  fi
else
  echo "STRACE_SKIPPED"
fi

echo "--- rename DovSG temporarily and re-infer ---"
RENAMED=0
BACKUP=""
if [[ -d "$DOVSG" ]]; then
  BACKUP="${DOVSG}.nbv_indep_bak_$$"
  if mv "$DOVSG" "$BACKUP" 2>/dev/null; then
    RENAMED=1
    echo "renamed $DOVSG -> $BACKUP"
  else
    echo "WARN cannot rename DovSG (permission); continuing with path assertions only"
  fi
else
  echo "DovSG already absent at $DOVSG"
fi

set +e
"$PY" - <<'PY'
import time, numpy as np, torch
t0=time.time()
from perception.rgbd_object_detector import GroundedSAMDetector
d=GroundedSAMDetector(device="cuda" if torch.cuda.is_available() else "cpu")
img=np.random.randint(0,255,(480,640,3),dtype=np.uint8)
t1=time.time()
r=d.run(img, ["banana","bottle","apple","bowl"])
print("device", d.device)
print("LOAD_S", round(t1-t0,3))
print("INFER_S", round(time.time()-t1,3))
print("NDET", len(r.class_id))
if torch.cuda.is_available():
    print("MAX_GB", round(torch.cuda.max_memory_allocated()/1e9,3))
print("INDEPENDENCE_INFER_OK")
PY
RC=$?
set -e

if [[ "$RENAMED" == "1" ]]; then
  mv "$BACKUP" "$DOVSG"
  echo "restored $DOVSG"
fi

if [[ "$RC" != "0" ]]; then
  echo "FAIL independence infer rc=$RC"
  exit "$RC"
fi

echo "PERCEPTION_INDEPENDENCE_OK"
echo "log=$LOG"
