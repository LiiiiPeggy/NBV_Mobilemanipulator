#!/usr/bin/env bash
# One-shot copy of Grounding DINO + SAM2 source/weights from DovSG into NBV.
# Does NOT create symlinks. Does NOT modify DovSG. Does NOT download weights.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${DOVSG_SOURCE:-/home/ubuntu/locomani/DovSG}"
DEST_VENDOR="${ROOT}/perception/vendor"
DEST_CKPT="${ROOT}/checkpoints"
MANIFEST="${DEST_CKPT}/CHECKSUMS.sha256"
NOTICE="${ROOT}/perception/NOTICE"

test -d "${SRC}/third_party/GroundingDINO"
test -d "${SRC}/third_party/segment-anything-2"
test -f "${SRC}/checkpoints/GroundingDINO/groundingdino_swint_ogc.pth"
test -f "${SRC}/checkpoints/segment-anything-2/sam2_hiera_large.pt"

mkdir -p "${DEST_VENDOR}" "${DEST_CKPT}/grounding_dino" "${DEST_CKPT}/sam2"

echo "[vendor] copying GroundingDINO tree..."
rm -rf "${DEST_VENDOR}/GroundingDINO"
cp -a "${SRC}/third_party/GroundingDINO" "${DEST_VENDOR}/GroundingDINO"

echo "[vendor] copying segment-anything-2 tree..."
rm -rf "${DEST_VENDOR}/segment-anything-2"
cp -a "${SRC}/third_party/segment-anything-2" "${DEST_VENDOR}/segment-anything-2"

echo "[vendor] copying checkpoints (binary)..."
cp -f "${SRC}/checkpoints/GroundingDINO/groundingdino_swint_ogc.pth" \
  "${DEST_CKPT}/grounding_dino/groundingdino_swint_ogc.pth"
cp -f "${SRC}/checkpoints/GroundingDINO/GroundingDINO_SwinT_OGC.py" \
  "${DEST_CKPT}/grounding_dino/GroundingDINO_SwinT_OGC.py"
cp -f "${SRC}/checkpoints/segment-anything-2/sam2_hiera_large.pt" \
  "${DEST_CKPT}/sam2/sam2_hiera_large.pt"
# bert for GDINO text encoder
if [ -d "${SRC}/checkpoints/bert-base-uncased" ]; then
  rm -rf "${DEST_CKPT}/bert-base-uncased"
  cp -a "${SRC}/checkpoints/bert-base-uncased" "${DEST_CKPT}/bert-base-uncased"
fi

# Copy LICENSE files if present
mkdir -p "${ROOT}/perception/licenses"
cp -f "${DEST_VENDOR}/GroundingDINO/LICENSE" "${ROOT}/perception/licenses/GroundingDINO.LICENSE" 2>/dev/null || true
cp -f "${DEST_VENDOR}/segment-anything-2/LICENSE"* "${ROOT}/perception/licenses/" 2>/dev/null || true

DOVSG_REV="$(git -C "${SRC}" rev-parse HEAD 2>/dev/null || echo unknown)"
cat > "${NOTICE}" <<EOF
Vendored perception sources for NBV_Mobilemanipulator.
Source project: ${SRC}
Source git rev: ${DOVSG_REV}
Copied (not linked): GroundingDINO, segment-anything-2, checkpoints.
Runtime MUST NOT import or mount DovSG. See checkpoints/CHECKSUMS.sha256.
EOF

echo "[vendor] writing SHA256 manifest..."
: > "${MANIFEST}"
(
  cd "${ROOT}"
  find checkpoints/grounding_dino checkpoints/sam2 checkpoints/bert-base-uncased \
    -type f \( -name '*.pth' -o -name '*.pt' -o -name '*.py' -o -name '*.yaml' -o -name '*.json' -o -name '*.safetensors' -o -name 'tokenizer*' -o -name 'vocab*' -o -name 'config*' \) \
    2>/dev/null | sort | while read -r f; do
      sha256sum "$f"
    done
) >> "${MANIFEST}"

# Also hash key source entrypoints
(
  cd "${ROOT}"
  sha256sum perception/vendor/GroundingDINO/groundingdino/util/inference.py \
    perception/vendor/segment-anything-2/sam2/build_sam.py \
    2>/dev/null || true
) >> "${MANIFEST}"

echo "[vendor] OK → ${DEST_VENDOR} + ${DEST_CKPT}"
echo "VENDOR_DOVSG_PERCEPTION_OK"
