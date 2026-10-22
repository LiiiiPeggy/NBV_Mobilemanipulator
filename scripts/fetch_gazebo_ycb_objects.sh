#!/usr/bin/env bash
# Download a small Gazebo-compatible object set for multi-object experiments.
# Assets stay under third_party/gazebo_objects/ (gitignore). Does not commit binaries.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/third_party/gazebo_objects"
mkdir -p "${DEST}"
MANIFEST="${DEST}/SOURCES.md"

cat > "${MANIFEST}" <<'EOF'
# Object asset sources

| Source | URL | License | Notes |
|--------|-----|---------|-------|
| Gazebo YCB Objects | https://github.com/CentralLabFacilities/gazebo_ycb | check upstream | Prefer Classic SDF |
| 3DGEMS | https://data.nvision2.eecs.yorku.ca/3DGEMS/ | check upstream | Often incomplete inertia — validate per model |
| VP-SOM | https://github.com/TINY-KE/VP-SOM | check upstream | Method reference; models optional |

First-wave subset (8–12): mug, bottle, cracker box, book, clock, cone — verify each has visual+collision+inertia before physics use.
EOF

if [ "${1:-}" = "--clone-ycb" ]; then
  if [ ! -d "${DEST}/gazebo_ycb/.git" ]; then
    git clone --depth 1 https://github.com/CentralLabFacilities/gazebo_ycb.git "${DEST}/gazebo_ycb"
  fi
  echo "Cloned to ${DEST}/gazebo_ycb"
  ls "${DEST}/gazebo_ycb" | head
else
  echo "Wrote ${MANIFEST}"
  echo "Run: $0 --clone-ycb   # to fetch models (large)"
fi
