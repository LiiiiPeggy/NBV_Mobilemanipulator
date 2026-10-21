#!/usr/bin/env bash
# Prepare X11 cookie for Docker without `xhost +`.
set -euo pipefail

XAUTH_FILE="${XAUTH_FILE:-/tmp/.docker.xauth.nbv}"
export DISPLAY="${DISPLAY:-:1}"

touch "${XAUTH_FILE}"
# Convert cookie family to FamilyWild so container hostname mismatches are OK.
if command -v xauth >/dev/null 2>&1; then
  xauth nlist "${DISPLAY}" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "${XAUTH_FILE}" nmerge - 2>/dev/null || true
fi
chmod 666 "${XAUTH_FILE}" 2>/dev/null || true

export XAUTH_FILE
echo "DISPLAY=${DISPLAY}"
echo "XAUTH_FILE=${XAUTH_FILE}"
