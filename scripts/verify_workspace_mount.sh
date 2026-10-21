#!/usr/bin/env bash
# Verify project root <-> /workspace bind mount for a running container.
set -euo pipefail
CONTAINER="${1:?container name}"
MARKER="/workspace/.mount_verify_$$"
HOST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST_MARKER="${HOST_ROOT}/.mount_verify_$$"

echo "probe-$(date +%s)" > "${HOST_MARKER}"
docker exec "${CONTAINER}" test -f "${MARKER##*/}" 2>/dev/null || docker exec "${CONTAINER}" test -f "/workspace/$(basename "${HOST_MARKER}")"
docker exec "${CONTAINER}" bash -lc "test -f /workspace/$(basename "${HOST_MARKER}") && cat /workspace/$(basename "${HOST_MARKER}")"
docker exec "${CONTAINER}" bash -lc "echo container-write-\$(date +%s) > /workspace/logs/mount_write_test.txt"
test -f "${HOST_ROOT}/logs/mount_write_test.txt"
cat "${HOST_ROOT}/logs/mount_write_test.txt"
rm -f "${HOST_MARKER}"
echo "MOUNT_VERIFY_OK container=${CONTAINER}"
