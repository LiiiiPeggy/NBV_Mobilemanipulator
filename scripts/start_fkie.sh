#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
# shellcheck disable=SC1091
source "${ROOT}/scripts/setup_x11.sh"
docker compose -f docker-compose.fkie.yml up -d
echo "nbv_fkie started. Attach: docker exec -it nbv_fkie bash"
