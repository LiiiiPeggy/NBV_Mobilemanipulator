#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
docker compose -f docker-compose.ssmi.yml down
echo "nbv_ssmi stopped."
