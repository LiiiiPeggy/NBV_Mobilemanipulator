#!/usr/bin/env bash
# Fetch Gazebo Classic object models for indoor multi-object experiments.
# Large assets stay under third_party/gazebo_objects/ (gitignored).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "${ROOT}/scripts/fetch_gazebo_ycb_objects.sh" "$@"
