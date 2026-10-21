#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
# shellcheck disable=SC1091
source "${ROOT}/scripts/setup_x11.sh"
"${ROOT}/scripts/setup_fkie_ws.sh"

docker compose -f docker-compose.fkie.yml build

docker compose -f docker-compose.fkie.yml run --rm --entrypoint bash fkie -lc '
  set -e
  source /opt/ros/noetic/setup.bash
  cd /workspace/ws_fkie
  catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin build fkie_ddynamic_reconfigure fkie_measurement_msgs fkie_nbv_planner --no-status
  source devel/setup.bash
  echo "FKIE build OK"
  rospack find fkie_nbv_planner
'
