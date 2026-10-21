#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
# shellcheck disable=SC1091
source "${ROOT}/scripts/setup_x11.sh"

docker compose -f docker-compose.ssmi.yml build

docker compose -f docker-compose.ssmi.yml run --rm --entrypoint bash ssmi -lc '
  set -e
  source /opt/ros/noetic/setup.bash
  WS=/workspace/third_party/SSMI-Example/ssmi_ws
  cd "$WS"
  if [ -d src/ros_gz ]; then
    rosdep install -r --from-paths src/ros_gz -i -y --rosdistro noetic || true
  fi
  catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin build --no-status || catkin build --no-status
  source devel/setup.bash
  echo "SSMI build OK"
  rospack find semantic_octomap || true
  rospack find semantic_info_gathering || true
'
