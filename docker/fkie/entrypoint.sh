#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
if [ -f /workspace/ws_fkie/devel/setup.bash ]; then
  # shellcheck disable=SC1091
  source /workspace/ws_fkie/devel/setup.bash
fi
exec "$@"
