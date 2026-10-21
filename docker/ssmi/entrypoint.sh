#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
if [ -f /workspace/third_party/SSMI-Example/ssmi_ws/devel/setup.bash ]; then
  # shellcheck disable=SC1091
  source /workspace/third_party/SSMI-Example/ssmi_ws/devel/setup.bash
fi
exec "$@"
