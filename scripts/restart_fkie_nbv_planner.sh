#!/usr/bin/env bash
# Restart only FKIE nbvplanner_node (clear visited_grid); keep Gazebo/octomap/REMANI.
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/ws_fkie/devel/setup.bash 2>/dev/null || true

LOGDIR="${LOGDIR:-/workspace/logs/fkie_agx_warehouse_mode_b}"
mkdir -p "$LOGDIR"
PLANNER_YAML="${PLANNER_YAML:-/workspace/configs/fkie/agx_adapt/config/nbv_planner_agx_warehouse_aisle_safe.yaml}"

# Optional height floor for usable camera samples (default keep yaml; override via env)
ARM_HEIGHT_MIN="${ARM_HEIGHT_MIN:-}"
ARM_HEIGHT_MAX="${ARM_HEIGHT_MAX:-}"

# Kill all planner copies (host may accumulate stale ones across restarts)
ps -eo pid,cmd | awk '/nbvplanner_node/ && !/awk/ {print $1}' | xargs -r kill -9 2>/dev/null || true
sleep 2
ps -eo pid,cmd | awk '/nbvplanner_node/ && !/awk/ {print $1}' | xargs -r kill -9 2>/dev/null || true
sleep 1

rosparam load "${PLANNER_YAML}" /robot/fkie_nbv_planner
if [ -n "$ARM_HEIGHT_MIN" ]; then
  rosparam set /robot/fkie_nbv_planner/arm_height_min "$ARM_HEIGHT_MIN"
fi
if [ -n "$ARM_HEIGHT_MAX" ]; then
  rosparam set /robot/fkie_nbv_planner/arm_height_max "$ARM_HEIGHT_MAX"
fi
rosparam set /robot/fkie_nbv_planner/gain_r_max 5.0

# Critical remap: planner expects relative octomap_full under /robot
nohup rosrun fkie_nbv_planner nbvplanner_node __ns:=robot __name:=fkie_nbv_planner \
  octomap:=octomap_full \
  >"${LOGDIR}/nbvplanner_restart.log" 2>&1 &
echo "nbvplanner_pid=$!"
sleep 2
python3 - <<'PY'
import rospy, actionlib
from fkie_nbv_planner.msg import NbvPlannerAction
rospy.init_node("wait_nbv_restart", anonymous=True)
c = actionlib.SimpleActionClient("/robot/nbv_rrt", NbvPlannerAction)
ok = c.wait_for_server(rospy.Duration(45))
print("arm_height_min", rospy.get_param("/robot/fkie_nbv_planner/arm_height_min", None))
print("NBV_PLANNER_RESTARTED" if ok else "NBV_PLANNER_RESTART_FAIL")
raise SystemExit(0 if ok else 1)
PY
