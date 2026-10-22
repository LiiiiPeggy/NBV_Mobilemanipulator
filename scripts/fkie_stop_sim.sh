#!/usr/bin/env bash
# Stop FKIE NBV simulation processes inside the current environment (nbv_fkie container).
# Safe scope: this container is dedicated to FKIE; does not docker stop other users' containers.
# Usage: bash scripts/fkie_stop_sim.sh
set -euo pipefail

echo "[fkie-stop] stopping FKIE demo / Gazebo / RViz in this environment..."

# Demo entry scripts — never kill the caller (STOP_SIM_KEEP_PID) or this stop script
KEEP_PID="${STOP_SIM_KEEP_PID:-}"
kill_other_matching() {
  local pat="$1"
  local p
  for p in $(pgrep -f "$pat" 2>/dev/null || true); do
    if [ "$p" = "$$" ] || [ "$p" = "$PPID" ]; then continue; fi
    if [ -n "$KEEP_PID" ] && [ "$p" = "$KEEP_PID" ]; then continue; fi
    kill -TERM "$p" 2>/dev/null || true
  done
  sleep 0.5
  for p in $(pgrep -f "$pat" 2>/dev/null || true); do
    if [ "$p" = "$$" ] || [ "$p" = "$PPID" ]; then continue; fi
    if [ -n "$KEEP_PID" ] && [ "$p" = "$KEEP_PID" ]; then continue; fi
    kill -9 "$p" 2>/dev/null || true
  done
}

kill_other_matching '/workspace/scripts/fkie_agx_warehouse_nbv.sh'
kill_other_matching '/workspace/scripts/fkie_warehouse_nbv_demo.sh'
kill_other_matching '/workspace/scripts/fkie_effective_nbv_test.sh'
kill_other_matching '/workspace/scripts/fkie_viz_demo.sh'

# ROS nodes started by demos (best-effort by cmdline)
pkill -f 'nbvplanner_node' 2>/dev/null || true
pkill -f 'octomap_server_node' 2>/dev/null || true
pkill -f 'camera_pose_tf.py' 2>/dev/null || true
pkill -f 'relay_points2' 2>/dev/null || true
pkill -f 'agx_world_base_tf' 2>/dev/null || true
pkill -f 'agx_ranger_footprint' 2>/dev/null || true
pkill -f 'agx_js_fallback' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true

# Gazebo Classic + RViz (FKIE stack only lives in this container)
killall -TERM gzserver gzclient rviz 2>/dev/null || true
pkill -TERM -x gzserver 2>/dev/null || true
pkill -TERM -x gzclient 2>/dev/null || true
sleep 2
killall -9 gzserver gzclient rviz 2>/dev/null || true
pkill -9 -x gzserver 2>/dev/null || true
pkill -9 -x gzclient 2>/dev/null || true
# Force by PID list (handles stubborn / defunct parents)
for p in $(pgrep -x gzserver 2>/dev/null; pgrep -x gzclient 2>/dev/null; pgrep -x rviz 2>/dev/null); do
  kill -9 "$p" 2>/dev/null || true
done

# Orphan rosrun gazebo wrappers
pkill -9 -f 'rosrun gazebo_ros gazebo' 2>/dev/null || true
pkill -9 -f '/opt/ros/noetic/lib/gazebo_ros/gazebo' 2>/dev/null || true
pkill -9 -f 'gzserver --verbose' 2>/dev/null || true

sleep 1
left=$(pgrep -a gzserver 2>/dev/null || true)
left2=$(pgrep -a gzclient 2>/dev/null || true)
# Defunct (zombie) entries may remain until parent exits; report but treat as cleaned if only Z
alive_server=$(ps -eo pid,stat,cmd | awk '/[g]zserver/ && $2 !~ /Z/ {print}' || true)
alive_client=$(ps -eo pid,stat,cmd | awk '/[g]zclient/ && $2 !~ /Z/ {print}' || true)
if [ -n "${alive_server}${alive_client}" ]; then
  echo "[fkie-stop] WARNING still running (non-zombie):"
  echo "${alive_server}"
  echo "${alive_client}"
  exit 1
fi
if [ -n "${left}${left2}" ]; then
  echo "[fkie-stop] note: zombie gz leftovers only (need container recreate to reap)"
  echo "${left}"
  echo "${left2}"
fi
echo "[fkie-stop] OK: no live gzserver/gzclient"
exit 0
