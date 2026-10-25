#!/usr/bin/env bash
# Restart Mode B kinematic bridges only (keep Gazebo / REMANI / FKIE).
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/ws_fkie/devel/setup.bash 2>/dev/null || true

ARM_POSE="${ARM_POSE:-/workspace/configs/fkie/agx_adapt/config/arm_observe_aisle_table.yaml}"
LOGDIR=/workspace/logs/fkie_agx_warehouse_mode_b
mkdir -p "$LOGDIR"

kill_pat() {
  local pat="$1"
  ps -eo pid,cmd | awk -v p="$pat" '$0 ~ p && $0 !~ /awk/ && $0 !~ /restart_mode_b/ {print $1}' | while read -r pid; do
    echo "kill $pid ($pat)"
    kill "$pid" 2>/dev/null || true
  done
}

kill_pat 'agx_mode_b_joint_bridge.py'
kill_pat 'agx_mode_b_car_cmd_bridge.py'
# keep p3d_tf unless requested
if [ "${RESTART_P3D:-0}" = "1" ]; then
  kill_pat 'agx_mode_b_p3d_tf.py'
fi
sleep 1

HOLD_HZ="${HOLD_HZ:-50}"
python3 /workspace/scripts/agx_mode_b_joint_bridge.py --arm-pose "$ARM_POSE" --hold-hz "$HOLD_HZ" \
  >"$LOGDIR/joint_bridge.log" 2>&1 &
echo "joint_bridge pid=$! hold_hz=$HOLD_HZ"

python3 /workspace/scripts/agx_mode_b_car_cmd_bridge.py \
  >"$LOGDIR/car_cmd_bridge.log" 2>&1 &
echo "car_cmd_bridge pid=$!"

if [ "${RESTART_P3D:-0}" = "1" ]; then
  python3 /workspace/scripts/agx_mode_b_p3d_tf.py >"$LOGDIR/p3d_tf.log" 2>&1 &
  echo "p3d_tf pid=$!"
fi

sleep 2
timeout 3 rostopic info /joint_states | head -20 || true
timeout 3 rostopic hz /joint_states 2>&1 | head -6 || true
echo "MODE_B_BRIDGES_RESTARTED"
