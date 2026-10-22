#!/usr/bin/env bash
# Interactive FKIE adaptation visualization: Gazebo GUI + RViz + one NBV action call.
# Leave running until Ctrl+C. Does not auto-kill on success.
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /workspace/ws_fkie/devel/setup.bash
export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"

LOGDIR=/workspace/logs/fkie_viz
mkdir -p "${LOGDIR}"
WORLD=/workspace/configs/fkie/adapt_test/nbv_minimal.world

cleanup() {
  set +e
  echo "[fkie-viz] shutting down..."
  rosnode kill -a >/dev/null 2>&1 || true
  kill $(jobs -p) >/dev/null 2>&1 || true
  sleep 1
  killall -9 gzserver gzclient rviz 2>/dev/null || true
}
trap cleanup EXIT INT TERM

killall -9 gzserver gzclient 2>/dev/null || true
sleep 1

echo "[fkie-viz] DISPLAY=${DISPLAY} world=${WORLD}"
roscore >"${LOGDIR}/roscore.log" 2>&1 &
sleep 3

echo "[fkie-viz] starting Gazebo GUI (single /gazebo ROS node)..."
# IMPORTANT: do NOT start gazebo_ros gzserver + gazebo_ros gzclient separately —
# both register node [/gazebo] and the client shuts down the server API (/clock dies).
rosparam set /use_sim_time true
rosrun gazebo_ros gazebo --verbose "${WORLD}" >"${LOGDIR}/gazebo.log" 2>&1 &
for i in $(seq 1 90); do
  if rosservice list 2>/dev/null | grep -q '/gazebo/unpause_physics'; then
    echo "[fkie-viz] gazebo ROS API ready (${i}s)"
    break
  fi
  sleep 1
done
rosservice call /gazebo/unpause_physics "{}" >/dev/null 2>&1 || true
sleep 2
if timeout 5 rostopic echo -n 1 /clock >/dev/null 2>&1; then
  echo "[fkie-viz] /clock OK"
else
  echo "[fkie-viz] WARN: /clock not publishing; check ${LOGDIR}/gazebo.log"
  tail -50 "${LOGDIR}/gazebo.log" || true
fi

# TF
rosrun tf2_ros static_transform_publisher -5.36 -5.685 1.0 0 0.35 0 world realsense_depth_optical_frame &
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 realsense_depth_optical_frame realsense_color_frame &
rosrun tf2_ros static_transform_publisher -5.36 -5.685 0.8 0 0 0 world turret_link &
rosrun tf2_ros static_transform_publisher -5.36 -5.685 0.0 0 0 0 world robot/base_link &

# Points → points2
python3 - <<'PY' >"${LOGDIR}/relay_pc.log" 2>&1 &
import rospy
from sensor_msgs.msg import PointCloud2
rospy.init_node("relay_points2")
pub = rospy.Publisher("/realsense/depth/points2", PointCloud2, queue_size=1)
def cb(msg):
    pub.publish(msg)
rospy.Subscriber("/realsense/depth/points", PointCloud2, cb, queue_size=1)
rospy.spin()
PY

# Depth→PC fallback
rosrun nodelet nodelet manager __name:=camera_manager >"${LOGDIR}/nodelet.log" 2>&1 &
sleep 1
rosrun nodelet nodelet load depth_image_proc/point_cloud_xyz camera_manager __name:=image_to_points \
  camera_info:=/realsense/depth/camera_info \
  image_rect:=/realsense/depth/image_raw \
  points:=/realsense/depth/points2 \
  >"${LOGDIR}/depth_pc.log" 2>&1 &

roslaunch fkie_nbv_planner octomap_server.launch robot_ns:=robot >"${LOGDIR}/octomap.log" 2>&1 &

rosrun fkie_nbv_planner camera_pose_tf.py __name:=camera_pose_pub \
  _world_frame:=world _sensor_frame:=realsense_color_frame _rate:=10.0 \
  camera_pose:=/robot/camera_pose >"${LOGDIR}/cam_pose.log" 2>&1 &

python3 - <<'PY' >"${LOGDIR}/footprint.log" 2>&1 &
import rospy
from geometry_msgs.msg import PolygonStamped, Point32
rospy.init_node("adapt_footprint")
pub = rospy.Publisher("/robot/robot_bbx", PolygonStamped, queue_size=1)
rate = rospy.Rate(2)
while not rospy.is_shutdown():
    msg = PolygonStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    for x, y in [(-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3)]:
        msg.polygon.points.append(Point32(x=x, y=y, z=0.0))
    pub.publish(msg)
    rate.sleep()
PY

rosparam load /workspace/third_party/fkie-nbv-planner/config/nbv_planner_freespace_fast.yaml /robot/fkie_nbv_planner
rosrun fkie_nbv_planner nbvplanner_node __ns:=robot __name:=fkie_nbv_planner \
  octomap:=octomap_full >"${LOGDIR}/planner.log" 2>&1 &

# RViz with common markers if available
if command -v rviz >/dev/null 2>&1; then
  echo "[fkie-viz] starting RViz (Fixed Frame = world)..."
  # Minimal config via args; markers from planner topics under /robot/*
  rviz -d /workspace/configs/fkie/adapt_test/nbv_viz.rviz >"${LOGDIR}/rviz.log" 2>&1 &
else
  echo "[fkie-viz] rviz not found; Gazebo GUI only"
fi

echo "[fkie-viz] waiting for live depth + octomap..."
READY=0
for i in $(seq 1 90); do
  if timeout 3 rostopic echo -n 1 /realsense/depth/image_raw >/dev/null 2>&1 && \
     timeout 5 rostopic echo -n 1 /robot/octomap_full >/dev/null 2>&1; then
    READY=1
    echo "[fkie-viz] sensors live at ${i}s"
    break
  fi
  sleep 2
done
if [ "${READY}" != "1" ]; then
  echo "[fkie-viz] WARN: sensors not live yet; still opening GUI for inspection"
fi

sleep 5
echo "[fkie-viz] calling NBV action once..."
python3 /workspace/scripts/verify_fkie_action.py 2>&1 | tee "${LOGDIR}/action_once.txt" || true

echo ""
echo "============================================================"
echo " Visualization is running."
echo " - Gazebo: world with depth camera + boxes"
echo " - RViz: Fixed Frame=world; MarkerArray /robot/viz_* if subscribed"
echo " - Action result: ${LOGDIR}/action_once.txt"
echo " Press Ctrl+C in this terminal to stop."
echo "============================================================"

# Keep alive
while true; do sleep 3600; done
