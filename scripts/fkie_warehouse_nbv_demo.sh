#!/usr/bin/env bash
# Reproduce FKIE NBV in official indoor warehouse world (with GUI).
# Adaptation: spawn depth camera into warehouse; not full mobile-manipulator stack.
# Keeps Gazebo/RViz running after one Action call for inspection.
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /workspace/ws_fkie/devel/setup.bash

export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export GAZEBO_MODEL_PATH="/workspace/third_party/gazebo_models:${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"

LOGDIR=/workspace/logs/fkie_warehouse_nbv
EVDIR=/workspace/data/metrics/fkie_warehouse_nbv
mkdir -p "${LOGDIR}" "${EVDIR}"

WORLD=/workspace/third_party/fkie-nbv-planner/worlds/warehouse_icra_large.world
CAM_SDF=/workspace/configs/fkie/adapt_test/nbv_depth_cam/model.sdf
# Pose matches nbv_planner_freespace_fast.yaml ROI center
CAM_X=-5.36
CAM_Y=-5.685
CAM_Z=1.0
CAM_PITCH=0.35

cleanup() {
  set +e
  echo "[warehouse-nbv] shutting down..."
  rosnode kill -a >/dev/null 2>&1 || true
  kill $(jobs -p) >/dev/null 2>&1 || true
  sleep 1
  killall -9 gzserver gzclient rviz 2>/dev/null || true
}
trap cleanup EXIT INT TERM

killall -9 gzserver gzclient rviz 2>/dev/null || true
sleep 1

echo "[warehouse-nbv] world=${WORLD}"
echo "[warehouse-nbv] GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}"
test -f "${WORLD}"
test -d /workspace/third_party/gazebo_models/shelves_high/meshes
test -f "${CAM_SDF}"

roscore >"${LOGDIR}/roscore.log" 2>&1 &
sleep 3
rosparam set /use_sim_time true

echo "[warehouse-nbv] starting Gazebo GUI (warehouse)..."
# Single /gazebo ROS node (do not start gzserver+gzclient via gazebo_ros separately)
rosrun gazebo_ros gazebo --verbose "${WORLD}" >"${LOGDIR}/gazebo.log" 2>&1 &
for i in $(seq 1 90); do
  if rosservice list 2>/dev/null | grep -q '/gazebo/spawn_sdf_model'; then
    echo "[warehouse-nbv] gazebo API ready (${i}s)"
    break
  fi
  sleep 1
done
rosservice call /gazebo/unpause_physics "{}" >/dev/null 2>&1 || true

# Wait until shelves mesh resolved (no more Failed to find in recent log) and clock ticks
for i in $(seq 1 60); do
  if timeout 3 rostopic echo -n 1 /clock >/dev/null 2>&1; then
    echo "[warehouse-nbv] /clock OK"
    break
  fi
  sleep 1
done

echo "[warehouse-nbv] spawning depth camera at (${CAM_X}, ${CAM_Y}, ${CAM_Z}) pitch=${CAM_PITCH}"
rosrun gazebo_ros spawn_model -file "${CAM_SDF}" -sdf -model nbv_depth_cam \
  -x "${CAM_X}" -y "${CAM_Y}" -z "${CAM_Z}" -P "${CAM_PITCH}" \
  >"${LOGDIR}/spawn_cam.log" 2>&1 || {
  echo "spawn failed:"; cat "${LOGDIR}/spawn_cam.log"; exit 2
}

# TF for FKIE
rosrun tf2_ros static_transform_publisher "${CAM_X}" "${CAM_Y}" "${CAM_Z}" 0 "${CAM_PITCH}" 0 world realsense_depth_optical_frame &
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 realsense_depth_optical_frame realsense_color_frame &
rosrun tf2_ros static_transform_publisher "${CAM_X}" "${CAM_Y}" 0.8 0 0 0 world turret_link &
rosrun tf2_ros static_transform_publisher "${CAM_X}" "${CAM_Y}" 0.0 0 0 0 world robot/base_link &

# points → points2
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

if [ -f /workspace/configs/fkie/adapt_test/nbv_viz.rviz ]; then
  rviz -d /workspace/configs/fkie/adapt_test/nbv_viz.rviz >"${LOGDIR}/rviz.log" 2>&1 &
fi

echo "[warehouse-nbv] waiting for live depth + octomap..."
READY=0
for i in $(seq 1 90); do
  if timeout 4 rostopic echo -n 1 /realsense/depth/image_raw >/dev/null 2>&1 && \
     timeout 6 rostopic echo -n 1 /robot/octomap_full >/dev/null 2>&1; then
    READY=1
    echo "[warehouse-nbv] sensors live at ${i}*2s"
    break
  fi
  sleep 2
done

if [ "${READY}" != "1" ]; then
  echo "FKIE_WAREHOUSE_NBV_FAIL: no live depth/octomap"
  grep -iE "Failed to find mesh|Error|spawn" "${LOGDIR}/gazebo.log" "${LOGDIR}/spawn_cam.log" | tail -30 || true
  exit 2
fi

python3 - <<'PY' | tee "${EVDIR}/octomap_before.txt"
import rospy
from octomap_msgs.msg import Octomap
from sensor_msgs.msg import Image, PointCloud2
rospy.init_node("wh_probe")
img = rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=10)
print("depth %dx%d enc=%s" % (img.width, img.height, img.encoding))
try:
    pc = rospy.wait_for_message("/realsense/depth/points2", PointCloud2, timeout=5)
    print("points2 %dx%d data_len=%d" % (pc.width, pc.height, len(pc.data)))
except Exception as e:
    print("points2:", e)
om = rospy.wait_for_message("/robot/octomap_full", Octomap, timeout=10)
print("octomap frame=%s res=%s data_len=%d id=%s" % (om.header.frame_id, om.resolution, len(om.data), om.id))
PY

sleep 10
echo "[warehouse-nbv] calling NBV action..."
python3 /workspace/scripts/verify_fkie_action.py 2>&1 | tee "${EVDIR}/action_result.txt"

python3 - <<'PY' | tee "${EVDIR}/summary.txt"
import re, pathlib
text = pathlib.Path("/workspace/data/metrics/fkie_warehouse_nbv/action_result.txt").read_text()
m = re.search(r"goals=(\d+)", text)
n = int(m.group(1)) if m else -1
ok = "VERIFY_FKIE_ACTION_OK" in text and n > 0
print("goals=%d nonempty=%s" % (n, ok))
print(text)
if ok:
    print("FKIE_WAREHOUSE_NBV_OK")
else:
    print("FKIE_WAREHOUSE_NBV_PARTIAL_OR_FAIL")
PY

tail -40 "${LOGDIR}/planner.log" | tee "${EVDIR}/planner_tail.txt" >/dev/null || true

echo ""
echo "============================================================"
echo " Warehouse FKIE demo running."
echo " Gazebo: indoor shelves world + spawned depth camera"
echo " RViz: Fixed Frame=world"
echo " Evidence: ${EVDIR}/"
echo " Leave this running to inspect; Ctrl+C to stop."
echo "============================================================"

while true; do sleep 3600; done
