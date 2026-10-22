#!/usr/bin/env bash
# ADAPTATION TEST: Gazebo depth → OctoMap → FKIE NBV with non-empty goals check.
# Not official mobile-manipulator stack. Requires container nbv_fkie + sourced workspace.
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /workspace/ws_fkie/devel/setup.bash
export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
# Isolate Gazebo Classic master from other host processes
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
# gazebo_ros_* plugins dlopen Gazebo DepthCameraPlugin; also need ROS API plugin path
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"

LOGDIR=/workspace/logs/fkie_effective_nbv
EVDIR=/workspace/data/metrics/fkie_effective_nbv
mkdir -p "${LOGDIR}" "${EVDIR}"
WORLD=/workspace/configs/fkie/adapt_test/nbv_minimal.world

cleanup() {
  set +e
  rosnode kill -a >/dev/null 2>&1 || true
  kill $(jobs -p) >/dev/null 2>&1 || true
  sleep 2
  killall -9 gzserver gzclient 2>/dev/null || true
}
trap cleanup EXIT

echo "[fkie-adapt] starting roscore"
roscore >"${LOGDIR}/roscore.log" 2>&1 &
sleep 3

echo "[fkie-adapt] starting gzserver (with gazebo_ros_api) world=${WORLD}"
# Camera plugins require ROS initialized via libgazebo_ros_api_plugin.so
rosrun gazebo_ros gzserver --verbose "${WORLD}" >"${LOGDIR}/gzserver.log" 2>&1 &
sleep 10

# TF tree
rosrun tf2_ros static_transform_publisher -5.36 -5.685 1.0 0 0.35 0 world realsense_depth_optical_frame &
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 realsense_depth_optical_frame realsense_color_frame &
rosrun tf2_ros static_transform_publisher -5.36 -5.685 0.8 0 0 0 world turret_link &
rosrun tf2_ros static_transform_publisher -5.36 -5.685 0.0 0 0 0 world robot/base_link &
sleep 1

# Relay / remaps for point cloud naming expected by octomap_server.launch
if ! rosrun topic_tools relay /realsense/depth/points /realsense/depth/points2 >"${LOGDIR}/relay.log" 2>&1 & then
  echo "topic_tools relay failed; trying republish via python"
fi
sleep 2

# Also accept depth image path if points are empty: convert via depth_image_proc
rosrun nodelet nodelet manager __name:=camera_manager >"${LOGDIR}/nodelet_mgr.log" 2>&1 &
sleep 1
# Prefer points from kinect plugin; if only depth image, build XYZ
rosrun nodelet nodelet load depth_image_proc/point_cloud_xyz camera_manager __name:=image_to_points \
  camera_info:=/realsense/depth/camera_info \
  image_rect:=/realsense/depth/image_raw \
  points:=/realsense/depth/points2 \
  >"${LOGDIR}/depth_to_pc.log" 2>&1 &

roslaunch fkie_nbv_planner octomap_server.launch robot_ns:=robot >"${LOGDIR}/octomap.log" 2>&1 &

rosrun fkie_nbv_planner camera_pose_tf.py __name:=camera_pose_pub \
  _world_frame:=world _sensor_frame:=realsense_color_frame _rate:=10.0 \
  camera_pose:=/robot/camera_pose >"${LOGDIR}/cam_pose.log" 2>&1 &

# Footprint
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
# Official run_planner.launch remaps octomap -> octomap_full
rosrun fkie_nbv_planner nbvplanner_node __ns:=robot __name:=fkie_nbv_planner \
  octomap:=octomap_full \
  >"${LOGDIR}/planner.log" 2>&1 &

echo "[fkie-adapt] waiting for depth / octomap..."
READY=0
for i in $(seq 1 60); do
  TOPICS=$(rostopic list 2>/dev/null || true)
  echo "$TOPICS" | grep -q '/realsense/depth' || true
  if echo "$TOPICS" | grep -q '/robot/octomap_full' && echo "$TOPICS" | grep -q '/robot/nbv_rrt/goal'; then
    # check non-empty point cloud or octomap size
    python3 - <<'PY' && READY=1 && break
import rospy
from sensor_msgs.msg import PointCloud2, Image
from octomap_msgs.msg import Octomap
rospy.init_node("probe_ready", anonymous=True)
ok = False
try:
    pc = rospy.wait_for_message("/realsense/depth/points2", PointCloud2, timeout=3.0)
    print("PC width=%s height=%s data_len=%s" % (pc.width, pc.height, len(pc.data)))
    if len(pc.data) > 100:
        ok = True
except Exception as e:
    print("points2 wait:", e)
try:
    img = rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=2.0)
    print("depth image %sx%s enc=%s" % (img.width, img.height, img.encoding))
    if img.width > 0:
        ok = True
except Exception as e:
    print("depth image wait:", e)
try:
    om = rospy.wait_for_message("/robot/octomap_full", Octomap, timeout=5.0)
    print("octomap id=%s res=%s data_len=%s binary=%s" % (om.id, om.resolution, len(om.data), om.binary))
    if len(om.data) > 50:
        ok = True
except Exception as e:
    print("octomap wait:", e)
raise SystemExit(0 if ok else 1)
PY
  fi
  sleep 2
done

rostopic list | tee "${LOGDIR}/topics.txt" >/dev/null
cp "${LOGDIR}/topics.txt" "${EVDIR}/topics.txt"

if [ "${READY}" != "1" ]; then
  echo "FKIE_EFFECTIVE_NBV_FAIL: no live depth/octomap"
  tail -40 "${LOGDIR}/gzserver.log" || true
  tail -40 "${LOGDIR}/octomap.log" || true
  exit 2
fi

# Snapshot octomap stats before planning
python3 - <<'PY' | tee "${EVDIR}/octomap_before.txt"
import rospy
from octomap_msgs.msg import Octomap
rospy.init_node("om_before")
om = rospy.wait_for_message("/robot/octomap_full", Octomap, timeout=10.0)
print("frame=%s res=%s data_len=%s id=%s" % (om.header.frame_id, om.resolution, len(om.data), om.id))
# rospy int8[] may contain negatives; mask to unsigned bytes for dump
open("/workspace/data/metrics/fkie_effective_nbv/octomap_before.bin", "wb").write(
    bytearray((d & 0xFF) for d in om.data)
)
PY

# Wait a bit more so octomap integrates several frames
sleep 8

python3 /workspace/scripts/verify_fkie_action.py | tee "${EVDIR}/action_result.txt"
# Also dump planner log snippet
tail -80 "${LOGDIR}/planner.log" | tee "${EVDIR}/planner_tail.txt" >/dev/null

python3 - <<'PY' | tee "${EVDIR}/action_parse.txt"
import re, pathlib
text = pathlib.Path("/workspace/data/metrics/fkie_effective_nbv/action_result.txt").read_text()
print(text)
m = re.search(r"goals=(\d+)", text)
n = int(m.group(1)) if m else -1
ok_marker = "VERIFY_FKIE_ACTION_OK" in text
pathlib.Path("/workspace/data/metrics/fkie_effective_nbv/summary.txt").write_text(
    "goals=%d action_ok=%s\n" % (n, ok_marker)
)
if ok_marker and n > 0:
    print("FKIE_EFFECTIVE_NBV_OK goals=%d" % n)
    raise SystemExit(0)
print("FKIE_EFFECTIVE_NBV_PARTIAL_OR_FAIL goals=%d" % n)
raise SystemExit(3)
PY
