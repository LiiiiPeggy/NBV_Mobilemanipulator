#!/usr/bin/env bash
# Start roscore + planner node, publish synthetic camera_pose, call action.
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /workspace/ws_fkie/devel/setup.bash

LOGDIR=/workspace/logs/fkie_smoke
mkdir -p "${LOGDIR}"

cleanup() {
  set +e
  rosnode kill -a >/dev/null 2>&1 || true
  kill $(jobs -p) >/dev/null 2>&1 || true
  sleep 1
}
trap cleanup EXIT

roscore >/dev/null 2>&1 &
sleep 2

# Static TF: world -> realsense_color_frame (camera at 1m height)
rosrun tf2_ros static_transform_publisher 0 0 1 0 0 0 1 world realsense_color_frame &
sleep 1

# Publish a steady camera_pose so planner initializes
python3 - <<'PY' &
import rospy
from geometry_msgs.msg import PoseStamped
rospy.init_node("fake_camera_pose")
pub = rospy.Publisher("/robot/camera_pose", PoseStamped, queue_size=1)
rate = rospy.Rate(5)
while not rospy.is_shutdown():
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.pose.position.z = 1.0
    msg.pose.orientation.w = 1.0
    pub.publish(msg)
    rate.sleep()
PY

# Publish a small occupied OcTree on the planner's remapped topic name.
# With rosrun (no remap), the node subscribes to /robot/octomap.
python3 - <<'PY' &
import rospy
from octomap_msgs.msg import Octomap
rospy.init_node("fake_octomap")
pub = rospy.Publisher("/robot/octomap", Octomap, queue_size=1, latch=True)
data = []
try:
    import subprocess, tempfile, os
    # Use octomap CLI if present to create a tiny binary tree
    tmp = tempfile.NamedTemporaryFile(suffix=".bt", delete=False)
    tmp.close()
    # Fallback: empty binary id so callback at least receives messages;
    # planner may abort planning without real geometry — still validates Action path.
    msg = Octomap()
    msg.header.frame_id = "world"
    msg.binary = True
    msg.id = "OcTree"
    msg.resolution = 0.1
    msg.data = list(data)
except Exception as e:
    rospy.logwarn("octomap setup issue: %s", e)
    msg = Octomap()
    msg.header.frame_id = "world"
    msg.binary = True
    msg.id = "OcTree"
    msg.resolution = 0.1
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rate.sleep()
PY

# Load YAML defaults via roslaunch-less param dump
rosparam load /workspace/third_party/fkie-nbv-planner/config/nbv_planner_freespace_fast.yaml /robot/fkie_nbv_planner || true

# Also publish TF for robot_sampling_frame (turret_link)
rosrun tf2_ros static_transform_publisher 0 0 0.8 0 0 0 1 world turret_link &

rosrun fkie_nbv_planner nbvplanner_node __ns:=robot __name:=fkie_nbv_planner \
  >/workspace/logs/fkie_smoke/planner.log 2>&1 &
sleep 8

rosnode list | tee "${LOGDIR}/nodes.txt"
rostopic list | tee "${LOGDIR}/topics.txt"

python3 /workspace/scripts/verify_fkie_action.py | tee "${LOGDIR}/action_result.txt"
echo "FKIE smoke test finished"
