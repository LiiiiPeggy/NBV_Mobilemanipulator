#!/usr/bin/env bash
# AGX (Ranger+CR10+AG95+D455) + FKIE warehouse NBV — Mode A static perception.
# AGX_MODE=static|dynamic (default static). Static ≠ whole-body motion closed loop.
set -euo pipefail

source /opt/ros/noetic/setup.bash
if [ -f /workspace/ws_fkie/devel/setup.bash ]; then
  # shellcheck disable=SC1091
  source /workspace/ws_fkie/devel/setup.bash
fi
export ROS_PACKAGE_PATH="/workspace/third_party/agx/rangerboxcr10lidar_description:${ROS_PACKAGE_PATH:-}"
export ROS_PACKAGE_PATH="/workspace/ws_fkie/src:${ROS_PACKAGE_PATH}"

export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export GAZEBO_MODEL_PATH="/workspace/third_party/gazebo_models:${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"

AGX_MODE="${AGX_MODE:-static}"
LOGDIR=/workspace/logs/fkie_agx_warehouse_nbv
EVDIR=/workspace/data/metrics/fkie_agx_warehouse_nbv
SHOTDIR=/workspace/reports/fkie_agx_integration_shots
mkdir -p "${LOGDIR}" "${EVDIR}" "${SHOTDIR}"

WORLD=/workspace/configs/fkie/adapt_test/warehouse_icra_large_viz.world
URDF_SRC=/workspace/third_party/agx/rangerboxcr10lidar_description/urdf/rangercr10lidar.urdf
URDF_GZ=/workspace/configs/fkie/agx_adapt/urdf/rangercr10lidar_gazebo.urdf
POSE_YAML=/workspace/configs/fkie/agx_adapt/config/arm_view_pose.yaml
PLANNER_YAML=/workspace/configs/fkie/agx_adapt/config/nbv_planner_agx_warehouse.yaml
RVIZ_DBG=/workspace/configs/fkie/agx_adapt/rviz/agx_debug.rviz
RVIZ_CAM=/workspace/configs/fkie/agx_adapt/rviz/agx_camera.rviz

# Wheel center ≈ base_z - 0.258; radius ≈ 0.10 → base_z ≈ 0.36 on ground
RX=-5.36
RY=-5.685
RZ="${AGX_SPAWN_Z:-0.36}"
RYAW=0.0

PIDS=()
register() { PIDS+=("$1"); }

cleanup() {
  set +e
  echo "[agx-nbv] cleanup PIDs: ${PIDS[*]:-}"
  for p in "${PIDS[@]:-}"; do kill "$p" >/dev/null 2>&1 || true; done
  sleep 1
  for p in "${PIDS[@]:-}"; do kill -9 "$p" >/dev/null 2>&1 || true; done
  killall -TERM gzserver gzclient rviz 2>/dev/null || true
  sleep 1
  killall -9 gzserver gzclient rviz 2>/dev/null || true
}
trap cleanup EXIT INT TERM

STOP_SIM_KEEP_PID=$$ bash /workspace/scripts/fkie_stop_sim.sh || true
sleep 2

echo "[agx-nbv] mode=${AGX_MODE} spawn_z=${RZ}"
python3 /workspace/configs/fkie/agx_adapt/scripts/generate_gazebo_urdf.py \
  --src "${URDF_SRC}" --dst "${URDF_GZ}" --mode "${AGX_MODE}"

test -f "${WORLD}"
test -f "${URDF_GZ}"
test -d /workspace/third_party/gazebo_models/shelves_high/meshes

if ! timeout 2 rostopic list >/dev/null 2>&1; then
  roscore >"${LOGDIR}/roscore.log" 2>&1 &
  register $!
  sleep 3
fi
rosparam set /use_sim_time true

echo "[agx-nbv] Gazebo: ${WORLD}"
rosrun gazebo_ros gazebo --verbose "${WORLD}" >"${LOGDIR}/gazebo.log" 2>&1 &
register $!
for i in $(seq 1 120); do
  if rosservice list 2>/dev/null | grep -q '/gazebo/spawn_urdf_model'; then
    echo "[agx-nbv] gazebo API ready (${i}s)"; break
  fi
  sleep 1
done

python3 - <<PY
import rospy
from pathlib import Path
rospy.init_node("load_robot_description", anonymous=True)
xml = Path("${URDF_GZ}").read_text()
rospy.set_param("/robot_description", xml)
print("robot_description bytes", len(xml))
PY

# Build -J args from pose yaml
mapfile -t JARGS < <(python3 - <<'PY'
import yaml
from pathlib import Path
pose = yaml.safe_load(Path("/workspace/configs/fkie/agx_adapt/config/arm_view_pose.yaml").read_text())
for k, v in pose.items():
    print("-J"); print(k); print(float(v))
PY
)

echo "[agx-nbv] spawn AGX at (${RX},${RY},${RZ}) yaw=${RYAW} mode=${AGX_MODE}"
rosrun gazebo_ros spawn_model -urdf -file "${URDF_GZ}" -model agx_ranger_cr10 \
  -x "${RX}" -y "${RY}" -z "${RZ}" -Y "${RYAW}" \
  "${JARGS[@]}" \
  >"${LOGDIR}/spawn_robot.log" 2>&1 || {
  echo "spawn failed:"; cat "${LOGDIR}/spawn_robot.log"; exit 2
}

# Also set configuration (helps when -J ignored)
python3 - <<'PY' >"${LOGDIR}/set_arm_pose.log" 2>&1
import rospy, yaml
from gazebo_msgs.srv import SetModelConfiguration
from pathlib import Path
rospy.init_node("set_agx_arm_pose")
pose = yaml.safe_load(Path("/workspace/configs/fkie/agx_adapt/config/arm_view_pose.yaml").read_text())
rospy.wait_for_service("/gazebo/set_model_configuration", timeout=30)
srv = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
names = list(pose.keys())
pos = [float(pose[n]) for n in names]
resp = srv("agx_ranger_cr10", "", names, pos)
print("success", resp.success, "status", resp.status_message)
PY

# Single authoritative joint_states in Mode A
python3 - <<'PY' >"${LOGDIR}/joint_states.log" 2>&1 &
import rospy, yaml
from sensor_msgs.msg import JointState
from pathlib import Path
rospy.init_node("agx_joint_states_authority")
pose = yaml.safe_load(Path("/workspace/configs/fkie/agx_adapt/config/arm_view_pose.yaml").read_text())
extras = [
  "fr_steering_joint","fr_wheel_joint","fl_steering_wheel_joint","fl_wheel_joint",
  "rl_steering_wheel_joint","rl_wheel_joint","rr_steering_wheel_joint","rr_wheel_joint",
]
pub = rospy.Publisher("/joint_states", JointState, queue_size=1, latch=False)
rate = rospy.Rate(50)
while not rospy.is_shutdown():
  msg = JointState()
  msg.header.stamp = rospy.Time.now()
  msg.name = extras + list(pose.keys())
  msg.position = [0.0]*len(extras) + [float(pose[n]) for n in pose.keys()]
  pub.publish(msg)
  rate.sleep()
PY
register $!

rosrun robot_state_publisher robot_state_publisher >"${LOGDIR}/rsp.log" 2>&1 &
register $!

# Mode A: ONLY static world→base_link (no p3d / no competing TF)
python3 - <<PY >"${LOGDIR}/tf_world_base.log" 2>&1 &
import rospy, math
import tf2_ros
from geometry_msgs.msg import TransformStamped
rospy.init_node("agx_world_base_tf_static")
br = tf2_ros.TransformBroadcaster()
rx, ry, rz, ryaw = ${RX}, ${RY}, ${RZ}, ${RYAW}
q = (
  0.0,
  0.0,
  math.sin(ryaw/2.0),
  math.cos(ryaw/2.0),
)
rate = rospy.Rate(50)
while not rospy.is_shutdown():
  t = TransformStamped()
  t.header.stamp = rospy.Time.now()
  t.header.frame_id = "world"
  t.child_frame_id = "base_link"
  t.transform.translation.x = rx
  t.transform.translation.y = ry
  t.transform.translation.z = rz
  t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
  br.sendTransform(t)
  rate.sleep()
PY
register $!

rosservice call /gazebo/unpause_physics "{}" >/dev/null 2>&1 || true

# Single points2 publisher
python3 - <<'PY' >"${LOGDIR}/relay_pc.log" 2>&1 &
import rospy
from sensor_msgs.msg import PointCloud2
rospy.init_node("relay_points2_only")
pub = rospy.Publisher("/realsense/depth/points2", PointCloud2, queue_size=1)
rospy.Subscriber("/realsense/depth/points", PointCloud2, lambda m: pub.publish(m), queue_size=1)
rospy.spin()
PY
register $!

roslaunch /workspace/configs/fkie/agx_adapt/launch/octomap_agx.launch robot_ns:=robot \
  >"${LOGDIR}/octomap.log" 2>&1 &
register $!

rosrun fkie_nbv_planner camera_pose_tf.py __name:=camera_pose_pub \
  _world_frame:=world _sensor_frame:=d455_color_optical_frame _rate:=10.0 \
  camera_pose:=/robot/camera_pose >"${LOGDIR}/cam_pose.log" 2>&1 &
register $!

python3 - <<'PY' >"${LOGDIR}/footprint.log" 2>&1 &
import rospy, math
import tf2_ros
from geometry_msgs.msg import PolygonStamped, Point32
rospy.init_node("agx_ranger_footprint")
pub = rospy.Publisher("/robot/robot_bbx", PolygonStamped, queue_size=1)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
hx, hy = 0.50, 0.35
rate = rospy.Rate(5)
while not rospy.is_shutdown():
  try:
    t = buf.lookup_transform("world", "base_link", rospy.Time(0), rospy.Duration(0.5))
    x, y = t.transform.translation.x, t.transform.translation.y
    q = t.transform.rotation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    msg = PolygonStamped(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = "world"
    for lx, ly in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]:
      msg.polygon.points.append(Point32(x=x+c*lx-s*ly, y=y+s*lx+c*ly, z=0.0))
    pub.publish(msg)
  except Exception:
    pass
  rate.sleep()
PY
register $!

rosparam load "${PLANNER_YAML}" /robot/fkie_nbv_planner
# Match octomap max range
rosparam set /robot/fkie_nbv_planner/gain_r_max 5.0
rosrun fkie_nbv_planner nbvplanner_node __ns:=robot __name:=fkie_nbv_planner \
  octomap:=octomap_full >"${LOGDIR}/planner.log" 2>&1 &
register $!

rviz -d "${RVIZ_DBG}" >"${LOGDIR}/rviz_debug.log" 2>&1 &
register $!
rviz -d "${RVIZ_CAM}" >"${LOGDIR}/rviz_camera.log" 2>&1 &
register $!

echo "[agx-nbv] waiting for sensors..."
READY=0
for i in $(seq 1 120); do
  python3 - <<'PY' && READY=1 && break
import rospy
from sensor_msgs.msg import Image, PointCloud2
from octomap_msgs.msg import Octomap
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState
rospy.init_node("agx_ready_probe", anonymous=True)
img = rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=3.0)
assert img.encoding not in ("rgb8", "bgr8"), img.encoding
pc = rospy.wait_for_message("/realsense/depth/points2", PointCloud2, timeout=3.0)
assert len(pc.data) > 100
om = rospy.wait_for_message("/robot/octomap_full", Octomap, timeout=5.0)
assert len(om.data) > 50
pose = rospy.wait_for_message("/robot/camera_pose", PoseStamped, timeout=3.0)
rospy.wait_for_service("/gazebo/get_model_state", timeout=5)
st = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("agx_ranger_cr10", "world")
r = st.pose.orientation
# roll/pitch small for upright
import math
sinr = 2*(r.w*r.x + r.y*r.z); cosr = 1-2*(r.x*r.x+r.y*r.y)
sinp = 2*(r.w*r.y - r.z*r.x)
roll, pitch = math.atan2(sinr, cosr), math.asin(max(-1,min(1,sinp)))
print("ready depth=%s oct=%d cam=(%.2f,%.2f,%.2f) model_xyz=(%.2f,%.2f,%.2f) roll=%.3f pitch=%.3f" % (
  img.encoding, len(om.data),
  pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
  st.pose.position.x, st.pose.position.y, st.pose.position.z, roll, pitch))
assert abs(roll) < 0.25 and abs(pitch) < 0.25, "robot not upright"
PY
  sleep 2
done

python3 /workspace/scripts/fkie_agx_diag.py 2>&1 | tee "${EVDIR}/diag_before.json"

# Stability snapshot
python3 - <<'PY' | tee "${EVDIR}/model_pose.txt"
import rospy, math
from gazebo_msgs.srv import GetModelState
rospy.init_node("agx_pose_snap")
rospy.wait_for_service("/gazebo/get_model_state")
st = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("agx_ranger_cr10", "world")
r = st.pose.orientation
sinr = 2*(r.w*r.x + r.y*r.z); cosr = 1-2*(r.x*r.x+r.y*r.y)
sinp = 2*(r.w*r.y - r.z*r.x)
roll, pitch = math.atan2(sinr, cosr), math.asin(max(-1,min(1,sinp)))
print("success=%s xyz=(%.3f,%.3f,%.3f) roll=%.4f pitch=%.4f" % (
  st.success, st.pose.position.x, st.pose.position.y, st.pose.position.z, roll, pitch))
PY

if [ "${READY}" != "1" ]; then
  echo "FKIE_AGX_NBV_FAIL: sensors/upright not ready"
  tail -40 "${LOGDIR}/gazebo.log" || true
  exit 2
fi

sleep 15
echo "[agx-nbv] waiting for action server..."
for i in $(seq 1 60); do
  if timeout 2 rostopic list 2>/dev/null | grep -q '/robot/nbv_rrt/goal'; then
    echo "[agx-nbv] action ready (${i}s)"; break
  fi
  sleep 1
done

set +e
python3 /workspace/scripts/verify_fkie_action.py _timeout:=90.0 2>&1 | tee "${EVDIR}/action_result.txt"
set -e

python3 - <<'PY' | tee "${EVDIR}/summary.txt"
import re, pathlib
text = pathlib.Path("/workspace/data/metrics/fkie_agx_warehouse_nbv/action_result.txt").read_text()
m = re.search(r"goals=(\d+)", text)
n = int(m.group(1)) if m else -1
ok = "VERIFY_FKIE_ACTION_OK" in text and n > 0
print("goals=%d nonempty=%s" % (n, ok))
print(text)
print("FKIE_AGX_WAREHOUSE_NBV_OK" if ok else "FKIE_AGX_WAREHOUSE_NBV_PARTIAL_OR_FAIL")
pathlib.Path("/workspace/data/metrics/fkie_agx_warehouse_nbv/RESULT.txt").write_text(
  "goals=%d nonempty=%s mode=static\n" % (n, ok))
PY
tail -80 "${LOGDIR}/planner.log" | tee "${EVDIR}/planner_tail.txt" >/dev/null || true

# Screenshots via ImageMagick if present
if command -v import >/dev/null 2>&1; then
  DISPLAY="${DISPLAY}" import -window root "${SHOTDIR}/desktop_$(date +%H%M%S).png" || true
fi

echo "============================================================"
echo " AGX Mode A running. gzserver/gzclient should be 1 each."
echo " RViz: agx_debug + agx_camera (no TF in CamView)."
echo " Evidence: ${EVDIR}/"
echo "============================================================"
while true; do sleep 3600; done
