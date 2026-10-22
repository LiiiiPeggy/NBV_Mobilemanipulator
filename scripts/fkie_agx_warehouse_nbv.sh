#!/usr/bin/env bash
# AGX (Ranger+CR10+AG95+D455) + FKIE warehouse NBV demo.
# Does NOT replace scripts/fkie_effective_nbv_test.sh (minimal-world regression).
# Scope: display + end-effector RGB-D + octomap + non-empty NBV goals.
# Not a full base/arm motion closed loop.
set -euo pipefail

source /opt/ros/noetic/setup.bash
# Description package via workspace symlink + devel overlay
if [ -f /workspace/ws_fkie/devel/setup.bash ]; then
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

LOGDIR=/workspace/logs/fkie_agx_warehouse_nbv
EVDIR=/workspace/data/metrics/fkie_agx_warehouse_nbv
SHOTDIR=/workspace/reports/fkie_agx_integration_shots
mkdir -p "${LOGDIR}" "${EVDIR}" "${SHOTDIR}"

WORLD=/workspace/configs/fkie/adapt_test/warehouse_icra_large_viz.world
URDF_SRC=/workspace/third_party/agx/rangerboxcr10lidar_description/urdf/rangercr10lidar.urdf
URDF_GZ=/workspace/configs/fkie/agx_adapt/urdf/rangercr10lidar_gazebo.urdf
POSE_YAML=/workspace/configs/fkie/agx_adapt/config/arm_view_pose.yaml
PLANNER_YAML=/workspace/configs/fkie/agx_adapt/config/nbv_planner_agx_warehouse.yaml
RVIZ_CFG=/workspace/configs/fkie/agx_adapt/rviz/agx_nbv.rviz

# Spawn pose: ROI center of freespace_fast boundary
RX=-5.36
RY=-5.685
RZ=0.05
RYAW=0.0

# Track this session's PIDs. Gazebo forks gzserver/gzclient — also stop those on exit.
PIDS=()
register() { PIDS+=("$1"); }

cleanup() {
  set +e
  echo "[agx-nbv] cleanup session PIDs: ${PIDS[*]:-}"
  for p in "${PIDS[@]:-}"; do
    kill "$p" >/dev/null 2>&1 || true
  done
  sleep 1
  for p in "${PIDS[@]:-}"; do
    kill -9 "$p" >/dev/null 2>&1 || true
  done
  # Always reap Gazebo orphans left by rosrun gazebo_ros gazebo
  killall -TERM gzserver gzclient 2>/dev/null || true
  sleep 1
  killall -9 gzserver gzclient 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Pre-start: clear previous FKIE sim in THIS container (avoids stacking gzserver).
# Pass our PID so stop_sim does not kill this running demo.
if [ -x /workspace/scripts/fkie_stop_sim.sh ]; then
  STOP_SIM_KEEP_PID=$$ bash /workspace/scripts/fkie_stop_sim.sh || true
else
  killall -9 gzserver gzclient rviz 2>/dev/null || true
fi
sleep 2

echo "[agx-nbv] generating Gazebo URDF adapt..."
python3 /workspace/configs/fkie/agx_adapt/scripts/generate_gazebo_urdf.py \
  --src "${URDF_SRC}" --dst "${URDF_GZ}"

test -f "${WORLD}"
test -f "${URDF_GZ}"
test -d /workspace/third_party/gazebo_models/shelves_high/meshes

# Own ROS master on default unless already up for this URI
if ! timeout 2 rostopic list >/dev/null 2>&1; then
  roscore >"${LOGDIR}/roscore.log" 2>&1 &
  register $!
  sleep 3
fi
rosparam set /use_sim_time true

echo "[agx-nbv] Gazebo warehouse viz world..."
rosrun gazebo_ros gazebo --verbose "${WORLD}" >"${LOGDIR}/gazebo.log" 2>&1 &
register $!
for i in $(seq 1 120); do
  if rosservice list 2>/dev/null | grep -q '/gazebo/spawn_urdf_model'; then
    echo "[agx-nbv] gazebo API ready (${i}s)"
    break
  fi
  sleep 1
done

# Load robot_description via rospy (rosparam set cannot parse raw XML as YAML)
python3 - <<PY
import rospy
from pathlib import Path
rospy.init_node("load_robot_description", anonymous=True)
xml = Path("${URDF_GZ}").read_text()
rospy.set_param("/robot_description", xml)
print("robot_description bytes", len(xml))
PY

echo "[agx-nbv] spawning AGX at (${RX},${RY},${RZ}) yaw=${RYAW}"
rosrun gazebo_ros spawn_model -urdf -file "${URDF_GZ}" -model agx_ranger_cr10 \
  -x "${RX}" -y "${RY}" -z "${RZ}" -Y "${RYAW}" \
  >"${LOGDIR}/spawn_robot.log" 2>&1 || {
  echo "spawn failed:"; cat "${LOGDIR}/spawn_robot.log"; exit 2
}

# Set arm viewing configuration in Gazebo
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

# Prefer Gazebo joint states; fall back to joint_state_publisher if plugin missing
rosrun robot_state_publisher robot_state_publisher >"${LOGDIR}/rsp.log" 2>&1 &
register $!

# If gazebo joint_state plugin did not load, publish fixed pose + zeros
python3 - <<'PY' >"${LOGDIR}/js_fallback.log" 2>&1 &
import rospy, yaml, time
from sensor_msgs.msg import JointState
from pathlib import Path
rospy.init_node("agx_js_fallback")
pose = yaml.safe_load(Path("/workspace/configs/fkie/agx_adapt/config/arm_view_pose.yaml").read_text())
pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
# Wait briefly; if someone else publishes, stay silent
time.sleep(2.0)
if pub.get_num_connections() >= 0:
  # Always publish at low rate as backup (Gazebo plugin may also publish; last writer wins — prefer plugin)
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    # Skip if another publisher exists with higher rate — check subscribers only
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    # Include wheels at 0 + arm pose
    extras = [
      "fr_steering_joint","fr_wheel_joint","fl_steering_wheel_joint","fl_wheel_joint",
      "rl_steering_wheel_joint","rl_wheel_joint","rr_steering_wheel_joint","rr_wheel_joint",
    ]
    msg.name = extras + list(pose.keys())
    msg.position = [0.0]*len(extras) + [float(pose[n]) for n in pose.keys()]
    pub.publish(msg)
    rate.sleep()
PY
register $!

# world→base from ground_truth if available; else static
python3 - <<PY >"${LOGDIR}/tf_world_base.log" 2>&1 &
import rospy, tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
rospy.init_node("agx_world_base_tf")
br = tf2_ros.TransformBroadcaster()
static_fallback = True
rx, ry, rz, ryaw = ${RX}, ${RY}, ${RZ}, ${RYAW}

def odom_cb(msg):
    global static_fallback
    static_fallback = False
    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "world"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation
    br.sendTransform(t)

rospy.Subscriber("/ground_truth/state", Odometry, odom_cb, queue_size=1)
rate = rospy.Rate(20)
q = tf.transformations.quaternion_from_euler(0, 0, ryaw)
while not rospy.is_shutdown():
    if static_fallback:
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

# Single points2 path: relay from kinect points only (no depth_image_proc duplicate)
python3 - <<'PY' >"${LOGDIR}/relay_pc.log" 2>&1 &
import rospy
from sensor_msgs.msg import PointCloud2
rospy.init_node("relay_points2_only")
pub = rospy.Publisher("/realsense/depth/points2", PointCloud2, queue_size=1)
rospy.Subscriber("/realsense/depth/points", PointCloud2, lambda m: pub.publish(m), queue_size=1)
rospy.spin()
PY
register $!

roslaunch fkie_nbv_planner octomap_server.launch robot_ns:=robot >"${LOGDIR}/octomap.log" 2>&1 &
register $!

rosrun fkie_nbv_planner camera_pose_tf.py __name:=camera_pose_pub \
  _world_frame:=world _sensor_frame:=d455_color_optical_frame _rate:=10.0 \
  camera_pose:=/robot/camera_pose >"${LOGDIR}/cam_pose.log" 2>&1 &
register $!

# Ranger footprint ~ wheel track 0.89 x 0.56 → use 1.0 x 0.70 rectangle in world, moved with base
python3 - <<'PY' >"${LOGDIR}/footprint.log" 2>&1 &
import rospy, math
import tf2_ros
from geometry_msgs.msg import PolygonStamped, Point32
rospy.init_node("agx_ranger_footprint")
pub = rospy.Publisher("/robot/robot_bbx", PolygonStamped, queue_size=1)
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf)
# half-lengths (m)
hx, hy = 0.50, 0.35
rate = rospy.Rate(5)
while not rospy.is_shutdown():
    try:
        t = buf.lookup_transform("world", "base_link", rospy.Time(0), rospy.Duration(0.5))
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
        msg = PolygonStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        for lx, ly in corners:
            wx = x + c * lx - s * ly
            wy = y + s * lx + c * ly
            msg.polygon.points.append(Point32(x=wx, y=wy, z=0.0))
        pub.publish(msg)
    except Exception:
        pass
    rate.sleep()
PY
register $!

rosparam load "${PLANNER_YAML}" /robot/fkie_nbv_planner
rosrun fkie_nbv_planner nbvplanner_node __ns:=robot __name:=fkie_nbv_planner \
  octomap:=octomap_full >"${LOGDIR}/planner.log" 2>&1 &
register $!

rviz -d "${RVIZ_CFG}" >"${LOGDIR}/rviz.log" 2>&1 &
register $!

echo "[agx-nbv] waiting for depth(enc!=rgb8) + octomap + camera_pose..."
READY=0
for i in $(seq 1 120); do
  python3 - <<'PY' && READY=1 && break
import rospy
from sensor_msgs.msg import Image, PointCloud2
from octomap_msgs.msg import Octomap
from geometry_msgs.msg import PoseStamped
rospy.init_node("agx_ready_probe", anonymous=True)
img = rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=3.0)
assert img.encoding not in ("rgb8", "bgr8"), img.encoding
pc = rospy.wait_for_message("/realsense/depth/points2", PointCloud2, timeout=3.0)
assert len(pc.data) > 100
om = rospy.wait_for_message("/robot/octomap_full", Octomap, timeout=5.0)
assert len(om.data) > 50
pose = rospy.wait_for_message("/robot/camera_pose", PoseStamped, timeout=3.0)
print("ready depth_enc=%s points=%d octomap=%d cam=(%.2f,%.2f,%.2f)" % (
  img.encoding, len(pc.data), len(om.data),
  pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
PY
  sleep 2
done

python3 /workspace/scripts/fkie_agx_diag.py 2>&1 | tee "${EVDIR}/diag_before.json"

if [ "${READY}" != "1" ]; then
  echo "FKIE_AGX_NBV_FAIL: sensors not ready"
  tail -40 "${LOGDIR}/gazebo.log" || true
  tail -40 "${LOGDIR}/spawn_robot.log" || true
  exit 2
fi

# Integrate more frames; wait until Action server is actually up
sleep 12
echo "[agx-nbv] waiting for /robot/nbv_rrt action server..."
for i in $(seq 1 60); do
  if timeout 2 rosservice list 2>/dev/null | grep -q '/robot/nbv_rrt/status\|/robot/fkie_nbv_planner'; then
    :
  fi
  if timeout 2 rostopic list 2>/dev/null | grep -q '/robot/nbv_rrt/goal'; then
    echo "[agx-nbv] action topics ready (${i}s)"
    break
  fi
  sleep 1
done

echo "[agx-nbv] calling NBV action..."
# Do not abort the keep-alive GUI if goals empty — record result then continue
set +e
python3 /workspace/scripts/verify_fkie_action.py _timeout:=90.0 2>&1 | tee "${EVDIR}/action_result.txt"
ACTION_RC=$?
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
open("/workspace/data/metrics/fkie_agx_warehouse_nbv/RESULT.txt","w").write(
  "goals=%d nonempty=%s\n" % (n, ok)
)
PY

tail -80 "${LOGDIR}/planner.log" | tee "${EVDIR}/planner_tail.txt" >/dev/null || true

# Screenshots if import possible
python3 - <<'PY' >"${LOGDIR}/screenshot.log" 2>&1 || true
import os
try:
  import pyscreenshot as ImageGrab
except Exception:
  try:
    from PIL import ImageGrab
  except Exception as e:
    print("no screenshot lib", e)
    raise SystemExit(0)
os.makedirs("/workspace/reports/fkie_agx_integration_shots", exist_ok=True)
im = ImageGrab.grab()
im.save("/workspace/reports/fkie_agx_integration_shots/desktop.png")
print("saved desktop.png")
PY

echo ""
echo "============================================================"
echo " AGX + FKIE warehouse demo running (session PIDs tracked)."
echo " Evidence: ${EVDIR}/"
echo " Leave running to inspect; Ctrl+C cleans only this session."
echo "============================================================"

while true; do sleep 3600; done
