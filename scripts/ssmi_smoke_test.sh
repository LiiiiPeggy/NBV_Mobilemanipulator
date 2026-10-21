#!/usr/bin/env bash
# Launch SSMI demo components and verify depth/semantic/map topics.
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /workspace/third_party/SSMI-Example/ssmi_ws/devel/setup.bash
export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
export IGNITION_VERSION=fortress

LOGDIR=/workspace/logs/ssmi_smoke
mkdir -p "${LOGDIR}"

cleanup() {
  set +e
  # Kill ROS nodes carefully; avoid matching this script via broad patterns.
  if [ -n "${ROS_PID:-}" ]; then kill "${ROS_PID}" 2>/dev/null || true; fi
  rosnode kill -a >/dev/null 2>&1 || true
  kill $(jobs -p) >/dev/null 2>&1 || true
  sleep 2
}
trap cleanup EXIT

roscore >"${LOGDIR}/roscore.log" 2>&1 &
sleep 3

echo "[ssmi] launching seg_husky.launch"
roslaunch semantic_segmentation_husky seg_husky.launch >"${LOGDIR}/sim.log" 2>&1 &
SIM_PID=$!

# Wait up to ~3 minutes for camera topics
READY=0
for i in $(seq 1 90); do
  if rostopic list 2>/dev/null | grep -Eiq 'depth|semantic|camera'; then
    READY=1
    echo "[ssmi] camera-related topics appeared at ${i}*2s"
    break
  fi
  if ! kill -0 "${SIM_PID}" 2>/dev/null; then
    echo "[ssmi] sim process exited early"
    tail -80 "${LOGDIR}/sim.log" || true
    break
  fi
  sleep 2
done

rostopic list 2>/dev/null | tee "${LOGDIR}/topics_after_sim.txt" || true
tail -60 "${LOGDIR}/sim.log" | tee "${LOGDIR}/sim_tail.txt" || true

if [ "${READY}" != "1" ]; then
  echo "SSMI_SIM_TOPICS_MISSING"
  exit 2
fi

echo "[ssmi] launching semantic_octomap"
roslaunch semantic_octomap semantic_octomap.launch >"${LOGDIR}/map.log" 2>&1 &
sleep 15

echo "[ssmi] launching planner"
roslaunch semantic_info_gathering run_semantic_exploration.launch >"${LOGDIR}/plan.log" 2>&1 &
sleep 15

rostopic list | tee "${LOGDIR}/topics_all.txt"

# Sample key topics if present
python3 - <<'PY' | tee "${LOGDIR}/topic_hz_sample.txt"
import rospy, time, sys
from sensor_msgs.msg import Image, PointCloud2
rospy.init_node("ssmi_topic_probe", anonymous=True)

candidates = [
    ("/camera/depth/image_raw", Image),
    ("/camera/depth/points", PointCloud2),
    ("/semantic_image", Image),
    ("/semantic_pointcloud", PointCloud2),
    ("/sem_octomap", None),
    ("/occupied_cells_vis_array", None),
]

# discover matching topics
topics = dict(rospy.get_published_topics())
print("published_topics=%d" % len(topics))
for name in sorted(topics):
    if any(k in name.lower() for k in ("depth", "semantic", "octomap", "camera", "seg", "rgb")):
        print("TOPIC", name, topics[name])

hits = []
for name, _ in candidates:
    for t in topics:
        if name in t or t.endswith(name):
            hits.append(t)
print("CANDIDATE_HITS", hits)

# measure a few messages on image-like topics
from rospy import AnyMsg
checked = 0
for tname, ttype in topics.items():
    low = tname.lower()
    if not any(k in low for k in ("depth", "semantic", "seg", "rgb", "image")):
        continue
    if "Image" not in ttype and "PointCloud" not in ttype:
        continue
    counted = {"n": 0}
    def cb(msg, c=counted):
        c["n"] += 1
    sub = rospy.Subscriber(tname, AnyMsg, cb)
    t0 = time.time()
    while time.time() - t0 < 5.0 and counted["n"] < 3:
        rospy.sleep(0.1)
    sub.unregister()
    print("RECV", tname, "count=", counted["n"], "type=", ttype)
    checked += 1
    if checked >= 8:
        break
print("SSMI_TOPIC_PROBE_DONE")
PY

echo "SSMI_SMOKE_FINISHED"
