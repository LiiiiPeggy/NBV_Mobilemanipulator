#!/usr/bin/env bash
# Mode A offline multiview: rebake arm per view, spawn same layout, GSAM+localize+memory.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SEED="${SEED:-42}"
LAYOUT="data/scenes/indoor_objects/ycb/layouts/seed${SEED}.json"
OUT="reports/multiview_ycb/seed${SEED}"
CONTAINER="${FKIE_CONTAINER:-nbv_fkie}"
PY_GSAM="${NBV_PERCEPTION_PYTHON:-$ROOT/.conda_envs/nbv_perception/bin/python}"
mkdir -p "${ROOT}/${OUT}"
chmod +x "${ROOT}/scripts/run_multiview_ycb_capture.sh" 2>/dev/null || true

bringup_view() {
  local pose="$1"
  echo "[multiview] bringup pose=${pose}"
  docker exec "${CONTAINER}" bash -lc 'STOP_SIM_KEEP_PID=$$ bash /workspace/scripts/fkie_stop_sim.sh || true; sleep 1; pkill -9 gzserver 2>/dev/null || true; pkill -9 gzclient 2>/dev/null || true; sleep 1' || true
  if docker exec "${CONTAINER}" bash -lc 'pgrep gzserver >/dev/null'; then
    docker restart "${CONTAINER}"; sleep 8
  fi
  docker exec -d "${CONTAINER}" bash -lc "source /opt/ros/noetic/setup.bash; source /workspace/ws_fkie/devel/setup.bash; \
    export SPAWN_PROFILE=aisle_safe_ycb AGX_MODE=static SKIP_NBV_ACTION=1 NBV_OCTOMAP_SETTLE_S=6 AGX_ARM_POSE=${pose}; \
    bash /workspace/scripts/fkie_agx_warehouse_nbv.sh > /workspace/${OUT}/bringup.log 2>&1"
  for i in $(seq 1 90); do
    if docker exec "${CONTAINER}" bash -lc 'source /opt/ros/noetic/setup.bash; rosservice list 2>/dev/null | grep -q spawn_sdf_model'; then
      echo "gazebo_ready_${i}"; return 0
    fi
    sleep 2
  done
  echo "gazebo_timeout"; return 1
}

capture_one() {
  local vid="$1" tag="$2" pose="$3"
  bringup_view "${pose}"
  docker exec "${CONTAINER}" bash -lc "source /opt/ros/noetic/setup.bash; source /workspace/ws_fkie/devel/setup.bash; \
    export GAZEBO_MODEL_PATH=/workspace/third_party/gazebo_objects/gazebo_ycb/models:/workspace/configs/fkie/agx_adapt/models:\${GAZEBO_MODEL_PATH}; \
    python3 /workspace/scripts/spawn_object_scene.py --layout /workspace/${LAYOUT} --out-json /workspace/${OUT}/spawned_view${vid}.json"
  docker exec "${CONTAINER}" bash -lc "source /opt/ros/noetic/setup.bash; python3 /workspace/scripts/_mv_save_rgbd.py ${vid} ${tag} /workspace/${OUT}"
  export NBV_ROOT="$ROOT" TRANSFORMERS_OFFLINE=1 HF_HOME="$ROOT/checkpoints"
  "$PY_GSAM" "$ROOT/scripts/perceive_saved_rgbd_gsam.py" \
    --rgb "${ROOT}/${OUT}/rgb_view${vid}.png" \
    --out "${ROOT}/${OUT}/pred2d_view${vid}.json" \
    --classes banana,apple,bottle,bowl,box,can,cup
  docker exec "${CONTAINER}" bash -lc "source /opt/ros/noetic/setup.bash; \
    python3 /workspace/scripts/localize_pred_boxes_ros.py \
      --pred-in /workspace/${OUT}/pred2d_view${vid}.json \
      --out /workspace/${OUT}/pred3d_view${vid}.json"
}

# helper saver
cat > "${ROOT}/scripts/_mv_save_rgbd.py" <<'PY'
#!/usr/bin/env python3
import sys, json, os
import numpy as np, cv2, rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros, tf.transformations as tft
vid, tag, out = int(sys.argv[1]), sys.argv[2], sys.argv[3]
os.makedirs(out, exist_ok=True)
rospy.init_node("mv_save", anonymous=True)
b = CvBridge()
rgb = rospy.wait_for_message("/realsense/rgb/image_raw", Image, timeout=20)
dep = rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=20)
info = rospy.wait_for_message("/realsense/depth/camera_info", CameraInfo, timeout=10)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf); rospy.sleep(1.0)
frame = info.header.frame_id or "d455_depth_optical_frame"
tfm = buf.lookup_transform("world", frame, rospy.Time(0), rospy.Duration(2))
T = tft.quaternion_matrix([tfm.transform.rotation.x, tfm.transform.rotation.y, tfm.transform.rotation.z, tfm.transform.rotation.w])
T[0:3, 3] = [tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z]
bgr = b.imgmsg_to_cv2(rgb, "bgr8")
cv2.imwrite("%s/rgb_view%d.png" % (out, vid), bgr)
np.save("%s/depth_view%d.npy" % (out, vid), b.imgmsg_to_cv2(dep, "passthrough"))
meta = {"view_id": vid, "tag": tag, "K": list(info.K), "frame": frame, "T_world_cam": T.tolist(), "camera_pose_xyz": T[0:3, 3].tolist()}
open("%s/cam_view%d.json" % (out, vid), "w").write(json.dumps(meta, indent=2))
print("saved_view", vid, bgr.shape)
PY

capture_one 0 front configs/fkie/agx_adapt/config/arm_observe_aisle_front.yaml
capture_one 1 high configs/fkie/agx_adapt/config/arm_observe_aisle_high.yaml
capture_one 2 oblique configs/fkie/agx_adapt/config/arm_observe_aisle_oblique.yaml

python3 "${ROOT}/scripts/fuse_multiview_memory.py" --seed "${SEED}" --out-dir "${ROOT}/${OUT}"
echo "MULTIVIEW_YCB_CAPTURE_OK"
