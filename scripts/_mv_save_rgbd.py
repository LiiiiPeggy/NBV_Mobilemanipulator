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
