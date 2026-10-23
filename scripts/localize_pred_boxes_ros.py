#!/usr/bin/env python3
"""Localize 2D boxes from a pred JSON using live ROS RGB-D + TF (runs in nbv_fkie)."""
from __future__ import print_function
import argparse
import json
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf.transformations as tft


def depth_to_m(d):
    d = d.astype(np.float32)
    if d.ndim == 3:
        d = d[:, :, 0]
    finite = d[np.isfinite(d)]
    if finite.size and np.nanmedian(finite) > 50:
        d = d * 0.001
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pred-in", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    pred = json.loads(Path(args.pred_in).read_text())
    rospy.init_node("localize_pred_boxes_ros", anonymous=True)
    bridge = CvBridge()
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(0.5)
    depth = rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=20)
    info = rospy.wait_for_message("/realsense/depth/camera_info", CameraInfo, timeout=10)
    depth_img = depth_to_m(bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough"))
    K = np.array(info.K, dtype=np.float64).reshape(3, 3)
    frame = info.header.frame_id or "d455_depth_optical_frame"
    tfm = buf.lookup_transform("world", frame, rospy.Time(0), rospy.Duration(2.0))
    t = tfm.transform.translation
    r = tfm.transform.rotation
    T = tft.quaternion_matrix([r.x, r.y, r.z, r.w])
    T[0:3, 3] = [t.x, t.y, t.z]

    h, w = depth_img.shape[:2]
    for view in pred.get("views", []):
        objs = []
        for o in view.get("objects", []):
            x1, y1, x2, y2 = [int(v) for v in o["bbox_xyxy"]]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w - 1, x2), min(h - 1, y2)
            patch = depth_img[y1:y2, x1:x2]
            near = np.isfinite(patch) & (patch > 0.2) & (patch < 5.0)
            if near.mean() < 0.05 or near.sum() < 20:
                continue
            z = float(np.median(patch[near]))
            ys, xs = np.where(near & (np.abs(patch - z) < 0.3))
            if xs.size < 15:
                continue
            u = float(np.median(xs) + x1)
            v = float(np.median(ys) + y1)
            x = (u - K[0, 2]) * z / K[0, 0]
            y = (v - K[1, 2]) * z / K[1, 1]
            pw = T.dot(np.array([x, y, z, 1.0]))[:3]
            objs.append(
                {
                    "class_name": o["class_name"],
                    "confidence": o.get("confidence", 0.0),
                    "bbox_xyxy": o["bbox_xyxy"],
                    "center_cam": [x, y, z],
                    "center_world": pw.tolist(),
                    "optical_frame": frame,
                }
            )
        view["objects"] = objs
        view["n_localized"] = len(objs)
        view["n_detections"] = len(objs)
        view["backend"] = "grounded_sam+ros_localize"
    Path(args.out).write_text(json.dumps(pred, indent=2))
    print("Wrote", args.out, "n", sum(len(v["objects"]) for v in pred["views"]))
    print("LOCALIZE_PRED_BOXES_ROS_OK")


if __name__ == "__main__":
    main()
