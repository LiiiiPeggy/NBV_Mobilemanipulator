#!/usr/bin/env python3
"""Capture D455 RGB/Depth and optional Gazebo model inventory snapshot for YCB demo."""
from __future__ import print_function
import argparse
import json
from pathlib import Path

import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetWorldProperties, GetModelState


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--prefix", default="mo_")
    args = ap.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    rospy.init_node("capture_ycb_scene_evidence", anonymous=True)
    bridge = CvBridge()
    rgb = rospy.wait_for_message("/realsense/rgb/image_raw", Image, timeout=30)
    depth = rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=30)
    info = rospy.wait_for_message("/realsense/depth/camera_info", CameraInfo, timeout=10)
    bgr = bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
    depth_img = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")

    import cv2

    cv2.imwrite(str(out / ("d455_rgb_seed%d.png" % args.seed)), bgr)
    d = depth_img.astype(np.float32)
    if d.ndim == 3:
        d = d[:, :, 0]
    finite = d[np.isfinite(d)]
    if finite.size and np.nanmedian(finite) > 50:
        d = d * 0.001
    d_viz = np.clip(d / 4.0, 0, 1)
    d_u8 = (d_viz * 255).astype(np.uint8)
    d_color = cv2.applyColorMap(d_u8, cv2.COLORMAP_JET)
    cv2.imwrite(str(out / ("d455_depth_seed%d.png" % args.seed)), d_color)

    inv = {"camera_info_frame": info.header.frame_id, "models": []}
    try:
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=5)
        names = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)().model_names
        get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        for n in sorted(names):
            if not n.startswith(args.prefix) and n != "agx_ranger_cr10":
                continue
            st = get(n, "world")
            if not st.success:
                continue
            p = st.pose.position
            inv["models"].append({"name": n, "xyz": [float(p.x), float(p.y), float(p.z)]})
    except Exception as e:
        inv["error"] = str(e)
    (out / ("gazebo_models_seed%d.json" % args.seed)).write_text(json.dumps(inv, indent=2))
    print("Wrote", out)
    print("CAPTURE_YCB_SCENE_EVIDENCE_OK")


if __name__ == "__main__":
    main()
