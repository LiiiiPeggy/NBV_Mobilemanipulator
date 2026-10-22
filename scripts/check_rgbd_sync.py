#!/usr/bin/env python3
"""Check RGB / Depth / CameraInfo timestamp and resolution consistency."""
from __future__ import print_function
import argparse
import json
from pathlib import Path

import rospy
from sensor_msgs.msg import Image, CameraInfo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rgb", default="/realsense/rgb/image_raw")
    ap.add_argument("--depth", default="/realsense/depth/image_raw")
    ap.add_argument("--info", default="/realsense/depth/camera_info")
    ap.add_argument("--out", required=True)
    ap.add_argument("--samples", type=int, default=5)
    args = ap.parse_args()

    rospy.init_node("check_rgbd_sync")
    pairs = []
    for _ in range(args.samples):
        rgb = rospy.wait_for_message(args.rgb, Image, timeout=10.0)
        depth = rospy.wait_for_message(args.depth, Image, timeout=10.0)
        info = rospy.wait_for_message(args.info, CameraInfo, timeout=10.0)
        dt_rd = abs((rgb.header.stamp - depth.header.stamp).to_sec())
        dt_di = abs((depth.header.stamp - info.header.stamp).to_sec())
        pairs.append(
            {
                "rgb_stamp": rgb.header.stamp.to_sec(),
                "depth_stamp": depth.header.stamp.to_sec(),
                "info_stamp": info.header.stamp.to_sec(),
                "dt_rgb_depth_s": dt_rd,
                "dt_depth_info_s": dt_di,
                "rgb_wh": [rgb.width, rgb.height],
                "depth_wh": [depth.width, depth.height],
                "info_wh": [info.width, info.height],
                "depth_encoding": depth.encoding,
                "rgb_encoding": rgb.encoding,
                "rgb_frame": rgb.header.frame_id,
                "depth_frame": depth.header.frame_id,
                "info_frame": info.header.frame_id,
            }
        )
    dts = [p["dt_rgb_depth_s"] for p in pairs]
    res_ok = all(
        p["rgb_wh"] == p["depth_wh"] == p["info_wh"] for p in pairs
    )
    out = {
        "samples": pairs,
        "max_dt_rgb_depth_s": max(dts) if dts else None,
        "mean_dt_rgb_depth_s": sum(dts) / len(dts) if dts else None,
        "resolution_match": res_ok,
        "sync_ok": bool(dts) and max(dts) < 0.05 and res_ok,
        "note": "Gazebo OpenNI plugin publishes RGB+Depth from same sensor; still verify encodings.",
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps({k: out[k] for k in ("max_dt_rgb_depth_s", "resolution_match", "sync_ok")}, indent=2))
    print("CHECK_RGBD_SYNC_OK" if out["sync_ok"] else "CHECK_RGBD_SYNC_WARN")


if __name__ == "__main__":
    main()
