#!/usr/bin/env python3
"""Capture D455 RGB plus aim Gazebo GUI camera for top/side/close placement shots."""
from __future__ import print_function
import argparse
import json
import subprocess
from pathlib import Path

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def aim_gz(pos, target):
    cmd = [
        "gz",
        "camera",
        "-c",
        "gzclient_camera",
        "-p",
        "%.3f %.3f %.3f" % tuple(pos),
        "-t",
        "%.3f %.3f %.3f" % tuple(target),
    ]
    try:
        subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--table", nargs=3, type=float, default=[-3.9, -7.5, 0.74])
    args = ap.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    tx, ty, tz = args.table

    rospy.init_node("capture_placement_views", anonymous=True)
    bridge = CvBridge()
    try:
        msg = rospy.wait_for_message("/realsense/rgb/image_raw", Image, timeout=15)
        bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(str(out / ("d455_rgb_seed%d.png" % args.seed)), bgr)
        print("Wrote", out / ("d455_rgb_seed%d.png" % args.seed))
    except Exception as e:
        print("D455 capture failed:", e)

    views = {
        "top": ([tx, ty, 4.0], [tx, ty, 0.5]),
        "side": ([tx + 2.2, ty - 0.2, 1.4], [tx, ty, 0.7]),
        "close": ([tx + 0.9, ty - 0.9, 1.3], [tx, ty, 0.75]),
    }
    meta = {"seed": args.seed, "table": [tx, ty, tz], "views": {}}
    for name, (pos, tgt) in views.items():
        aim_gz(pos, tgt)
        rospy.sleep(0.8)
        meta["views"][name] = {"camera_pos": pos, "target": tgt}
        print("aimed", name, pos, "->", tgt)
    (out / ("placement_views_seed%d.json" % args.seed)).write_text(json.dumps(meta, indent=2))
    print("CAPTURE_PLACEMENT_VIEWS_OK")


if __name__ == "__main__":
    main()
