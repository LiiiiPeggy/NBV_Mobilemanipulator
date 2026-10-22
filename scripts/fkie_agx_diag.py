#!/usr/bin/env python3
"""FKIE+AGX warehouse diagnostics (no cheat maps)."""
from __future__ import print_function
import json
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from octomap_msgs.msg import Octomap
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from nav_msgs.msg import Odometry


def wait_msg(topic, typ, timeout=8.0):
    try:
        return rospy.wait_for_message(topic, typ, timeout=timeout)
    except Exception as e:
        return "ERR:%s" % e


def main():
    rospy.init_node("fkie_agx_diag", anonymous=True)
    out = {"ok": True}
    buf = tf2_ros.Buffer(cache_time=rospy.Duration(10))
    listener = tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    img = wait_msg("/realsense/depth/image_raw", Image, 6)
    if isinstance(img, str):
        out["depth"] = img
        out["ok"] = False
    else:
        out["depth"] = {
            "enc": img.encoding,
            "wh": [img.width, img.height],
            "frame": img.header.frame_id,
        }
        if img.encoding in ("rgb8", "bgr8"):
            out["ok"] = False
            out["depth_bad_encoding"] = True

    rgb = wait_msg("/realsense/rgb/image_raw", Image, 4)
    out["rgb"] = (
        {"enc": rgb.encoding, "wh": [rgb.width, rgb.height]}
        if not isinstance(rgb, str)
        else rgb
    )

    info = wait_msg("/realsense/depth/camera_info", CameraInfo, 4)
    out["camera_info"] = (
        {"frame": info.header.frame_id, "K": list(info.K)}
        if not isinstance(info, str)
        else info
    )

    pc = wait_msg("/realsense/depth/points2", PointCloud2, 6)
    if isinstance(pc, str):
        out["points2"] = pc
        out["ok"] = False
    else:
        out["points2"] = {
            "frame": pc.header.frame_id,
            "wh": [pc.width, pc.height],
            "data_len": len(pc.data),
        }

    om = wait_msg("/robot/octomap_full", Octomap, 10)
    if isinstance(om, str):
        out["octomap"] = om
        out["ok"] = False
    else:
        out["octomap"] = {
            "frame": om.header.frame_id,
            "res": om.resolution,
            "data_len": len(om.data),
            "id": om.id,
        }

    frames = [
        ("world", "base_link"),
        ("world", "d455_depth_optical_frame"),
        ("world", "d455_color_optical_frame"),
        ("world", "turret_link"),
        ("world", "robot/base_link"),
    ]
    out["tf"] = {}
    for a, b in frames:
        try:
            t = buf.lookup_transform(a, b, rospy.Time(0), rospy.Duration(2.0))
            p = t.transform.translation
            out["tf"]["%s->%s" % (a, b)] = [round(p.x, 3), round(p.y, 3), round(p.z, 3)]
        except Exception as e:
            out["tf"]["%s->%s" % (a, b)] = "ERR:%s" % e
            out["ok"] = False

    # publishers on points2
    try:
        import subprocess

        r = subprocess.check_output(
            ["rostopic", "info", "/realsense/depth/points2"],
            stderr=subprocess.STDOUT,
            universal_newlines=True,
        )
        pubs = [ln.strip() for ln in r.splitlines() if "Publishers:" in r]
        out["points2_info"] = r
    except Exception as e:
        out["points2_info"] = str(e)

    print(json.dumps(out, indent=2))
    return 0 if out.get("ok") else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(1)
