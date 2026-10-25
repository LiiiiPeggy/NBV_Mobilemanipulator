#!/usr/bin/env python3
"""Capture pre/post NBV loop snapshot: RGB-D, TF, OctoMap meta, optional GSAM+memory."""
from __future__ import print_function

import argparse
import json
import subprocess
import time
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import OccupancyGrid


def lookup(buf, parent, child):
    t = buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(2.0))
    p = t.transform.translation
    q = t.transform.rotation
    return {
        "xyz": [p.x, p.y, p.z],
        "quat_xyzw": [q.x, q.y, q.z, q.w],
        "stamp": t.header.stamp.to_sec() if t.header.stamp else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--goal-id", default="")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--shot-dir", default="reports/fkie_remani_one_step_shots")
    ap.add_argument("--run-gsam", action="store_true", default=True)
    ap.add_argument("--no-gsam", action="store_true")
    args = ap.parse_args()
    run_gsam = bool(args.run_gsam) and not bool(args.no_gsam)

    out = Path(args.out_dir)
    shot = Path(args.shot_dir)
    out.mkdir(parents=True, exist_ok=True)
    shot.mkdir(parents=True, exist_ok=True)

    rospy.init_node("capture_nbv_loop_snapshot_%s" % args.tag, anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    bridge = CvBridge()
    rospy.sleep(0.5)

    # Mode B Gazebo D455 uses /realsense/rgb/* (not /color/*)
    rgb = None
    info = None
    for topic in ("/realsense/rgb/image_raw", "/realsense/color/image_raw"):
        try:
            rgb = rospy.wait_for_message(topic, Image, timeout=5)
            break
        except Exception:
            continue
    if rgb is None:
        raise SystemExit("No RGB topic (/realsense/rgb|color/image_raw)")
    depth = rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=15)
    for topic in ("/realsense/rgb/camera_info", "/realsense/color/camera_info"):
        try:
            info = rospy.wait_for_message(topic, CameraInfo, timeout=5)
            break
        except Exception:
            continue
    if info is None:
        raise SystemExit("No camera_info")

    rgb_np = bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
    depth_np = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")

    import cv2

    rgb_path = out / ("rgb_%s.png" % args.tag)
    depth_path = out / ("depth_%s.npy" % args.tag)
    depth_viz = out / ("depth_viz_%s.png" % args.tag)
    cv2.imwrite(str(rgb_path), rgb_np)
    np.save(str(depth_path), depth_np)
    d = depth_np.astype(np.float32)
    if np.nanmedian(d[np.isfinite(d)]) > 50:
        d = d * 0.001
    dv = np.clip(d / 5.0, 0, 1)
    cv2.imwrite(str(depth_viz), (dv * 255).astype(np.uint8))
    # copy to shot dir
    cv2.imwrite(str(shot / ("rgb_%s_%s.png" % (args.goal_id or "g", args.tag))), rgb_np)

    cam = lookup(buf, "world", "d455_depth_optical_frame")
    ee = lookup(buf, "world", "cr10_Link6")
    base = lookup(buf, "world", "base_link")

    octo_meta = {"available": False}
    try:
        # FKIE octomap binary topic may be large; record header only via latch if present
        from octomap_msgs.msg import Octomap

        om = rospy.wait_for_message("/octomap_binary", Octomap, timeout=3.0)
        octo_meta = {
            "available": True,
            "frame_id": om.header.frame_id,
            "stamp": om.header.stamp.to_sec() if om.header.stamp else None,
            "resolution": float(om.resolution),
            "id": om.id,
            "binary": bool(om.binary),
            "data_len": len(om.data) if om.data is not None else 0,
        }
    except Exception as e:
        octo_meta = {"available": False, "error": str(e)}

    meta = {
        "tag": args.tag,
        "goal_id": args.goal_id,
        "wall_time": time.time(),
        "sim_time": rospy.Time.now().to_sec(),
        "rgb_stamp": rgb.header.stamp.to_sec() if rgb.header.stamp else None,
        "depth_stamp": depth.header.stamp.to_sec() if depth.header.stamp else None,
        "camera_info_frame": info.header.frame_id,
        "K": list(info.K),
        "camera_tf": cam,
        "ee_tf": ee,
        "base_tf": base,
        "octomap": octo_meta,
        "rgb_path": str(rgb_path),
        "depth_path": str(depth_path),
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2))

    if run_gsam:
        perc_out = out / "perception.json"
        cmd = [
            "python3",
            "/workspace/scripts/perceive_rgbd_objects.py",
            "--backend",
            "grounded_sam",
            "--world-tf-mode",
            "tf",
            "--out",
            str(perc_out),
        ]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=240)
            meta["perception_rc"] = r.returncode
            meta["perception_tail"] = (r.stdout or "")[-800:]
            (out / "perceive_stdout.txt").write_text((r.stdout or "") + "\n" + (r.stderr or ""))
        except Exception as e:
            meta["perception_error"] = str(e)
        (out / "meta.json").write_text(json.dumps(meta, indent=2))

    print(json.dumps(meta, indent=2))
    print("SNAPSHOT_OK_%s" % args.tag.upper())


if __name__ == "__main__":
    main()
