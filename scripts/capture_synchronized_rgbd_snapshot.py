#!/usr/bin/env python3
"""Capture one synchronized RGB-D + CameraInfo + stamp TF snapshot.

Does NOT run GSAM. Perception must consume this snapshot directory only.
Uses ApproximateTimeSynchronizer when message_filters is available.
"""
from __future__ import print_function

import argparse
import json
import time
import uuid
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

try:
    import message_filters
except Exception:
    message_filters = None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True, help="observation_XXX directory")
    ap.add_argument("--snapshot-id", default="")
    ap.add_argument("--slop", type=float, default=0.05)
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--rgb-topic", default="/realsense/rgb/image_raw")
    ap.add_argument("--depth-topic", default="/realsense/depth/image_raw")
    ap.add_argument("--info-topic", default="/realsense/rgb/camera_info")
    ap.add_argument("--optical-frame", default="d455_depth_optical_frame")
    ap.add_argument("--world-frame", default="world")
    ap.add_argument("--min-stamp-after", type=float, default=0.0,
                    help="Reject frames with rgb stamp <= this (sim seconds)")
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    snap_id = args.snapshot_id or ("snap_" + uuid.uuid4().hex[:10])

    rospy.init_node("capture_synchronized_rgbd_snapshot", anonymous=True)
    buf = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
    tf2_ros.TransformListener(buf)
    bridge = CvBridge()

    rgb_msg = depth_msg = info_msg = None
    sync_dt = None

    if message_filters is not None:
        rgb_sub = message_filters.Subscriber(args.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(args.depth_topic, Image)
        info_sub = message_filters.Subscriber(args.info_topic, CameraInfo)
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=10, slop=args.slop
        )
        box = {"done": False, "pack": None}

        def _cb(rgb, depth, info):
            if box["done"]:
                return
            if args.min_stamp_after > 0 and rgb.header.stamp.to_sec() <= args.min_stamp_after:
                return
            box["pack"] = (rgb, depth, info)
            box["done"] = True

        sync.registerCallback(_cb)
        t0 = time.time()
        while not box["done"] and (time.time() - t0) < args.timeout and not rospy.is_shutdown():
            rospy.sleep(0.05)
        if not box["done"]:
            raise SystemExit("SYNC_FAIL: timeout waiting ApproximateTimeSynchronizer")
        rgb_msg, depth_msg, info_msg = box["pack"]
        sync_dt = abs(rgb_msg.header.stamp.to_sec() - depth_msg.header.stamp.to_sec())
    else:
        # Fallback: sequential wait (still record stamp deltas)
        rgb_msg = rospy.wait_for_message(args.rgb_topic, Image, timeout=args.timeout)
        depth_msg = rospy.wait_for_message(args.depth_topic, Image, timeout=args.timeout)
        info_msg = rospy.wait_for_message(args.info_topic, CameraInfo, timeout=args.timeout)
        if args.min_stamp_after > 0 and rgb_msg.header.stamp.to_sec() <= args.min_stamp_after:
            raise SystemExit("SYNC_FAIL: rgb stamp not after min_stamp_after")
        sync_dt = abs(rgb_msg.header.stamp.to_sec() - depth_msg.header.stamp.to_sec())
        if sync_dt > args.slop * 4:
            raise SystemExit("SYNC_FAIL: rgb/depth stamp delta %.3f > %.3f" % (sync_dt, args.slop * 4))

    stamp = rgb_msg.header.stamp
    tfm = None
    tf_err = None
    for wait in (0.5, 1.0, 2.0):
        try:
            tfm = buf.lookup_transform(
                args.world_frame, args.optical_frame, stamp, rospy.Duration(wait)
            )
            break
        except Exception as e:
            tf_err = e
            rospy.sleep(0.05)
    if tfm is None:
        # Last resort: latest TF (record that stamp match failed)
        try:
            tfm = buf.lookup_transform(
                args.world_frame, args.optical_frame, rospy.Time(0), rospy.Duration(2.0)
            )
            tf_err = "fallback_Time0_after: %s" % tf_err
        except Exception as e:
            raise SystemExit("TF_FAIL: optical TF at rgb stamp: %s" % e)

    import cv2

    bgr = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    depth_np = np.asarray(depth)
    if depth_np.dtype == np.uint16:
        depth_m = depth_np.astype(np.float32) * 0.001
    else:
        depth_m = depth_np.astype(np.float32)
        finite = depth_m[np.isfinite(depth_m)]
        if finite.size and np.nanmedian(finite) > 50:
            depth_m = depth_m * 0.001

    rgb_path = out / "rgb.png"
    depth_path = out / "depth.npy"
    depth_viz = out / "depth_viz.png"
    cv2.imwrite(str(rgb_path), bgr)
    np.save(str(depth_path), depth_m)
    dviz = np.clip(depth_m / 5.0, 0, 1)
    cv2.imwrite(str(depth_viz), (dviz * 255).astype(np.uint8))

    t = tfm.transform.translation
    r = tfm.transform.rotation
    cam_pose = {
        "frame_id": args.world_frame,
        "child_frame_id": args.optical_frame,
        "xyz": [float(t.x), float(t.y), float(t.z)],
        "quat_xyzw": [float(r.x), float(r.y), float(r.z), float(r.w)],
        "stamp": stamp.to_sec(),
        "tf_lookup_stamp": stamp.to_sec(),
        "tf_note": None if tf_err is None or not isinstance(tf_err, str) else tf_err,
    }
    cam_info = {
        "frame_id": info_msg.header.frame_id,
        "stamp": info_msg.header.stamp.to_sec() if info_msg.header.stamp else None,
        "height": int(info_msg.height),
        "width": int(info_msg.width),
        "K": list(info_msg.K),
        "D": list(info_msg.D),
        "P": list(info_msg.P),
        "distortion_model": info_msg.distortion_model,
    }
    (out / "camera_pose.json").write_text(json.dumps(cam_pose, indent=2))
    (out / "camera_info.json").write_text(json.dumps(cam_info, indent=2))

    meta = {
        "snapshot_id": snap_id,
        "status": "ok",
        "wall_time": time.time(),
        "sim_time": rospy.Time.now().to_sec(),
        "rgb_topic": args.rgb_topic,
        "depth_topic": args.depth_topic,
        "info_topic": args.info_topic,
        "rgb_stamp": stamp.to_sec(),
        "depth_stamp": depth_msg.header.stamp.to_sec(),
        "info_stamp": info_msg.header.stamp.to_sec() if info_msg.header.stamp else None,
        "sync_dt_s": float(sync_dt) if sync_dt is not None else None,
        "sync_slop_s": float(args.slop),
        "sync_method": "ApproximateTimeSynchronizer" if message_filters else "sequential",
        "min_stamp_after": float(args.min_stamp_after),
        "rgb_path": str(rgb_path),
        "depth_path": str(depth_path),
        "camera_pose_path": str(out / "camera_pose.json"),
        "camera_info_path": str(out / "camera_info.json"),
        "depth_valid_ratio": float(
            np.mean(np.isfinite(depth_m) & (depth_m > 0.2) & (depth_m < 5.0))
        ),
    }
    if meta["depth_valid_ratio"] < 0.01:
        meta["status"] = "depth_invalid"
        (out / "meta.json").write_text(json.dumps(meta, indent=2))
        print(json.dumps(meta, indent=2))
        raise SystemExit("DEPTH_INVALID")
    (out / "meta.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, indent=2))
    print("SNAPSHOT_SYNC_OK", snap_id)


if __name__ == "__main__":
    main()
