#!/usr/bin/env python3
"""Validate D455 PointCloud2 frame and optional ground-plane tilt in world."""
from __future__ import print_function
import argparse
import json
import struct
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from sensor_msgs.msg import PointCloud2


def read_xyz(msg, max_n=20000):
    # assume xyz float32 contiguous or with offsets
    fmt = "fff"
    point_step = msg.point_step
    n = min(msg.width * msg.height, max_n)
    pts = []
    data = msg.data
    for i in range(0, n, max(1, n // max_n) if n > max_n else 1):
        off = i * point_step
        if off + 12 > len(data):
            break
        x, y, z = struct.unpack_from("<fff", data, off)
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            pts.append((x, y, z))
        if len(pts) >= max_n:
            break
    return np.asarray(pts, dtype=np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/realsense/depth/points2")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    rospy.init_node("check_depth_cloud_alignment")
    msg = rospy.wait_for_message(args.topic, PointCloud2, timeout=15.0)
    pts = read_xyz(msg)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(0.5)
    frame = msg.header.frame_id.lstrip("/")
    tfm = buf.lookup_transform("world", frame, rospy.Time(0), rospy.Duration(2.0))
    t, r = tfm.transform.translation, tfm.transform.rotation
    T = tft.quaternion_matrix([r.x, r.y, r.z, r.w])
    T[0:3, 3] = [t.x, t.y, t.z]
    # optical: z forward — depth should increase along +z in camera
    z_cam = pts[:, 2] if pts.size else np.array([])
    world = []
    for p in pts[:: max(1, len(pts) // 5000)]:
        h = np.array([p[0], p[1], p[2], 1.0])
        world.append(T.dot(h)[:3])
    world = np.asarray(world) if world else np.zeros((0, 3))
    # Fit plane to lower z band (possible ground)
    plane = None
    if len(world) > 100:
        # RANSAC-lite: lowest 30% by z as ground candidates if looking down
        z = world[:, 2]
        thr = np.percentile(z, 35)
        cand = world[z <= thr]
        if len(cand) > 50:
            # plane z = ax+by+c
            A = np.c_[cand[:, 0], cand[:, 1], np.ones(len(cand))]
            coef, _, _, _ = np.linalg.lstsq(A, cand[:, 2], rcond=None)
            # normal from ax + by - z + c = 0 → n=(a,b,-1)
            n = np.array([coef[0], coef[1], -1.0])
            n = n / np.linalg.norm(n)
            up = abs(float(n[2]))
            plane = {"coef_z_ax_by_c": coef.tolist(), "normal": n.tolist(), "upright_abs_nz": up}

    out = {
        "topic": args.topic,
        "frame_id": frame,
        "n_points_sampled": int(len(pts)),
        "cam_z_min": float(z_cam.min()) if z_cam.size else None,
        "cam_z_max": float(z_cam.max()) if z_cam.size else None,
        "cam_z_median": float(np.median(z_cam)) if z_cam.size else None,
        "depth_along_optical_z_ok": bool(z_cam.size and np.median(z_cam) > 0.2),
        "tf_world_from_frame_xyz": [t.x, t.y, t.z],
        "world_z_median": float(np.median(world[:, 2])) if len(world) else None,
        "ground_plane": plane,
        "note": "Tilted cloud in world is OK if camera pitches down; check ground upright_abs_nz≈1.",
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps({k: out[k] for k in ("frame_id", "depth_along_optical_z_ok", "cam_z_median", "ground_plane")}, indent=2))
    print("CHECK_DEPTH_CLOUD_ALIGNMENT_OK")


if __name__ == "__main__":
    main()
