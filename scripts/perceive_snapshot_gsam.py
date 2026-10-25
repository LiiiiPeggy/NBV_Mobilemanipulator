#!/usr/bin/env python3
"""Run Grounded-SAM + 3D localization on a FIXED synchronized snapshot directory.

Never re-subscribes to live RGB/Depth. Failures are explicit status codes.
"""
from __future__ import print_function

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def backproject(u, v, z, K):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=np.float64)


def quat_xyzw_to_R(q):
    """Rotation matrix from quaternion xyzw (no rospy/tf dependency)."""
    x, y, z, w = [float(v) for v in q]
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def transform_cam_to_world(p_cam, xyz, quat_xyzw):
    R = quat_xyzw_to_R(quat_xyzw)
    return R.dot(np.asarray(p_cam, dtype=np.float64)) + np.asarray(xyz, dtype=np.float64)


def localize_mask(depth, K, xyxy, mask, cam_xyz, cam_quat):
    x1, y1, x2, y2 = [int(v) for v in xyxy]
    h, w = depth.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w - 1, x2), min(h - 1, y2)
    if x2 <= x1 or y2 <= y1:
        return None
    patch = depth[y1:y2, x1:x2]
    if mask is not None and mask.shape[:2] == depth.shape[:2]:
        near = mask[y1:y2, x1:x2].astype(bool) & np.isfinite(patch) & (patch > 0.25) & (patch < 4.5)
    else:
        near = np.isfinite(patch) & (patch > 0.25) & (patch < 4.5)
    if float(near.mean()) < 0.05:
        return None
    zs = patch[near]
    z = float(np.median(zs))
    zs = zs[np.abs(zs - z) < 0.25]
    if zs.size < 15:
        return None
    z = float(np.median(zs))
    ys, xs = np.where(near & (np.abs(patch - z) < 0.25))
    if xs.size < 15:
        return None
    u = float(np.median(xs) + x1)
    v = float(np.median(ys) + y1)
    p_cam = backproject(u, v, z, K)
    pw = transform_cam_to_world(p_cam, cam_xyz, cam_quat)
    if pw[2] < -0.05 or pw[2] > 3.5:
        return None
    return {
        "center_cam": p_cam.tolist(),
        "center_world": pw.tolist(),
        "valid_depth_ratio": float(near.mean()),
        "median_depth_m": z,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot-dir", required=True)
    ap.add_argument("--classes", default="banana,apple,bottle,bowl,box,can,cup,cracker,mustard")
    ap.add_argument("--device", default="")
    args = ap.parse_args()

    snap = Path(args.snapshot_dir)
    meta_path = snap / "meta.json"
    if not meta_path.is_file():
        raise SystemExit("INPUT_MISSING: meta.json")
    meta = json.loads(meta_path.read_text())
    snap_id = meta.get("snapshot_id", snap.name)
    rgb_path = snap / "rgb.png"
    depth_path = snap / "depth.npy"
    pose_path = snap / "camera_pose.json"
    info_path = snap / "camera_info.json"
    for p, tag in (
        (rgb_path, "rgb.png"),
        (depth_path, "depth.npy"),
        (pose_path, "camera_pose.json"),
        (info_path, "camera_info.json"),
    ):
        if not p.is_file():
            raise SystemExit("INPUT_MISSING: %s" % tag)

    bgr = cv2.imread(str(rgb_path))
    if bgr is None:
        raise SystemExit("INPUT_MISSING: failed to read rgb.png")
    depth = np.load(str(depth_path))
    pose = json.loads(pose_path.read_text())
    info = json.loads(info_path.read_text())
    K = np.array(info["K"], dtype=np.float64).reshape(3, 3)
    classes = [c.strip() for c in args.classes.split(",") if c.strip()]

    status = {
        "snapshot_id": snap_id,
        "status": "ok",
        "backend": "grounded_sam_snapshot",
        "n_det": 0,
        "n_localized": 0,
        "error": None,
        "elapsed_s": None,
    }
    t0 = time.time()
    try:
        from perception.rgbd_object_detector import GroundedSAMDetector

        kwargs = {}
        if args.device:
            kwargs["device"] = args.device
        det = GroundedSAMDetector(**kwargs)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = det.run(rgb, classes)
    except Exception as e:
        status["status"] = "model_fail"
        status["error"] = str(e)
        status["elapsed_s"] = time.time() - t0
        (snap / "detection.json").write_text(json.dumps({"status": status, "objects": []}, indent=2))
        (snap / "objects_3d.json").write_text(
            json.dumps({"status": status, "views": [{"view_id": 0, "objects": [], "n_detections": 0, "n_localized": 0}]}, indent=2)
        )
        print(json.dumps(status, indent=2))
        raise SystemExit("MODEL_FAIL")

    masks_dir = snap / "masks"
    masks_dir.mkdir(exist_ok=True)
    vis = bgr.copy()
    objs2d = []
    objs3d = []
    for i, cid in enumerate(res.class_id):
        cls = classes[int(cid)] if 0 <= int(cid) < len(classes) else "object"
        conf = float(res.confidence[i])
        box = [float(v) for v in res.xyxy[i]]
        mask = None if res.masks is None else res.masks[i]
        if mask is not None:
            np.save(str(masks_dir / ("mask_%02d.npy" % i)), mask.astype(np.uint8))
            m = mask.astype(bool)
            vis[m] = (0.55 * vis[m] + 0.45 * np.array([0, 0, 255])).astype(np.uint8)
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            vis,
            "%s %.2f" % (cls, conf),
            (x1, max(0, y1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
        )
        entry = {
            "class_name": cls,
            "confidence": conf,
            "bbox_xyxy": box,
            "snapshot_id": snap_id,
            "has_mask": mask is not None,
        }
        loc = localize_mask(depth, K, box, mask, pose["xyz"], pose["quat_xyzw"])
        if loc:
            entry.update(loc)
            objs3d.append(dict(entry))
        objs2d.append(entry)

    status["n_det"] = len(objs2d)
    status["n_localized"] = len(objs3d)
    status["elapsed_s"] = time.time() - t0
    if status["n_det"] == 0:
        status["status"] = "ok_empty"
    cv2.imwrite(str(snap / "gsam_vis.png"), vis)
    (snap / "detection.json").write_text(
        json.dumps({"status": status, "snapshot_id": snap_id, "objects": objs2d}, indent=2)
    )
    perc = {
        "status": status,
        "snapshot_id": snap_id,
        "backend": "grounded_sam_snapshot",
        "views": [
            {
                "view_id": 0,
                "snapshot_id": snap_id,
                "camera_pose_xyz": pose["xyz"],
                "n_detections": len(objs2d),
                "n_localized": len(objs3d),
                "objects": objs3d,
            }
        ],
    }
    (snap / "objects_3d.json").write_text(json.dumps(perc, indent=2))
    # Alias for evaluate_object_discovery / memory ingest
    (snap / "perception.json").write_text(json.dumps(perc, indent=2))
    print(json.dumps(status, indent=2))
    print("PERCEIVE_SNAPSHOT_GSAM_OK" if status["status"].startswith("ok") else "PERCEIVE_SNAPSHOT_GSAM_FAIL")


if __name__ == "__main__":
    main()
