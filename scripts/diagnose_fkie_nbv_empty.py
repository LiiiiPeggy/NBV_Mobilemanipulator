#!/usr/bin/env python3
"""Diagnose FKIE NBV empty goals under current camera/octomap/ROI (read-only)."""
from __future__ import print_function
import argparse
import json
import math
from pathlib import Path

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from octomap_msgs.msg import Octomap
from visualization_msgs.msg import MarkerArray


def point_in_poly(x, y, xs, ys):
    n = len(xs)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi, xj, yj = xs[i], ys[i], xs[j], ys[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--boundary-x", nargs=4, type=float, default=[-8.3603, -2.3603, -2.3603, -8.3603])
    ap.add_argument("--boundary-y", nargs=4, type=float, default=[-2.6852, -2.6852, -8.6852, -8.6852])
    ap.add_argument("--optical-frame", default="d455_depth_optical_frame")
    args = ap.parse_args()

    rospy.init_node("diagnose_fkie_nbv_empty", anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    out = {"optical_frame": args.optical_frame}
    try:
        cam = rospy.wait_for_message("/robot/camera_pose", PoseStamped, timeout=5.0)
        out["camera_pose"] = {
            "frame_id": cam.header.frame_id,
            "xyz": [cam.pose.position.x, cam.pose.position.y, cam.pose.position.z],
        }
        out["camera_in_boundary"] = point_in_poly(
            cam.pose.position.x, cam.pose.position.y, args.boundary_x, args.boundary_y
        )
    except Exception as e:
        out["camera_pose_error"] = str(e)

    try:
        tfm = buf.lookup_transform("world", args.optical_frame, rospy.Time(0), rospy.Duration(2.0))
        t = tfm.transform.translation
        out["tf_optical_xyz"] = [t.x, t.y, t.z]
        out["tf_optical_in_boundary"] = point_in_poly(t.x, t.y, args.boundary_x, args.boundary_y)
    except Exception as e:
        out["tf_optical_error"] = str(e)

    try:
        om = rospy.wait_for_message("/robot/octomap_full", Octomap, timeout=5.0)
        out["octomap"] = {"frame": om.header.frame_id, "id": om.id, "data_len": len(om.data), "res": om.resolution}
    except Exception as e:
        out["octomap_error"] = str(e)

    frontiers_n = None
    try:
        ma = rospy.wait_for_message("/robot/viz_curr_frontiers", MarkerArray, timeout=3.0)
        frontiers_n = len(ma.markers)
    except Exception:
        try:
            ma = rospy.wait_for_message("/robot/viz_frontiers", MarkerArray, timeout=2.0)
            frontiers_n = len(ma.markers)
        except Exception as e:
            out["frontiers_error"] = str(e)
    out["frontiers_marker_count"] = frontiers_n

    # Classify likely empty-goal cause (heuristic)
    cause = "unknown"
    if out.get("camera_pose_error") or out.get("tf_optical_error"):
        cause = "missing_camera_pose_or_tf"
    elif out.get("camera_in_boundary") is False and out.get("tf_optical_in_boundary") is False:
        cause = "camera_outside_roi"
    elif out.get("octomap", {}).get("data_len", 0) < 100:
        cause = "octomap_too_sparse"
    elif frontiers_n == 0:
        cause = "no_frontiers_in_roi"
    elif frontiers_n is not None and frontiers_n > 0:
        cause = "frontiers_exist_but_goals_filtered_or_rrt_failed"
    out["likely_cause"] = cause
    out["boundary_xy"] = {"x": args.boundary_x, "y": args.boundary_y}
    out["note"] = "Does not call NBV action; pair with verify_fkie_action.py result."

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps({k: out[k] for k in ("likely_cause", "camera_in_boundary", "tf_optical_in_boundary", "frontiers_marker_count", "octomap")}, indent=2))
    print("DIAGNOSE_FKIE_NBV_EMPTY_OK")


if __name__ == "__main__":
    main()
