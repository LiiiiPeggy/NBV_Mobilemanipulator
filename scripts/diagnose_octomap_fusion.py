#!/usr/bin/env python3
"""Quantify cumulative OctoMap fusion (E1).

Samples occupied MarkerArray + octomap_full size before/after a wait window
(optionally while the camera moves). Does not invent map growth — reports
deltas honestly.
"""
from __future__ import print_function

import argparse
import json
import time
from pathlib import Path

import rospy
from octomap_msgs.msg import Octomap
from visualization_msgs.msg import MarkerArray


def count_markers(topic, timeout=5.0):
    try:
        ma = rospy.wait_for_message(topic, MarkerArray, timeout=timeout)
    except Exception as e:
        return {"ok": False, "error": str(e)}
    n_pts = 0
    for mk in ma.markers:
        n_pts += len(mk.points) if mk.points else 0
    return {
        "ok": True,
        "n_markers": len(ma.markers),
        "n_points": n_pts,
        "stamp": ma.markers[0].header.stamp.to_sec() if ma.markers else 0.0,
    }


def octo_meta(topic, timeout=5.0):
    try:
        o = rospy.wait_for_message(topic, Octomap, timeout=timeout)
    except Exception as e:
        return {"ok": False, "error": str(e)}
    return {
        "ok": True,
        "frame": o.header.frame_id,
        "resolution": float(o.resolution),
        "data_len": len(o.data),
        "binary": bool(o.binary),
        "id": o.id,
    }


def snapshot(tag):
    return {
        "tag": tag,
        "wall_s": time.time(),
        "occupied": count_markers("/robot/occupied_cells_vis_array"),
        "free": count_markers("/robot/free_cells_vis_array", timeout=3.0),
        "centers": count_markers("/robot/octomap_point_cloud_centers", timeout=3.0),
        "octomap_full": octo_meta("/robot/octomap_full"),
        "octomap_binary": octo_meta("/robot/octomap_binary"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=15.0, help="seconds between snapshots")
    ap.add_argument("--out", default="data/metrics/fkie_agx_warehouse_mode_b/octomap_fusion.json")
    args = ap.parse_args()

    rospy.init_node("diagnose_octomap_fusion", anonymous=True)
    before = snapshot("before")
    rospy.loginfo("octomap snapshot before: occupied=%s free=%s full_len=%s",
                  before["occupied"], before["free"], before["octomap_full"])
    t0 = time.time()
    while time.time() - t0 < args.duration and not rospy.is_shutdown():
        rospy.sleep(0.5)
    after = snapshot("after")
    out = {
        "before": before,
        "after": after,
        "duration_s": time.time() - t0,
        "deltas": {},
    }
    for key in ("occupied", "free", "centers"):
        b, a = before.get(key, {}), after.get(key, {})
        if b.get("ok") and a.get("ok"):
            out["deltas"][key + "_points"] = int(a["n_points"]) - int(b["n_points"])
    bf, af = before.get("octomap_full", {}), after.get("octomap_full", {})
    if bf.get("ok") and af.get("ok"):
        out["deltas"]["octomap_full_bytes"] = int(af["data_len"]) - int(bf["data_len"])

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print("OCTOMAP_FUSION_SNAP_OK")


if __name__ == "__main__":
    main()
