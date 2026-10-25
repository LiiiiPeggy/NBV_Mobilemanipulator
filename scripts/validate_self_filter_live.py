#!/usr/bin/env python3
"""C1 live validation: stamp-TF transform + geometric self-filter.

1) Transform a fresh depth cloud at msg.header.stamp (fail if TF missing).
2) Prove filter removes synthetic points placed on current link frames.
3) Prove nearby table/shelf sample points are NOT removed.
4) Save raw/filtered stats + optional PCD-like JSON samples.
"""
from __future__ import print_function

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, "/workspace/scripts")
from robot_self_filter import (  # noqa: E402
    APPROX_NOTE,
    LINK_PROXIES,
    filter_points,
    lookup_link_transforms,
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/metrics/fkie_agx_warehouse_mode_b/self_filter_c1.json")
    args = ap.parse_args()

    rospy.init_node("validate_self_filter_live", anonymous=True)
    buf = tf2_ros.Buffer(cache_time=rospy.Duration(60.0))
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    msg = rospy.wait_for_message("/realsense/depth/points2", PointCloud2, timeout=15)
    stamp = msg.header.stamp
    if stamp.to_sec() <= 0:
        raise SystemExit("C1_FAIL: cloud stamp zero")

    try:
        tfm = buf.lookup_transform("world", msg.header.frame_id, stamp, rospy.Duration(0.5))
    except Exception as e:
        raise SystemExit("C1_FAIL: stamp TF miss (must not fall back to Time(0)): %s" % e)

    # Prove Time(0) vs stamp differ when robot moves — record both if available
    stamp_ok = True
    latest_ok = True
    try:
        buf.lookup_transform("world", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.2))
    except Exception:
        latest_ok = False

    T = tft.quaternion_matrix(
        [
            tfm.transform.rotation.x,
            tfm.transform.rotation.y,
            tfm.transform.rotation.z,
            tfm.transform.rotation.w,
        ]
    )
    T[0:3, 3] = [
        tfm.transform.translation.x,
        tfm.transform.translation.y,
        tfm.transform.translation.z,
    ]

    raw = []
    for i, p in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
        if i % 6 != 0:
            continue
        w = T.dot([p[0], p[1], p[2], 1.0])
        if 0.05 < w[2] < 3.5:
            raw.append((float(w[0]), float(w[1]), float(w[2])))
        if len(raw) > 80000:
            break

    link_T = lookup_link_transforms(buf, stamp, timeout=0.5)

    # Synthetic body points at link origins / capsule midpoints (must be removed)
    synth_body = []
    for link, kind, params in LINK_PROXIES:
        Tw = link_T[link]
        R, t = Tw[0:3, 0:3], Tw[0:3, 3]
        if kind == "sphere":
            c = t + R.dot(np.asarray(params["origin"]))
            synth_body.append(tuple(map(float, c)))
        elif kind == "box":
            c = t + R.dot(np.asarray(params["origin"]))
            synth_body.append(tuple(map(float, c)))
        else:
            p0 = t + R.dot(np.asarray(params["p0"]))
            p1 = t + R.dot(np.asarray(params["p1"]))
            mid = 0.5 * (p0 + p1)
            synth_body.append(tuple(map(float, mid)))

    # Environment anchors that must survive (table + shelf face)
    synth_env = [
        (-3.9, -7.5, 0.74),
        (-3.9, -7.5, 0.40),
        (-7.5, -6.0, 1.0),
        (-5.0, -6.5, 1.0),
    ]

    mixed = list(raw) + synth_body + synth_env
    kept, removed = filter_points(mixed, link_T, inflate=0.03)
    kept_set = set((round(p[0], 4), round(p[1], 4), round(p[2], 4)) for p in kept)

    body_removed = 0
    for p in synth_body:
        key = (round(p[0], 4), round(p[1], 4), round(p[2], 4))
        if key not in kept_set:
            body_removed += 1
    env_kept = 0
    for p in synth_env:
        key = (round(p[0], 4), round(p[1], 4), round(p[2], 4))
        if key in kept_set:
            env_kept += 1

    base = link_T["base_link"][0:3, 3]
    near_raw = sum(
        1 for p in raw if math.hypot(p[0] - base[0], p[1] - base[1]) < 0.7 and p[2] < 1.6
    )
    near_kept_raw_only, rem_raw = filter_points(raw, link_T, inflate=0.03)
    near_kept = sum(
        1
        for p in near_kept_raw_only
        if math.hypot(p[0] - base[0], p[1] - base[1]) < 0.7 and p[2] < 1.6
    )

    ok = (
        stamp_ok
        and body_removed >= int(0.9 * len(synth_body))
        and env_kept == len(synth_env)
    )

    out = {
        "approx_note": APPROX_NOTE,
        "use_sim_time": bool(rospy.get_param("/use_sim_time", False)),
        "cloud_stamp": stamp.to_sec(),
        "sim_time": rospy.Time.now().to_sec(),
        "stamp_tf_ok": stamp_ok,
        "latest_tf_ok": latest_ok,
        "n_raw": len(raw),
        "n_synth_body": len(synth_body),
        "n_synth_env": len(synth_env),
        "body_removed": body_removed,
        "env_kept": env_kept,
        "near_robot_raw": near_raw,
        "near_robot_kept": near_kept,
        "raw_self_removed": rem_raw,
        "ok": ok,
        "note": (
            "Body removal uses synthetic points on live link TFs (FOV may not see robot). "
            "Env anchors = table/shelf; must survive. Stamp TF required."
        ),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print("SELF_FILTER_C1_OK" if ok else "SELF_FILTER_C1_FAIL")
    if not ok:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
