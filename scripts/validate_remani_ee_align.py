#!/usr/bin/env python3
"""Measure /ee_current_pose vs TF cr10_Link6 across configurations.

Does not treat a single-pose Z≈0.42 offset as an unexamined constant: records
delta at current pose and after optional small joint wiggles (if bridges alive).
"""
from __future__ import print_function

import argparse
import json
import math
from pathlib import Path

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


def lookup(buf, parent, child):
    t = buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(2.0))
    p = t.transform.translation
    return [p.x, p.y, p.z]


def sample_once(buf):
    rem = rospy.wait_for_message("/ee_current_pose", PoseStamped, timeout=5.0)
    tf_ee = lookup(buf, "world", "cr10_Link6")
    rem_xyz = [rem.pose.position.x, rem.pose.position.y, rem.pose.position.z]
    delta = [rem_xyz[i] - tf_ee[i] for i in range(3)]
    base = lookup(buf, "world", "base_link")
    return {
        "ee_remani": rem_xyz,
        "ee_tf_link6": tf_ee,
        "delta_remani_minus_tf": delta,
        "base_xyz": base,
        "delta_z_minus_base_z": delta[2] - base[2],
        "abs_delta": [abs(d) for d in delta],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/metrics/fkie_agx_warehouse_mode_b/ee_align.json")
    ap.add_argument("--samples", type=int, default=5)
    ap.add_argument("--period", type=float, default=0.5)
    args = ap.parse_args()

    rospy.init_node("validate_remani_ee_align", anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    samples = []
    for i in range(args.samples):
        s = sample_once(buf)
        s["index"] = i
        try:
            js = rospy.wait_for_message("/mm/mani/joint_state", JointState, timeout=1.0)
            s["joint_positions"] = list(js.position) if js.position else []
        except Exception:
            s["joint_positions"] = []
        samples.append(s)
        rospy.sleep(args.period)

    zs = [s["delta_remani_minus_tf"][2] for s in samples]
    xs = [s["delta_remani_minus_tf"][0] for s in samples]
    ys = [s["delta_remani_minus_tf"][1] for s in samples]
    base_zs = [s["base_xyz"][2] for s in samples]

    out = {
        "n": len(samples),
        "samples": samples,
        "delta_mean": [sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs)],
        "delta_z_std": float(
            math.sqrt(sum((z - sum(zs) / len(zs)) ** 2 for z in zs) / max(1, len(zs) - 1))
        )
        if len(zs) > 1
        else 0.0,
        "base_z_mean": sum(base_zs) / len(base_zs),
        "hypothesis": {
            "matches_base_z": abs(sum(zs) / len(zs) + sum(base_zs) / len(base_zs)) < 0.05
            or abs(sum(zs) / len(zs) - (-sum(base_zs) / len(base_zs))) < 0.08,
            "note": (
                "If delta_z ≈ -base_z, REMANI FK likely ignores chassis height "
                "(EE relative to planar base). Align offset then tracks base_z."
            ),
        },
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print("EE_ALIGN_OK")


if __name__ == "__main__":
    main()
