#!/usr/bin/env python3
"""Export object ground truth from live Gazebo model states (not layout assumptions).

Fail-closed: missing any layout target → non-zero exit and no EXPORT_OBJECT_GT_OK.
"""
from __future__ import print_function
import argparse
import json
import sys
from pathlib import Path

import rospy
from gazebo_msgs.srv import GetModelState, GetModelProperties
import tf.transformations as tft


def yaw_from_quat(q):
    e = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return float(e[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--spawned", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--prefix", default="mo_")
    ap.add_argument(
        "--allow-partial",
        action="store_true",
        help="Debug only: write GT for found objects even if some missing",
    )
    args = ap.parse_args()

    layout = json.loads(Path(args.layout).read_text())
    spawned_path = args.spawned or str(Path(args.layout).with_suffix(".spawned.json"))
    spawned = json.loads(Path(spawned_path).read_text()) if Path(spawned_path).is_file() else None

    rospy.init_node("export_object_gt", anonymous=True)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=30)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    get_props = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)

    if spawned:
        items = [s for s in spawned["spawned"] if s.get("class_name") not in ("table", "occluder")]
    else:
        items = layout["objects"]

    expected_n = len(layout.get("objects", []))
    gt_objects = []
    missing = []
    for item in items:
        name = item.get("gazebo_name") or (args.prefix + item["instance_id"])
        st = get_state(name, "world")
        if not st.success:
            print("MISSING", name)
            missing.append(name)
            continue
        p = st.pose.position
        q = st.pose.orientation
        size_guess = {
            "bottle": [0.08, 0.08, 0.22],
            "can": [0.07, 0.07, 0.11],
            "box": [0.1, 0.16, 0.2],
            "banana": [0.2, 0.05, 0.04],
            "bowl": [0.16, 0.16, 0.07],
            "apple": [0.08, 0.08, 0.08],
            "cone": [0.24, 0.24, 0.4],
        }.get(item.get("class_name", ""), [0.1, 0.1, 0.1])
        if item.get("source") == "primitive" or item.get("model") == "primitive_sphere":
            r = 0.06
            size_guess = [2 * r, 2 * r, 2 * r]
            cx, cy, cz = float(p.x), float(p.y), float(p.z)
        else:
            cx, cy, cz = float(p.x), float(p.y), float(p.z) + size_guess[2] * 0.5
        gt_objects.append(
            {
                "instance_id": item["instance_id"],
                "gazebo_name": name,
                "class_name": item.get("class_name"),
                "difficulty": item.get("difficulty"),
                "pose": {
                    "x": float(p.x),
                    "y": float(p.y),
                    "z": float(p.z),
                    "qx": float(q.x),
                    "qy": float(q.y),
                    "qz": float(q.z),
                    "qw": float(q.w),
                    "yaw": yaw_from_quat(q),
                },
                "center_xyz": [cx, cy, cz],
                "aabb_size_xyz": size_guess,
                "frame_id": "world",
            }
        )

    out = {
        "scene_id": layout.get("scene_id"),
        "seed": layout.get("seed"),
        "source": "gazebo_get_model_state",
        "n_objects": len(gt_objects),
        "n_expected": expected_n,
        "missing": missing,
        "complete": len(missing) == 0 and len(gt_objects) == expected_n,
        "objects": gt_objects,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print("Wrote", args.out, "n=", len(gt_objects), "expected=", expected_n, "missing=", missing)
    if out["complete"]:
        print("EXPORT_OBJECT_GT_OK")
        return 0
    print("EXPORT_OBJECT_GT_INCOMPLETE")
    if args.allow_partial:
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main() or 0)
