#!/usr/bin/env python3
"""Runtime acceptance for a spawned multi-object Gazebo scene (fail-closed)."""
from __future__ import print_function
import argparse
import json
import os
import sys
from pathlib import Path

import rospy
from gazebo_msgs.srv import GetModelState, GetWorldProperties, GetModelProperties


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--spawned", default="")
    ap.add_argument("--prefix", default="mo_")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-xy-err", type=float, default=0.15)
    ap.add_argument("--max-z-err", type=float, default=0.30)
    ap.add_argument("--min-z", type=float, default=-0.02)
    args = ap.parse_args()

    layout = json.loads(Path(args.layout).read_text())
    spawned_path = args.spawned or str(Path(args.layout).with_suffix(".spawned.json"))
    spawned = json.loads(Path(spawned_path).read_text()) if Path(spawned_path).is_file() else None

    rospy.init_node("verify_gazebo_object_scene", anonymous=True)
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=30)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=30)
    get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    get_props = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)

    world = get_world()
    world_names = list(world.model_names)

    expected_items = (
        list(layout.get("furniture", []))
        + list(layout.get("objects", []))
        + list(layout.get("occluders", []))
    )
    targets = list(layout.get("objects", []))

    rows = []
    n_exist = 0
    n_pose_ok = 0
    n_below = 0
    for item in expected_items:
        name = args.prefix + item["instance_id"]
        row = {
            "gazebo_name": name,
            "model": item.get("model"),
            "class_name": item.get("class_name"),
            "expected": item.get("spawn"),
            "in_world": name in world_names,
            "exists": False,
            "actual": None,
            "err_xyz": None,
            "below_ground": False,
            "geom_names": [],
        }
        st = get_state(name, "world")
        if st.success:
            row["exists"] = True
            n_exist += 1
            p = st.pose.position
            row["actual"] = {"x": float(p.x), "y": float(p.y), "z": float(p.z)}
            ex = item["spawn"]
            err = [
                abs(p.x - ex["x"]),
                abs(p.y - ex["y"]),
                abs(p.z - ex["z"]),
            ]
            row["err_xyz"] = err
            if err[0] <= args.max_xy_err and err[1] <= args.max_xy_err and err[2] <= args.max_z_err:
                n_pose_ok += 1
                row["pose_ok"] = True
            else:
                row["pose_ok"] = False
            if p.z < args.min_z:
                row["below_ground"] = True
                n_below += 1
            try:
                props = get_props(name)
                row["geom_names"] = list(props.geom_names) if props.success else []
            except Exception as e:
                row["geom_error"] = str(e)
        else:
            row["pose_ok"] = False
        rows.append(row)

    spawn_ok = None
    if spawned:
        spawn_ok = int(spawned.get("n_ok", 0))

    n_targets = len(targets)
    n_targets_exist = sum(
        1 for r in rows if r["class_name"] not in ("table", "occluder") and r["exists"]
    )
    # Support-surface checks from layout metadata (fail-closed when present).
    support_issues = []
    top_z = float(layout.get("table_top_z", 0.74))
    for item, row in zip(expected_items, rows):
        if not row.get("exists") or item.get("class_name") in ("table",):
            continue
        if "support" not in item or "contact" not in item or not row.get("actual"):
            continue
        zb = float(item["contact"]["z_bottom"])
        support = item["support"]
        expected_z = (top_z if support == "table" else 0.0) - zb + 0.001
        az = float(row["actual"]["z"])
        if abs(az - expected_z) > 0.05:
            support_issues.append(
                {"id": item["instance_id"], "actual_z": az, "expected_z": expected_z, "support": support}
            )
        if support == "floor" and az > 0.15:
            support_issues.append({"id": item["instance_id"], "type": "floor_too_high", "z": az})

    ok = (
        n_targets_exist == n_targets
        and n_exist == len(expected_items)
        and n_below == 0
        and all(r.get("pose_ok") for r in rows if r["exists"])
        and len(support_issues) == 0
    )

    report = {
        "layout": args.layout,
        "scene_id": layout.get("scene_id"),
        "n_expected_total": len(expected_items),
        "n_expected_targets": n_targets,
        "n_world_models": len(world_names),
        "n_exist": n_exist,
        "n_targets_exist": n_targets_exist,
        "n_pose_ok": n_pose_ok,
        "n_below_ground": n_below,
        "n_support_issues": len(support_issues),
        "support_issues": support_issues,
        "spawned_n_ok": spawn_ok,
        "ok": ok,
        "world_model_names": sorted(world_names),
        "objects": rows,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=2))
    print(json.dumps({k: report[k] for k in ("n_expected_targets", "n_targets_exist", "n_exist", "n_pose_ok", "n_below_ground", "ok")}, indent=2))
    if ok:
        print("VERIFY_GAZEBO_OBJECT_SCENE_OK")
        return 0
    print("VERIFY_GAZEBO_OBJECT_SCENE_FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main() or 0)
