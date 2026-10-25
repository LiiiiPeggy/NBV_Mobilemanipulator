#!/usr/bin/env python3
"""Diagnose YCB models that exist in Gazebo but may be invisible in GUI."""
from __future__ import print_function
import argparse
import json
import math
import os
from pathlib import Path

import rospy
from gazebo_msgs.srv import GetModelState, GetModelProperties, GetWorldProperties
from sensor_msgs.msg import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--prefix", default="mo_")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    layout = json.loads(Path(args.layout).read_text())
    items = (
        list(layout.get("furniture", []))
        + list(layout.get("objects", []))
        + list(layout.get("occluders", []))
    )

    rospy.init_node("diagnose_ycb_gazebo_visibility", anonymous=True)
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=30)
    get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    get_props = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)

    world = get_world()
    world_names = set(world.model_names)
    master = os.environ.get("GAZEBO_MASTER_URI", "http://127.0.0.1:11355")
    ros_master = os.environ.get("ROS_MASTER_URI", "")

    rows = []
    for item in items:
        name = args.prefix + item["instance_id"]
        row = {
            "gazebo_name": name,
            "in_world": name in world_names,
            "pose": None,
            "geom_names": [],
            "visual_hint": None,
            "issues": [],
        }
        if name not in world_names:
            row["issues"].append("missing_from_world")
            rows.append(row)
            continue
        st = get_state(name, "world")
        if not st.success:
            row["issues"].append("get_model_state_failed")
            rows.append(row)
            continue
        p = st.pose.position
        row["pose"] = {"x": float(p.x), "y": float(p.y), "z": float(p.z)}
        if abs(p.x) + abs(p.y) + abs(p.z) < 1e-6:
            row["issues"].append("pose_at_origin_suspicious")
        if math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z):
            row["issues"].append("pose_nan")
        try:
            props = get_props(name)
            if props.success:
                row["geom_names"] = list(props.geom_names)
                if not props.geom_names:
                    row["issues"].append("no_collision_geoms")
                row["visual_hint"] = {
                    "body_names": list(getattr(props, "body_names", []) or []),
                }
                if hasattr(props, "static"):
                    row["visual_hint"]["is_static"] = bool(props.static)
            else:
                row["issues"].append("get_model_properties_failed")
        except Exception as e:
            row["issues"].append("props_exc:" + str(e))
        rows.append(row)

    d455 = {"rgb": False, "depth": False}
    try:
        rospy.wait_for_message("/realsense/rgb/image_raw", Image, timeout=3)
        d455["rgb"] = True
    except Exception:
        pass
    try:
        rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=3)
        d455["depth"] = True
    except Exception:
        pass

    # Camera should roughly see table ROI if AGX is aisle_safe_ycb
    table_xy = (-3.9, -7.5)
    agx = get_state("agx_ranger_cr10", "world")
    agx_xy = None
    if agx.success:
        agx_xy = [float(agx.pose.position.x), float(agx.pose.position.y)]

    out = {
        "gazebo_master_uri": master,
        "ros_master_uri": ros_master,
        "sim_time": float(world.sim_time),
        "n_world_models": len(world_names),
        "agx_xy": agx_xy,
        "table_xy_expected": list(table_xy),
        "d455": d455,
        "n_layout": len(items),
        "n_present": sum(1 for r in rows if r["in_world"]),
        "n_with_issues": sum(1 for r in rows if r["issues"]),
        "models": rows,
        "gui_notes": [
            "If models exist with geom but GUI empty: check gzclient connected to same GAZEBO_MASTER_URI",
            "Aim camera: gz camera -c gzclient_camera -p '-1.6 -5.6 2.2' -t '-3.5 -7.2 0.9'",
            "Warehouse world may spawn gzclient looking at origin — table is near (-3.9,-7.5)",
        ],
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps({k: out[k] for k in (
        "gazebo_master_uri", "n_world_models", "n_present", "n_with_issues", "d455", "agx_xy"
    )}, indent=2))
    print("YCB_VISIBILITY_DIAG_OK" if out["n_present"] == out["n_layout"] else "YCB_VISIBILITY_DIAG_PARTIAL")


if __name__ == "__main__":
    main()
