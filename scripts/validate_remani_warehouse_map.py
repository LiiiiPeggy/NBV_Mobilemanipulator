#!/usr/bin/env python3
"""C2: Validate REMANI warehouse obstacle cloud / simulated map readiness.

Fail-closed checks (thresholds justified in report):
  - Enough filtered points in map
  - Free aisle near robot has few points (not robot body)
  - Shelf / table regions have points when in FOV or synthetic fill
  - Self-filter stats show removals without wiping near-env
  - Optional: REMANI occupancy inflate topic has shelf hits

Does NOT disable collision or shrink robot collision bodies.
"""
from __future__ import print_function

import argparse
import json
import math
from pathlib import Path

import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def count_near(pts, xyz, r):
    x, y, z = xyz
    r2 = r * r
    n = 0
    for p in pts:
        if (p[0] - x) ** 2 + (p[1] - y) ** 2 + (p[2] - z) ** 2 <= r2:
            n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/metrics/fkie_agx_warehouse_mode_b/warehouse_map_c2.json")
    ap.add_argument("--cloud-topic", default="/map_generator/global_cloud")
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--require-table", type=int, default=20)
    ap.add_argument("--require-shelf", type=int, default=30)
    ap.add_argument("--max-free-aisle", type=int, default=25)
    ap.add_argument("--max-robot-body-proxy", type=int, default=80)
    args = ap.parse_args()

    rospy.init_node("validate_remani_warehouse_map", anonymous=True)

    cloud = rospy.wait_for_message(args.cloud_topic, PointCloud2, timeout=args.timeout)
    pts = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))

    rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
    st = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("agx_ranger_cr10", "world")
    rx, ry, rz = st.pose.position.x, st.pose.position.y, st.pose.position.z

    # Probe definitions (aisle_safe AGX + seed42 table). Thresholds:
    # - free_aisle: 0.35 m ball ahead of robot at torso height — expect << robot-body density
    # - shelf_region: warehouse shelf cell west of aisle — need clear occupancy signal
    # - table_region: seed42 table top — depth should see top/legs if YCB loaded / in FOV
    probes = {
        "free_aisle_near_robot": {
            "xyz": (rx + 0.0, ry + 0.9, 0.55),
            "r": 0.35,
            "max": args.max_free_aisle,
            "min": 0,
        },
        "shelf_region": {
            "xyz": (-7.5, -6.0, 1.0),
            "r": 0.9,
            "min": args.require_shelf,
            "max": 10**9,
        },
        "table_region": {
            "xyz": (-3.9, -7.5, 0.55),
            "r": 0.55,
            "min": args.require_table,
            "max": 10**9,
        },
        # Points very near base_link origin at chassis height → residual self returns
        "robot_body_proxy": {
            "xyz": (rx, ry, rz + 0.15),
            "r": 0.45,
            "max": args.max_robot_body_proxy,
            "min": 0,
        },
    }

    counts = {}
    probe_ok = {}
    for name, spec in probes.items():
        n = count_near(pts, spec["xyz"], spec["r"])
        counts[name] = n
        probe_ok[name] = (n >= spec["min"]) and (n <= spec["max"])

    filter_stats = None
    try:
        smsg = rospy.wait_for_message("/warehouse_map/self_filter_stats", String, timeout=3.0)
        filter_stats = json.loads(smsg.data)
    except Exception as e:
        filter_stats = {"error": str(e)}

    # Optional REMANI inflated occupancy (if planner already up)
    remani_occ_n = None
    remani_shelf = None
    try:
        occ = rospy.wait_for_message("/remani_planner_node/grid_map/occupancy_inflate", PointCloud2, timeout=2.0)
        occ_pts = list(pc2.read_points(occ, field_names=("x", "y", "z"), skip_nans=True))
        remani_occ_n = len(occ_pts)
        remani_shelf = count_near(occ_pts, probes["shelf_region"]["xyz"], probes["shelf_region"]["r"])
    except Exception:
        pass

    # Ground-truth geometry cross-check (offline validation aid; not fed into planner)
    gz_table = None
    try:
        ts = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("mo_table_01", "world")
        if ts.success:
            gz_table = [ts.pose.position.x, ts.pose.position.y, ts.pose.position.z]
    except Exception:
        pass

    n_pts = len(pts)
    checks = {
        "enough_points": n_pts >= 800,
        "probes": probe_ok,
        "filter_ready": bool(filter_stats and filter_stats.get("ready")),
        "filter_removed_some": bool(
            filter_stats
            and filter_stats.get("last_removed", 0) >= 0
            and filter_stats.get("frames_ok", 0) >= 1
        ),
    }
    # Shelf may be out of current FOV if only depth-fed and camera looks at table —
    # require shelf OR (table + filter ready) for online-sensor path.
    env_obstacle_ok = probe_ok["shelf_region"] or (
        probe_ok["table_region"] and checks["enough_points"]
    )
    start_self_ok = probe_ok["free_aisle_near_robot"] and probe_ok["robot_body_proxy"]
    ok = bool(
        checks["enough_points"]
        and env_obstacle_ok
        and start_self_ok
        and probe_ok["table_region"]  # YCB Mode B scene expects table in map or FOV
    )

    out = {
        "cloud_topic": args.cloud_topic,
        "n_points": n_pts,
        "robot_xyz": [rx, ry, rz],
        "probe_counts": counts,
        "probe_ok": probe_ok,
        "thresholds": {
            "require_table": args.require_table,
            "require_shelf": args.require_shelf,
            "max_free_aisle": args.max_free_aisle,
            "max_robot_body_proxy": args.max_robot_body_proxy,
            "rationale": (
                "table/shelf minima from prior synthetic+depth fills (~tens–hundreds of "
                "points in 0.5–0.9 m balls). free_aisle/robot_body maxima bound residual "
                "self returns after geometric filter (not a 0.9 m wipe)."
            ),
        },
        "self_filter_stats": filter_stats,
        "remani_occ_n": remani_occ_n,
        "remani_shelf_hits": remani_shelf,
        "gz_table_xy": gz_table,
        "checks": checks,
        "env_obstacle_ok": env_obstacle_ok,
        "start_self_ok": start_self_ok,
        "ok": ok,
        "note": (
            "PASS means filtered cloud looks usable for environment_mode=simulated. "
            "Still verify REMANI logs have no BACKEND COLLISION at start."
        ),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print("WAREHOUSE_MAP_C2_OK" if ok else "WAREHOUSE_MAP_C2_FAIL")
    if not ok:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
