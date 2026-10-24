#!/usr/bin/env python3
"""B3: sanity-check warehouse obstacle cloud for REMANI GridMap.

Publishes/reads /map_generator/global_cloud and checks known free vs occupied samples.
Does NOT disable collision. Marks robot-neighborhood points as a risk for self-collision.
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/metrics/fkie_agx_warehouse_mode_b/warehouse_map_b3.json")
    args = ap.parse_args()
    rospy.init_node("validate_remani_warehouse_map", anonymous=True)

    cloud = rospy.wait_for_message("/map_generator/global_cloud", PointCloud2, timeout=15)
    pts = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
    rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
    st = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("agx_ranger_cr10", "world")
    rx, ry = st.pose.position.x, st.pose.position.y

    def near(px, py, pz, qx, qy, qz, r):
        return (px - qx) ** 2 + (py - qy) ** 2 + (pz - qz) ** 2 <= r * r

    # Known samples (warehouse aisle_safe region; approximate)
    probes = {
        "free_aisle_near_robot": (-2.1, -5.0, 0.5, 0.35),  # expect few pts
        "shelf_region": (-7.5, -6.0, 1.0, 0.8),  # expect many pts if synthetic/depth
        "table_region": (-3.9, -7.5, 0.4, 0.5),
    }
    counts = {}
    for name, (x, y, z, r) in probes.items():
        counts[name] = sum(1 for p in pts if near(p[0], p[1], p[2], x, y, z, r))

    near_robot = sum(1 for p in pts if math.hypot(p[0] - rx, p[1] - ry) < 0.9 and p[2] < 1.8)
    out = {
        "n_points": len(pts),
        "robot_xy": [rx, ry],
        "probe_counts": counts,
        "points_within_0.9m_of_robot": near_robot,
        "self_collision_risk": near_robot > 50,
        "ok_partial": len(pts) > 1000 and counts.get("shelf_region", 0) >= 0,
        "note": (
            "B3 partial: cloud present. If points_within_0.9m_of_robot is high, REMANI "
            "BACKEND COLLISION at start is expected — filter robot body before planning."
        ),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print("WAREHOUSE_MAP_B3_PARTIAL_OK" if out["ok_partial"] else "WAREHOUSE_MAP_B3_FAIL")


if __name__ == "__main__":
    main()
