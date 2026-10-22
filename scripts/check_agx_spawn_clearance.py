#!/usr/bin/env python3
"""Check AGX base footprint vs warehouse shelves_high + nearby boxes (OBB/SAT).

World-AABB of a long rotated shelf is overly conservative; use oriented boxes.
Optional live /gazebo/get_model_state when --live.
"""
from __future__ import print_function
import argparse
import json
import math
import re
from pathlib import Path

import numpy as np


def load_shelf_local_aabb():
    # From shelves_high_collision.dae + scale 1 1.2 1.2 (validated earlier)
    return np.array([-1.0, -1.2]), np.array([5.03, 1.24])


def obb_corners(cx, cy, yaw, hx, hy):
    c, s = math.cos(yaw), math.sin(yaw)
    local = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]], dtype=float)
    R = np.array([[c, -s], [s, c]])
    return local @ R.T + np.array([cx, cy], dtype=float)


def shelf_corners(sh, smin, smax):
    local = np.array(
        [
            [smin[0], smin[1]],
            [smax[0], smin[1]],
            [smax[0], smax[1]],
            [smin[0], smax[1]],
        ],
        dtype=float,
    )
    c, s = math.cos(sh["yaw"]), math.sin(sh["yaw"])
    R = np.array([[c, -s], [s, c]])
    return local @ R.T + np.array([sh["x"], sh["y"]], dtype=float)


def sat_overlap(c1, c2, margin=0.0):
    if margin:
        def grow(corners, m):
            cen = corners.mean(0)
            v = corners - cen
            n = np.linalg.norm(v, axis=1, keepdims=True) + 1e-9
            return cen + v * (1.0 + m / n)

        c1, c2 = grow(c1, margin), grow(c2, margin)

    def axes(corners):
        e0 = corners[1] - corners[0]
        e1 = corners[3] - corners[0]
        a0 = e0 / (np.linalg.norm(e0) + 1e-12)
        a1 = e1 / (np.linalg.norm(e1) + 1e-12)
        return [a0, a1]

    for ax in axes(c1) + axes(c2):
        p1 = c1 @ ax
        p2 = c2 @ ax
        if p1.max() < p2.min() or p2.max() < p1.min():
            return False
    return True


def parse_world_shelves_and_boxes(world_path):
    text = Path(world_path).read_text()
    shelves = []
    boxes = []
    for m in re.finditer(
        r"<model name='([^']+)'>\s*<pose[^>]*>\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)",
        text,
    ):
        name = m.group(1)
        x, y, z, rr, rp, ry = map(float, m.groups()[1:])
        if name.startswith("Shelves"):
            shelves.append({"name": name, "x": x, "y": y, "z": z, "yaw": ry})
        elif "box" in name.lower() or name.startswith("Big box") or name.startswith("big_box"):
            boxes.append({"name": name, "x": x, "y": y, "z": z, "yaw": ry, "hx": 0.60, "hy": 0.40})
    return shelves, boxes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spawn-cfg", required=True)
    ap.add_argument("--world", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--half-x", type=float, default=0.55)
    ap.add_argument("--half-y", type=float, default=0.40)
    ap.add_argument("--margin", type=float, default=0.15)
    ap.add_argument("--live", action="store_true")
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[1]
    cfg = __import__("yaml").safe_load(Path(args.spawn_cfg).read_text())
    sp = cfg["spawn"]
    world = args.world or str(root / cfg.get("world", "configs/fkie/adapt_test/warehouse_icra_large_viz.world"))
    shelves, boxes = parse_world_shelves_and_boxes(world)
    smin, smax = load_shelf_local_aabb()

    ax, ay, ayaw = float(sp["x"]), float(sp["y"]), float(sp["yaw"])
    agx = obb_corners(ax, ay, ayaw, args.half_x, args.half_y)

    hits = []
    shelf_footprints = []
    for sh in shelves:
        sc = shelf_corners(sh, smin, smax)
        shelf_footprints.append({"name": sh["name"], "corners": sc.tolist(), "pose": sh})
        if sat_overlap(agx, sc, args.margin):
            hits.append({"type": "shelf", "name": sh["name"], "corners": sc.tolist()})
    for bx in boxes:
        bc = obb_corners(bx["x"], bx["y"], bx["yaw"], bx["hx"], bx["hy"])
        if sat_overlap(agx, bc, args.margin):
            hits.append({"type": "box", "name": bx["name"], "center": [bx["x"], bx["y"]], "corners": bc.tolist()})

    live = None
    if args.live:
        try:
            import rospy
            from gazebo_msgs.srv import GetModelState

            rospy.init_node("check_agx_spawn_clearance_live", anonymous=True)
            rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
            srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            resp = srv("agx_ranger_cr10", "world")
            live = {
                "success": bool(resp.success),
                "x": float(resp.pose.position.x),
                "y": float(resp.pose.position.y),
                "z": float(resp.pose.position.z),
            }
        except Exception as e:
            live = {"error": str(e)}

    out = {
        "spawn_cfg": args.spawn_cfg,
        "scene_id": cfg.get("scene_id"),
        "method": "obb_sat",
        "margin_m": args.margin,
        "spawn": sp,
        "agx_obb_corners": agx.tolist(),
        "n_shelves": len(shelves),
        "n_boxes_parsed": len(boxes),
        "shelf_footprints": shelf_footprints,
        "overlaps": hits,
        "clear": len(hits) == 0,
        "live_model_state": live,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps({"clear": out["clear"], "n_overlaps": len(hits), "spawn": sp, "method": "obb_sat"}, indent=2))
    print("CHECK_AGX_SPAWN_CLEARANCE_OK" if out["clear"] else "CHECK_AGX_SPAWN_CLEARANCE_FAIL")


if __name__ == "__main__":
    main()
