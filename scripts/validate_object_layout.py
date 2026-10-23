#!/usr/bin/env python3
"""Fail-closed offline layout checks for YCB aisle scenes (table+floor only)."""
from __future__ import print_function
import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from ycb_model_contact_z import contact_for_model, table_top_z  # noqa: E402


def load_shelf_local_aabb():
    return np.array([-1.0, -1.2]), np.array([5.03, 1.24])


def shelf_corners(sh, smin, smax):
    local = np.array(
        [[smin[0], smin[1]], [smax[0], smin[1]], [smax[0], smax[1]], [smin[0], smax[1]]],
        dtype=float,
    )
    c, s = math.cos(sh["yaw"]), math.sin(sh["yaw"])
    R = np.array([[c, -s], [s, c]])
    return local @ R.T + np.array([sh["x"], sh["y"]], dtype=float)


def obb_corners(cx, cy, yaw, hx, hy):
    c, s = math.cos(yaw), math.sin(yaw)
    local = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]], dtype=float)
    R = np.array([[c, -s], [s, c]])
    return local @ R.T + np.array([cx, cy], dtype=float)


def sat_overlap(c1, c2):
    def axes(corners):
        e0 = corners[1] - corners[0]
        e1 = corners[3] - corners[0]
        return [
            e0 / (np.linalg.norm(e0) + 1e-12),
            e1 / (np.linalg.norm(e1) + 1e-12),
        ]

    for ax in axes(c1) + axes(c2):
        p1 = c1 @ ax
        p2 = c2 @ ax
        if p1.max() < p2.min() or p2.max() < p1.min():
            return False
    return True


def parse_world_shelves_and_boxes(world_path):
    text = Path(world_path).read_text()
    shelves, boxes = [], []
    for m in re.finditer(
        r"<model name='([^']+)'>\s*<pose[^>]*>\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)",
        text,
    ):
        name = m.group(1)
        x, y, z, rr, rp, ry = map(float, m.groups()[1:])
        if name.startswith("Shelves"):
            shelves.append({"name": name, "x": x, "y": y, "yaw": ry})
        elif "box" in name.lower():
            boxes.append({"name": name, "x": x, "y": y, "yaw": ry, "hx": 0.60, "hy": 0.40})
    return shelves, boxes


def point_in_oriented_rect(px, py, cx, cy, yaw, hx, hy):
    dx, dy = px - cx, py - cy
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return abs(lx) <= hx and abs(ly) <= hy


def validate(layout, world_path, root, min_box_dist=1.7, z_tol=0.025, min_obj_sep=0.06):
    issues = []
    root = Path(root)
    top_z = float(layout.get("table_top_z") or table_top_z(root))
    table = layout["furniture"][0]["spawn"]
    tx, ty, tyaw = float(table["x"]), float(table["y"]), float(table.get("yaw", 0.0))
    # tabletop half from simple_table collision
    tinfo = contact_for_model("simple_table", source="local", root=root)
    thx, thy = float(tinfo["half_xy"][0]), float(tinfo["half_xy"][1])

    shelves, boxes = parse_world_shelves_and_boxes(world_path)
    smin, smax = load_shelf_local_aabb()
    tc = obb_corners(tx, ty, tyaw, thx, thy)
    for sh in shelves:
        sc = shelf_corners(sh, smin, smax)
        if sat_overlap(tc, sc):
            issues.append({"type": "table_shelf_overlap", "shelf": sh["name"]})
    for bx in boxes:
        d = math.hypot(tx - bx["x"], ty - bx["y"])
        if d < min_box_dist:
            issues.append({"type": "table_near_box", "box": bx["name"], "dist": d, "min": min_box_dist})

    # mid-air ban
    for o in layout.get("objects", []):
        z = float(o["spawn"]["z"])
        support = o.get("support")
        if support not in ("table", "floor"):
            # infer
            support = "floor" if z < 0.2 else "table"
        contact = o.get("contact") or contact_for_model(o["model"], source=o.get("source", "ycb"), root=root)
        zb = float(contact["z_bottom"])
        expected_support = top_z if support == "table" else 0.0
        expected_z = expected_support - zb + 0.001
        if abs(z - expected_z) > z_tol:
            issues.append(
                {
                    "type": "support_z_mismatch",
                    "id": o["instance_id"],
                    "z": z,
                    "expected": expected_z,
                    "support": support,
                }
            )
        # ban fake shelf band
        if 0.95 < z < 1.50 and support != "table":
            issues.append({"type": "midair_shelf_z", "id": o["instance_id"], "z": z})
        if support == "table":
            if not point_in_oriented_rect(
                o["spawn"]["x"], o["spawn"]["y"], tx, ty, tyaw, thx - 0.02, thy - 0.02
            ):
                issues.append({"type": "off_table_xy", "id": o["instance_id"], "xy": [o["spawn"]["x"], o["spawn"]["y"]]})

    objs = layout.get("objects", [])
    for i in range(len(objs)):
        for j in range(i + 1, len(objs)):
            a, b = objs[i], objs[j]
            d = math.hypot(a["spawn"]["x"] - b["spawn"]["x"], a["spawn"]["y"] - b["spawn"]["y"])
            ra = max(contact_for_model(a["model"], a.get("source", "ycb"), root).get("half_xy", [0.04])[0], 0.03)
            rb = max(contact_for_model(b["model"], b.get("source", "ycb"), root).get("half_xy", [0.04])[0], 0.03)
            if d < (ra + rb) * 0.85 + min_obj_sep * 0.0:
                if d < max(min_obj_sep, 0.5 * (ra + rb)):
                    issues.append(
                        {
                            "type": "object_overlap_xy",
                            "a": a["instance_id"],
                            "b": b["instance_id"],
                            "dist": d,
                        }
                    )

    return {
        "ok": len(issues) == 0,
        "n_issues": len(issues),
        "issues": issues,
        "table": table,
        "table_top_z": top_z,
        "n_objects": len(objs),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument(
        "--world",
        default=str(ROOT / "configs/fkie/adapt_test/warehouse_icra_large_viz.world"),
    )
    ap.add_argument("--root", default=str(ROOT))
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    layout = json.loads(Path(args.layout).read_text())
    report = validate(layout, args.world, args.root)
    report["layout"] = args.layout
    text = json.dumps(report, indent=2)
    if args.out:
        Path(args.out).write_text(text)
        print("Wrote", args.out)
    print(text)
    if not report["ok"]:
        print("VALIDATE_OBJECT_LAYOUT_FAIL")
        sys.exit(2)
    print("VALIDATE_OBJECT_LAYOUT_OK")


if __name__ == "__main__":
    main()
