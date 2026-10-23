#!/usr/bin/env python3
"""Generate a reproducible multi-object layout in aisle_safe_ycb optical FOV.

Default slots match arm_observe_table / spawn_warehouse_aisle_safe_ycb camera ray
(~ table at -4.05,-4.70). Re-check angles if arm bake changes.
"""
from __future__ import print_function
import argparse
import json
import random
from pathlib import Path

YCB = {
    "mustard_bottle": ("bottle", 0.19),
    "bleach_cleanser": ("bottle", 0.25),
    "windex_bottle": ("bottle", 0.27),
    "tomato_soup_can": ("can", 0.10),
    "cracker_box": ("box", 0.21),
    "sugar_box": ("box", 0.18),
    "gelatin_box": ("box", 0.08),
    "banana": ("banana", 0.04),
    "bowl": ("bowl", 0.06),
    "apple": ("apple", 0.08),
}
LOCAL = {"traffic_cone": ("cone", 0.40)}


def build_layout(seed, n_objects=12, fov_slots_path=None):
    rng = random.Random(seed)
    # FOV-aligned table under observe-table optical ray (measured 2026-10-23).
    table_pose = {"x": -4.05, "y": -4.70, "z": 0.0, "yaw": 0.3}
    table_top_z = 0.72
    shelf_z = 1.05

    pool = list(YCB.items()) + list(LOCAL.items())
    rng.shuffle(pool)
    chosen = pool[: max(8, min(n_objects, len(pool)))]

    slots = [
        ("easy", -3.95, -4.55, table_top_z, 0.0),
        ("easy", -4.10, -4.75, table_top_z, 0.2),
        ("easy", -4.20, -4.55, table_top_z, -0.2),
        ("medium", -3.90, -4.85, table_top_z, 0.3),
        ("medium", -4.25, -4.80, table_top_z, -0.3),
        ("medium", -4.40, -5.10, shelf_z, 0.1),
        ("hard", -4.55, -5.30, shelf_z, -0.2),
        ("hard", -4.30, -5.50, shelf_z + 0.30, 0.2),
        ("hard", -3.70, -5.00, 0.0, 0.1),
        ("hard", -4.50, -4.40, 0.0, -0.4),
        ("easy", -4.00, -4.65, table_top_z, 0.0),
        ("medium", -4.15, -4.90, table_top_z, 0.15),
    ]
    if fov_slots_path:
        p = Path(fov_slots_path)
        if p.is_file():
            data = json.loads(p.read_text())
            if data.get("slots"):
                tiers = ["easy", "easy", "easy", "medium", "medium", "medium", "hard", "hard", "hard", "hard", "medium", "easy"]
                slots = []
                for i, s in enumerate(data.get("slots", [])[:12]):
                    slots.append((tiers[i % len(tiers)], s["x"], s["y"], s["z"], rng.uniform(-0.3, 0.3)))

    objects = []
    for i, ((model, (cls, h)), slot) in enumerate(zip(chosen, slots)):
        tier, x, y, z, yaw = slot
        z_place = z + 0.001
        if model == "traffic_cone":
            z_place = max(z, 0.0)
        objects.append(
            {
                "instance_id": "obj_%02d" % i,
                "model": model,
                "class_name": cls,
                "difficulty": tier,
                "spawn": {
                    "x": x + rng.uniform(-0.02, 0.02),
                    "y": y + rng.uniform(-0.02, 0.02),
                    "z": z_place,
                    "yaw": yaw + rng.uniform(-0.05, 0.05),
                },
                "source": "ycb" if model in YCB else "local",
            }
        )

    layout = {
        "scene_id": "ycb_indoor_seed%d" % seed,
        "scene_kind": "ycb",
        "seed": seed,
        "frame_id": "world",
        "notes": "FOV-aligned for aisle_safe_ycb (arm_observe_table). GT from Gazebo after spawn.",
        "furniture": [
            {
                "instance_id": "table_01",
                "model": "simple_table",
                "class_name": "table",
                "spawn": table_pose,
                "source": "local",
            }
        ],
        "objects": objects,
        "occluders": [
            {
                "instance_id": "occluder_01",
                "model": "cardboard_occluder",
                "class_name": "occluder",
                "difficulty": "n/a",
                "spawn": {"x": -3.75, "y": -4.90, "z": 0.35, "yaw": 0.4},
                "source": "local",
                "size": [0.3, 0.06, 0.55],
            }
        ],
    }
    return layout


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--n-objects", type=int, default=12)
    ap.add_argument("--out", default="data/scenes/indoor_objects/ycb/layouts/seed42.json")
    ap.add_argument("--fov-slots", default="")
    args = ap.parse_args()
    layout = build_layout(args.seed, args.n_objects, fov_slots_path=args.fov_slots or None)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(layout, indent=2))
    print("Wrote", out, "objects", len(layout["objects"]))


if __name__ == "__main__":
    main()
