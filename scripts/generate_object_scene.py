#!/usr/bin/env python3
"""Generate a reproducible multi-object layout near the AGX warehouse ROI.

Does NOT write Gazebo ground-truth from assumptions alone — spawn+export_gt
must refresh poses from /gazebo/get_model_state after spawn.
"""
from __future__ import print_function
import argparse
import json
import random
from pathlib import Path

# Catalog: (model_name, class_name, approx_height_m for placing on surfaces)
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

# Local simple models (not YCB)
LOCAL = {
    "traffic_cone": ("cone", 0.40),
}


def build_layout(seed, n_objects=12, fov_slots_path=None):
    rng = random.Random(seed)
    # YCB/local: shelf bay + table + floor near aisle_safe spawn (~(-2.10,-5.89), yaw≈2.83).
    # Separate from primitive_debug layouts (do not overwrite those).
    table_pose = {"x": -2.55, "y": -6.55, "z": 0.0, "yaw": 0.2}
    table_top_z = 0.72
    # Approximate mid-shelf height facing robot (warehouse shelf east of aisle).
    shelf_z = 0.95

    pool = list(YCB.items()) + list(LOCAL.items())
    rng.shuffle(pool)
    chosen = pool[: max(8, min(n_objects, len(pool)))]

    objects = []
    slots = [
        # table (easy / medium)
        ("easy", -2.40, -6.40, table_top_z, 0.0),
        ("easy", -2.55, -6.60, table_top_z, 0.25),
        ("easy", -2.70, -6.45, table_top_z, -0.2),
        ("medium", -2.35, -6.25, table_top_z, 0.4),
        ("medium", -2.80, -6.30, table_top_z, -0.3),
        # shelf-adjacent (facing aisle_safe camera)
        ("medium", -3.35, -5.55, shelf_z, 0.1),
        ("hard", -3.45, -5.85, shelf_z, -0.2),
        ("hard", -3.25, -6.10, shelf_z + 0.25, 0.3),
        # floor
        ("hard", -2.20, -6.90, 0.0, 0.1),
        ("hard", -2.95, -6.95, 0.0, -0.6),
        ("medium", -2.15, -6.20, 0.0, 0.5),
        ("easy", -2.65, -6.65, table_top_z, -0.1),
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

    for i, ((model, (cls, h)), slot) in enumerate(zip(chosen, slots)):
        tier, x, y, z, yaw = slot
        # place object resting: for table/floor z is surface; add small epsilon
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
                    "x": x + rng.uniform(-0.03, 0.03),
                    "y": y + rng.uniform(-0.03, 0.03),
                    "z": z_place,
                    "yaw": yaw + rng.uniform(-0.05, 0.05),
                },
                "source": "ycb" if model in YCB else "local",
            }
        )

    # Simple occluder box between robot and one hard object
    occluders = [
        {
            "instance_id": "occluder_01",
            "model": "cardboard_occluder",
            "class_name": "occluder",
            "difficulty": "n/a",
            "spawn": {"x": -2.75, "y": -6.80, "z": 0.35, "yaw": 0.7},
            "source": "local",
            "size": [0.4, 0.08, 0.7],
        }
    ]

    layout = {
        "scene_id": "ycb_indoor_seed%d" % seed,
        "scene_kind": "ycb",
        "seed": seed,
        "frame_id": "world",
        "notes": "YCB/local models on table + shelf bay + floor near aisle_safe; GT from Gazebo after spawn. Not primitive_debug.",
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
        "occluders": occluders,
    }
    return layout


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--n-objects", type=int, default=12)
    ap.add_argument(
        "--out",
        default="data/scenes/indoor_objects/ycb/layouts/seed42.json",
    )
    ap.add_argument(
        "--fov-slots",
        default="data/scenes/indoor_objects/layouts/fov_slots_modeA.json",
        help="Optional JSON of depth-valid placement slots for Mode A camera",
    )
    args = ap.parse_args()
    layout = build_layout(args.seed, args.n_objects, fov_slots_path=args.fov_slots)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(layout, indent=2))
    print("Wrote", out, "objects", len(layout["objects"]))


if __name__ == "__main__":
    main()
