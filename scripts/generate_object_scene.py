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


def build_layout(seed, n_objects=10, fov_slots_path=None):
    rng = random.Random(seed)
    # Table + objects around arm_observe_table look-at (~(-3.05,-6.37,0.86)).
    # Default Mode A arm_view_pose remains for NBV; P3 uses temporary observation joints.
    table_pose = {"x": -3.05, "y": -6.35, "z": 0.0, "yaw": 0.1}
    table_top_z = 0.72

    pool = list(YCB.items()) + list(LOCAL.items())
    rng.shuffle(pool)
    chosen = pool[: max(8, min(n_objects, len(pool)))]

    objects = []
    slots = [
        ("easy", -2.90, -6.20, table_top_z, 0.0),
        ("easy", -3.10, -6.40, table_top_z, 0.25),
        ("easy", -3.25, -6.25, table_top_z, -0.2),
        ("medium", -2.85, -6.00, 1.00, 0.4),
        ("medium", -3.30, -6.10, 1.15, -0.3),
        ("medium", -2.95, -6.55, table_top_z, 0.8),
        ("hard", -2.70, -6.70, 0.0, 0.1),
        ("hard", -3.40, -6.75, 0.0, -0.6),
        ("hard", -2.60, -6.10, 0.0, 0.5),
        ("hard", -3.50, -5.95, 1.35, 0.0),
        ("medium", -3.05, -6.30, table_top_z, 0.35),
        ("easy", -3.15, -6.45, table_top_z, -0.1),
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
            "spawn": {"x": -3.25, "y": -6.55, "z": 0.35, "yaw": 0.7},
            "source": "local",
            "size": [0.4, 0.08, 0.7],
        }
    ]

    layout = {
        "scene_id": "indoor_objects_seed%d" % seed,
        "seed": seed,
        "frame_id": "world",
        "notes": "Layout near AGX ROI; GT must be refreshed from Gazebo after spawn.",
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
    ap.add_argument("--n-objects", type=int, default=10)
    ap.add_argument(
        "--out",
        default="data/scenes/indoor_objects/layouts/seed42.json",
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
