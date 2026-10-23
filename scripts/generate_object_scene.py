#!/usr/bin/env python3
"""Generate a reproducible multi-object layout in clear aisle (outside Shelves high OBB).

Objects only on table top or floor. Spawn Z uses per-model collision contact
(scripts/ycb_model_contact_z.py), not a uniform table height.
"""
from __future__ import print_function
import argparse
import json
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from ycb_model_contact_z import contact_for_model, spawn_z_on_support, table_top_z  # noqa: E402

YCB = {
    "mustard_bottle": "bottle",
    "bleach_cleanser": "bottle",
    "windex_bottle": "bottle",
    "tomato_soup_can": "can",
    "cracker_box": "box",
    "sugar_box": "box",
    "gelatin_box": "box",
    "banana": "banana",
    "bowl": "bowl",
    "apple": "apple",
}
LOCAL = {"traffic_cone": "cone"}

# Clear-aisle table (outside Shelves high OBB; arm_observe_aisle_table).
TABLE_XY = (-3.90, -7.50)
TABLE_YAW = 0.2


def build_layout(seed, n_objects=12, fov_slots_path=None, root=None):
    root = Path(root or ROOT)
    rng = random.Random(seed)
    tx, ty = TABLE_XY
    top_z = table_top_z(root)
    table_pose = {"x": tx, "y": ty, "z": 0.0, "yaw": TABLE_YAW}

    pool = list(YCB.items()) + list(LOCAL.items())
    rng.shuffle(pool)
    chosen = pool[: max(8, min(n_objects, len(pool)))]

    # slots: tier, x, y, support ("table"|"floor"), yaw
    slots = [
        ("easy", tx + 0.12, ty + 0.10, "table", 0.0),
        ("easy", tx - 0.05, ty - 0.05, "table", 0.2),
        ("easy", tx - 0.15, ty + 0.12, "table", -0.2),
        ("medium", tx + 0.18, ty - 0.12, "table", 0.3),
        ("medium", tx - 0.18, ty - 0.08, "table", -0.3),
        ("medium", tx + 0.02, ty + 0.18, "table", 0.1),
        ("hard", tx + 0.22, ty + 0.02, "table", -0.15),
        ("hard", tx - 0.22, ty + 0.05, "table", 0.25),
        ("hard", tx + 0.45, ty - 0.40, "floor", 0.1),
        ("hard", tx - 0.50, ty - 0.30, "floor", -0.4),
        ("easy", tx + 0.08, ty - 0.18, "table", 0.0),
        ("medium", tx - 0.08, ty + 0.00, "table", 0.15),
    ]
    if fov_slots_path:
        p = Path(fov_slots_path)
        if p.is_file():
            data = json.loads(p.read_text())
            if data.get("slots"):
                tiers = ["easy", "easy", "easy", "medium", "medium", "medium", "hard", "hard", "hard", "hard", "medium", "easy"]
                slots = []
                for i, s in enumerate(data.get("slots", [])[:12]):
                    z = float(s["z"])
                    support = "floor" if abs(z) < 0.05 else "table"
                    slots.append((tiers[i % len(tiers)], s["x"], s["y"], support, rng.uniform(-0.3, 0.3)))

    objects = []
    for i, ((model, cls), slot) in enumerate(zip(chosen, slots)):
        tier, x, y, support, yaw = slot
        source = "ycb" if model in YCB else "local"
        if model == "traffic_cone":
            support = "floor"
            x, y = tx + 0.70, ty - 0.15
        contact = contact_for_model(model, source=source, root=root)
        support_z = top_z if support == "table" else 0.0
        z_place = spawn_z_on_support(support_z, contact["z_bottom"], eps=0.001)
        # Keep table objects inside tabletop half-extent (1.2 x 0.7) with margin.
        if support == "table":
            dx, dy = x - tx, y - ty
            # rotate into table frame (yaw TABLE_YAW)
            import math

            c, s = math.cos(-TABLE_YAW), math.sin(-TABLE_YAW)
            lx, ly = c * dx - s * dy, s * dx + c * dy
            hx, hy = 0.55, 0.30
            lx = max(-hx, min(hx, lx))
            ly = max(-hy, min(hy, ly))
            x = tx + c * lx + s * ly
            y = ty - s * lx + c * ly
        objects.append(
            {
                "instance_id": "obj_%02d" % i,
                "model": model,
                "class_name": cls,
                "difficulty": tier,
                "support": support,
                "contact": {
                    "z_bottom": contact["z_bottom"],
                    "kind": contact.get("kind"),
                    "half_xy": contact.get("half_xy"),
                },
                "spawn": {
                    "x": x + rng.uniform(-0.015, 0.015),
                    "y": y + rng.uniform(-0.015, 0.015),
                    "z": z_place,
                    "yaw": yaw + rng.uniform(-0.05, 0.05),
                },
                "source": source,
            }
        )

    layout = {
        "scene_id": "ycb_indoor_seed%d" % seed,
        "scene_kind": "ycb",
        "seed": seed,
        "frame_id": "world",
        "table_top_z": top_z,
        "notes": (
            "Clear-aisle table (~%.2f,%.2f); table_top_z=%.3f from simple_table collision; "
            "spawn Z from per-model contact; table+floor only; arm_observe_aisle_table.yaml."
            % (tx, ty, top_z)
        ),
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
                "spawn": {
                    "x": tx + 0.55,
                    "y": ty + 0.05,
                    "z": spawn_z_on_support(
                        0.0,
                        contact_for_model("cardboard_occluder", source="local", root=root)["z_bottom"],
                    ),
                    "yaw": TABLE_YAW + 0.2,
                },
                "source": "local",
                "size": [0.4, 0.08, 0.7],
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
    ap.add_argument("--root", default=str(ROOT))
    args = ap.parse_args()
    layout = build_layout(args.seed, args.n_objects, fov_slots_path=args.fov_slots or None, root=args.root)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(layout, indent=2))
    print("Wrote", out, "objects", len(layout["objects"]))
    print("table", layout["furniture"][0]["spawn"], "table_top_z", layout["table_top_z"])
    for o in layout["objects"]:
        print(
            o["instance_id"],
            o["model"],
            o["support"],
            "z=%.4f" % o["spawn"]["z"],
            "z_bottom=%.4f" % o["contact"]["z_bottom"],
        )


if __name__ == "__main__":
    main()
