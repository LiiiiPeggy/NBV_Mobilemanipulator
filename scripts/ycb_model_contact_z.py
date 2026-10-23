#!/usr/bin/env python3
"""Compute model contact geometry from SDF collision (box/cylinder preferred).

Returns z_bottom relative to the model origin (world Z of the lowest collision
point when the model is spawned at identity). Spawn on a support at height H:
  spawn_z = H - z_bottom + eps

Mesh-only YCB models in gazebo_ycb ship unit-normalized DAEs without scale;
for those we use frozen catalog dimensions (documented), not raw mesh AABB.
"""
from __future__ import print_function
import argparse
import json
import re
from pathlib import Path

# Frozen fallbacks when SDF has mesh collision without usable metric scale.
# Heights / radii from YCB object set + gazebo_ycb inertial comments; origin
# assumed near geometric center unless noted. z_bottom = -height/2.
MESH_FALLBACK = {
    "banana": {"z_bottom": -0.018, "z_top": 0.018, "half_xy": (0.09, 0.035), "source": "ycb_banana_lying"},
    "apple": {"z_bottom": -0.037, "z_top": 0.037, "half_xy": (0.037, 0.037), "source": "ycb_apple"},
    "bowl": {"z_bottom": -0.027, "z_top": 0.028, "half_xy": (0.08, 0.08), "source": "ycb_bowl"},
    # SDF ships mesh collision + commented box (gazebo_ycb); use commented dims, origin at bottom.
    "mustard_bottle": {"z_bottom": 0.0, "z_top": 0.19, "half_xy": (0.029, 0.0475), "source": "sdf_commented_box"},
    "bleach_cleanser": {"z_bottom": 0.0, "z_top": 0.25, "half_xy": (0.049, 0.0325), "source": "sdf_commented_box"},
}


def _parse_pose_xyz(block):
    pm = re.search(r"<pose[^>]*>([^<]+)</pose>", block)
    if not pm:
        return [0.0, 0.0, 0.0]
    parts = [float(x) for x in pm.group(1).split()]
    return parts[:3]


def contact_from_sdf_text(sdf_text, model_name=""):
    # Prefer first non-commented primitive collision.
    sdf_nocomment = re.sub(r"<!--.*?-->", "", sdf_text, flags=re.S)
    for m in re.finditer(r"<collision[^>]*>(.*?)</collision>", sdf_nocomment, re.S):
        block = m.group(1)
        pose = _parse_pose_xyz(block)
        bm = re.search(r"<box>\s*<size>\s*([^<]+)\s*</size>", block, re.S)
        if bm:
            sx, sy, sz = [float(x) for x in bm.group(1).split()]
            return {
                "kind": "box",
                "z_bottom": pose[2] - sz / 2.0,
                "z_top": pose[2] + sz / 2.0,
                "half_xy": [sx / 2.0, sy / 2.0],
                "model": model_name,
            }
        if "<cylinder>" in block:
            rm = re.search(r"<radius>\s*([^<]+)\s*</radius>", block)
            lm = re.search(r"<length>\s*([^<]+)\s*</length>", block)
            if rm and lm:
                r, L = float(rm.group(1)), float(lm.group(1))
                return {
                    "kind": "cylinder",
                    "z_bottom": pose[2] - L / 2.0,
                    "z_top": pose[2] + L / 2.0,
                    "half_xy": [r, r],
                    "model": model_name,
                }
        if "<sphere>" in block:
            rm = re.search(r"<radius>\s*([^<]+)\s*</radius>", block)
            if rm:
                r = float(rm.group(1))
                return {
                    "kind": "sphere",
                    "z_bottom": pose[2] - r,
                    "z_top": pose[2] + r,
                    "half_xy": [r, r],
                    "model": model_name,
                }

    # Commented box in original SDF (mustard/bleach style) — bottom on z=0.
    cm = re.search(
        r"<!--.*?\<box\>\s*<size>\s*([^<]+)\s*</size>.*?-->",
        sdf_text,
        re.S,
    )
    if cm:
        sx, sy, sz = [float(x) for x in cm.group(1).split()]
        return {
            "kind": "box_comment",
            "z_bottom": 0.0,
            "z_top": sz,
            "half_xy": [sx / 2.0, sy / 2.0],
            "model": model_name,
        }

    if model_name in MESH_FALLBACK:
        fb = dict(MESH_FALLBACK[model_name])
        fb["kind"] = "mesh_fallback"
        fb["model"] = model_name
        fb["half_xy"] = list(fb["half_xy"])
        return fb

    raise ValueError("cannot resolve contact geometry for model %s" % model_name)


def resolve_model_dir(model, source, root):
    root = Path(root)
    if source == "ycb":
        return root / "third_party/gazebo_objects/gazebo_ycb/models" / model
    return root / "configs/fkie/agx_adapt/models" / model


def contact_for_model(model, source="ycb", root="."):
    d = resolve_model_dir(model, source, root)
    sdf = d / "model.sdf"
    if not sdf.is_file():
        raise FileNotFoundError(sdf)
    return contact_from_sdf_text(sdf.read_text(), model_name=model)


def table_top_z(root="."):
    """Upper surface of simple_table tabletop collision box."""
    info = contact_for_model("simple_table", source="local", root=root)
    return float(info["z_top"])


def spawn_z_on_support(support_z, z_bottom, eps=0.001):
    return float(support_z) - float(z_bottom) + float(eps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=".")
    ap.add_argument("--model", action="append", default=[])
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    root = Path(args.root)
    models = args.model or []
    if not models:
        ycb = root / "third_party/gazebo_objects/gazebo_ycb/models"
        models = sorted([p.name for p in ycb.iterdir() if (p / "model.sdf").is_file()]) if ycb.is_dir() else []
        models += ["simple_table", "traffic_cone", "cardboard_occluder"]
    rows = {}
    for m in models:
        src = "local" if m in ("simple_table", "traffic_cone", "cardboard_occluder") else "ycb"
        try:
            rows[m] = contact_for_model(m, source=src, root=root)
        except Exception as e:
            rows[m] = {"error": str(e), "model": m}
    print("table_top_z", table_top_z(root))
    if args.out:
        Path(args.out).write_text(json.dumps(rows, indent=2))
        print("Wrote", args.out)
    else:
        print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
