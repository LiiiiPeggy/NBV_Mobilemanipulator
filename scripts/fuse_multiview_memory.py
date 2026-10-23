#!/usr/bin/env python3
"""Fuse multiview pred3d JSONs into ObjectMemory and compare metrics."""
from __future__ import print_function
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from perception.object_memory import ObjectMemory  # noqa: E402
from evaluate_object_discovery import match, collect_preds  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--gt", default="")
    args = ap.parse_args()
    out = Path(args.out_dir)
    gt_path = Path(args.gt) if args.gt else ROOT / ("data/scenes/indoor_objects/ycb/gt/seed%d_gt.json" % args.seed)
    gt = json.loads(gt_path.read_text())
    all_pred = {"views": []}
    single = []
    for vid in (0, 1, 2):
        p3 = out / ("pred3d_view%d.json" % vid)
        if not p3.is_file():
            continue
        p = json.loads(p3.read_text())
        cam_p = out / ("cam_view%d.json" % vid)
        cam = json.loads(cam_p.read_text()) if cam_p.is_file() else {}
        for v in p.get("views", []):
            v["view_id"] = vid
            v["camera_pose_xyz"] = cam.get("camera_pose_xyz")
            v["tag"] = cam.get("tag")
        all_pred["views"].extend(p.get("views", []))
        m = match(collect_preds(p), gt["objects"], 0.15)
        single.append(
            {
                "view_id": vid,
                "tag": cam.get("tag"),
                "n_det": m["tp"] + m["fp"],
                "tp": m["tp"],
                "fp": m["fp"],
                "fn": m["fn"],
                "precision": m["precision"],
                "unique_recall": m["unique_recall"],
            }
        )

    naive = match(collect_preds(all_pred), gt["objects"], 0.15)
    mem = ObjectMemory(assoc_thresh_m=0.25, confirm_min_obs=2)
    mem.ingest_perception_json(all_pred)
    mem.save(str(out / "memory.json"))
    mem_objs = mem.to_dict()["objects"]
    mem_m = match(
        [
            {
                "class_name": o["class_name"],
                "center": np.array(o["position_world"]),
                "confidence": o.get("observation_quality", 0),
                "view_id": -1,
            }
            for o in mem_objs
        ],
        gt["objects"],
        0.15,
    )
    confirmed = [o for o in mem_objs if o.get("confirmation_status") == "confirmed"]
    conf_m = match(
        [
            {
                "class_name": o["class_name"],
                "center": np.array(o["position_world"]),
                "confidence": o.get("observation_quality", 0),
                "view_id": -1,
            }
            for o in confirmed
        ],
        gt["objects"],
        0.15,
    )
    report = {
        "seed": args.seed,
        "per_view": single,
        "naive_accumulate": {k: naive[k] for k in ("tp", "fp", "fn", "precision", "unique_recall", "mean_center_error_m")},
        "memory_all": {k: mem_m[k] for k in ("tp", "fp", "fn", "precision", "unique_recall", "mean_center_error_m")},
        "memory_confirmed": {k: conf_m[k] for k in ("tp", "fp", "fn", "precision", "unique_recall", "mean_center_error_m")},
        "n_memory_objects": len(mem_objs),
        "n_confirmed": len(confirmed),
        "n_duplicates_reduced": max(0, (naive["tp"] + naive["fp"]) - len(mem_objs)),
    }
    (out / "multiview_summary.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    print("FUSE_MULTIVIEW_MEMORY_OK")


if __name__ == "__main__":
    main()
