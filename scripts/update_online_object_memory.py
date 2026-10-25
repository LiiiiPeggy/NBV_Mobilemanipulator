#!/usr/bin/env python3
"""Ingest one snapshot perception into online ObjectMemory and optional GT eval delta."""
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


def load_gt(path):
    d = json.loads(Path(path).read_text())
    if "objects" in d and d["objects"] and "center_xyz" in d["objects"][0]:
        return d
    objs = []
    for o in d.get("objects", []):
        c = o.get("center_xyz")
        if c is None and "pose" in o:
            c = [o["pose"]["x"], o["pose"]["y"], o["pose"].get("z", 0.0) + 0.05]
        objs.append(
            {
                "class_name": o["class_name"],
                "center_xyz": c,
                "instance_id": o.get("instance_id"),
            }
        )
    return {"objects": objs}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--memory", required=True, help="path to object_memory.json (created if missing)")
    ap.add_argument("--perception", required=True, help="snapshot perception.json / objects_3d.json")
    ap.add_argument("--gt", default="")
    ap.add_argument("--out-eval", default="")
    ap.add_argument("--view-id", type=int, default=0)
    ap.add_argument("--assoc-thresh", type=float, default=0.25)
    ap.add_argument("--confirm-min-obs", type=int, default=2)
    args = ap.parse_args()

    mem_path = Path(args.memory)
    perc = json.loads(Path(args.perception).read_text())
    st = perc.get("status") or {}
    if isinstance(st, dict) and st.get("status") == "model_fail":
        raise SystemExit("PERCEPTION_FAIL: refusing to update memory from model_fail")
    if isinstance(st, dict) and st.get("status") == "INPUT_MISSING":
        raise SystemExit("PERCEPTION_FAIL: input missing")

    if mem_path.is_file():
        mem = ObjectMemory.load(str(mem_path))
        mem.assoc_thresh_m = float(args.assoc_thresh)
        mem.confirm_min_obs = int(args.confirm_min_obs)
    else:
        mem = ObjectMemory(assoc_thresh_m=args.assoc_thresh, confirm_min_obs=args.confirm_min_obs)

    before_n = len(mem.objects)
    before_confirmed_ids = {
        oid for oid, o in mem.objects.items() if o.confirmation_status == "confirmed"
    }
    before_ids = set(mem.objects.keys())

    for v in perc.get("views", []):
        v["view_id"] = int(args.view_id)

    mem.ingest_perception_json(perc)
    after_ids = set(mem.objects.keys())
    after_confirmed = {
        oid for oid, o in mem.objects.items() if o.confirmation_status == "confirmed"
    }
    mem.save(str(mem_path))

    frame_preds = collect_preds(perc)
    frame_n = len(frame_preds)
    new_ids = sorted(after_ids - before_ids)
    newly_confirmed = sorted(after_confirmed - before_confirmed_ids)

    report = {
        "snapshot_id": perc.get("snapshot_id"),
        "view_id": args.view_id,
        "new_observation": True,
        "new_detection": frame_n,
        "new_confirmed_object": len(newly_confirmed),
        "n_memory_before": before_n,
        "n_memory_after": len(mem.objects),
        "n_confirmed_before": len(before_confirmed_ids),
        "n_confirmed_after": len(after_confirmed),
        "new_object_ids": new_ids,
        "newly_confirmed_ids": newly_confirmed,
        "status": st,
    }

    if args.gt:
        gt = load_gt(args.gt)
        frame_m = match(frame_preds, gt["objects"], 0.15)
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
        conf_objs = [o for o in mem_objs if o.get("confirmation_status") == "confirmed"]
        conf_m = match(
            [
                {
                    "class_name": o["class_name"],
                    "center": np.array(o["position_world"]),
                    "confidence": o.get("observation_quality", 0),
                    "view_id": -1,
                }
                for o in conf_objs
            ],
            gt["objects"],
            0.15,
        )
        keys = ("tp", "fp", "fn", "precision", "unique_recall", "mean_center_error_m")
        report["frame_eval"] = {k: frame_m[k] for k in keys}
        report["memory_eval"] = {k: mem_m[k] for k in keys}
        report["confirmed_eval"] = {k: conf_m[k] for k in keys}

    out_eval = (
        Path(args.out_eval)
        if args.out_eval
        else mem_path.parent / ("memory_update_view%d.json" % args.view_id)
    )
    out_eval.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    print("MEMORY_UPDATE_OK")


if __name__ == "__main__":
    main()
