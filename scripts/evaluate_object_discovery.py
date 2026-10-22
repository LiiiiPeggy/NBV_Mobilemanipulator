#!/usr/bin/env python3
"""Independent evaluator: perception JSON vs Gazebo GT JSON.

Does not feed GT back into planners. One-to-one greedy matching by class + distance.
"""
from __future__ import print_function
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def collect_preds(perc):
    preds = []
    for view in perc.get("views", []):
        for o in view.get("objects", []):
            preds.append(
                {
                    "class_name": o["class_name"],
                    "center": np.array(o["center_world"], dtype=np.float64),
                    "confidence": o.get("confidence", 0.0),
                    "view_id": view.get("view_id", 0),
                }
            )
    return preds


def match(preds, gts, dist_thresh):
    # Unique discovery: match each GT at most once across all preds
    used_gt = set()
    used_pred = set()
    matches = []
    # sort preds by confidence
    order = sorted(range(len(preds)), key=lambda i: -preds[i]["confidence"])
    for pi in order:
        p = preds[pi]
        best_j, best_d = None, 1e9
        for j, g in enumerate(gts):
            if j in used_gt:
                continue
            if g["class_name"] != p["class_name"]:
                continue
            d = float(np.linalg.norm(p["center"] - np.array(g["center_xyz"])))
            if d < best_d:
                best_d, best_j = d, j
        if best_j is not None and best_d <= dist_thresh:
            used_gt.add(best_j)
            used_pred.add(pi)
            matches.append({"pred": pi, "gt": best_j, "dist": best_d, "class": p["class_name"]})
    tp = len(matches)
    fp = len(preds) - len(used_pred)
    fn = len(gts) - len(used_gt)
    precision = tp / float(tp + fp) if (tp + fp) else 0.0
    recall = tp / float(tp + fn) if (tp + fn) else 0.0
    mean_err = float(np.mean([m["dist"] for m in matches])) if matches else None
    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "unique_recall": recall,
        "mean_center_error_m": mean_err,
        "matches": matches,
        "dist_thresh_m": dist_thresh,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--pred", required=True)
    ap.add_argument("--dist-thresh", type=float, default=0.15)
    ap.add_argument("--out-json", required=True)
    ap.add_argument("--out-csv", default="")
    args = ap.parse_args()

    gt = json.loads(Path(args.gt).read_text())
    pred = json.loads(Path(args.pred).read_text())
    gts = gt["objects"]
    preds = collect_preds(pred)
    metrics = match(preds, gts, args.dist_thresh)
    metrics["n_gt"] = len(gts)
    metrics["n_pred"] = len(preds)
    metrics["scene_id"] = gt.get("scene_id")
    Path(args.out_json).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out_json).write_text(json.dumps(metrics, indent=2))
    print(json.dumps({k: metrics[k] for k in ("tp", "fp", "fn", "precision", "unique_recall", "mean_center_error_m")}, indent=2))
    if args.out_csv:
        with open(args.out_csv, "w") as f:
            w = csv.DictWriter(
                f,
                fieldnames=["tp", "fp", "fn", "precision", "unique_recall", "mean_center_error_m", "dist_thresh_m"],
            )
            w.writeheader()
            w.writerow({k: metrics.get(k) for k in w.fieldnames})
    print("EVALUATE_OBJECT_DISCOVERY_OK")


if __name__ == "__main__":
    main()
