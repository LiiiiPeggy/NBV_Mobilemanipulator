#!/usr/bin/env python3
"""Compare before/after NBV snapshots: RGB-D freshness, discovery metrics, octomap meta."""
from __future__ import print_function

import argparse
import json
import math
from pathlib import Path

import numpy as np


def load_meta(d):
    return json.loads((Path(d) / "meta.json").read_text())


def load_perception(path):
    if not Path(path).is_file():
        return {"views": []}
    return json.loads(Path(path).read_text())


def run_eval(gt_layout, perc_path, out_json):
    import subprocess

    # evaluate_object_discovery expects perceive_rgbd_objects schema (views[].objects[])
    if not Path(perc_path).is_file():
        Path(out_json).write_text(json.dumps({"tp": 0, "unique_recall": 0.0, "n_pred": 0}, indent=2))
        return 0, "missing_pred", out_json
    cmd = [
        "python3",
        "/workspace/scripts/evaluate_object_discovery.py",
        "--gt",
        gt_layout,
        "--pred",
        str(perc_path),
        "--dist-thresh",
        "0.15",
        "--out-json",
        str(out_json),
    ]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    return r.returncode, (r.stdout or "")[-400:], out_json


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--before", required=True)
    ap.add_argument("--after", required=True)
    ap.add_argument("--layout", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--report", default="reports/fkie_remani_perception_delta.md")
    args = ap.parse_args()

    b = load_meta(args.before)
    a = load_meta(args.after)

    def dist(u, v):
        return math.sqrt(sum((u[i] - v[i]) ** 2 for i in range(3)))

    cam_delta = dist(b["camera_tf"]["xyz"], a["camera_tf"]["xyz"])
    stamp_delta = None
    if b.get("rgb_stamp") is not None and a.get("rgb_stamp") is not None:
        stamp_delta = a["rgb_stamp"] - b["rgb_stamp"]

    # Image content change (mean abs diff)
    import cv2

    rb = cv2.imread(str(Path(args.before) / ("rgb_%s.png" % b["tag"])))
    ra = cv2.imread(str(Path(args.after) / ("rgb_%s.png" % a["tag"])))
    img_mad = None
    if rb is not None and ra is not None and rb.shape == ra.shape:
        img_mad = float(np.mean(np.abs(rb.astype(np.float32) - ra.astype(np.float32))))

    perc_b = Path(args.before) / "perception.json"
    perc_a = Path(args.after) / "perception.json"
    eval_b = Path(args.out).parent / "eval_before.json"
    eval_a = Path(args.out).parent / "eval_after.json"
    run_eval(args.layout, perc_b, eval_b)
    run_eval(args.layout, perc_a, eval_a)
    mb = json.loads(eval_b.read_text()) if eval_b.is_file() else {}
    ma = json.loads(eval_a.read_text()) if eval_a.is_file() else {}
    nb = sum(len(v.get("objects", [])) for v in load_perception(perc_b).get("views", []))
    na = sum(len(v.get("objects", [])) for v in load_perception(perc_a).get("views", []))

    def metric(d, key, default=0):
        if key == "recall":
            return d.get("unique_recall", d.get("recall", default))
        return d.get(key, d.get("metrics", {}).get(key, default))

    delta = {
        "goal_id": a.get("goal_id") or b.get("goal_id"),
        "camera_delta_m": cam_delta,
        "rgb_stamp_delta_s": stamp_delta,
        "image_mad": img_mad,
        "new_observation": bool(
            (stamp_delta is not None and stamp_delta > 0.05)
            and (cam_delta > 0.02)
            and (img_mad is None or img_mad > 1.0)
        ),
        "octomap_before": b.get("octomap"),
        "octomap_after": a.get("octomap"),
        "octomap_data_len_delta": (
            (a.get("octomap") or {}).get("data_len", 0)
            - (b.get("octomap") or {}).get("data_len", 0)
        ),
        "n_pred_before": nb,
        "n_pred_after": na,
        "eval_before": {k: mb.get(k) for k in ("tp", "fp", "fn", "unique_recall", "n_pred", "n_gt")},
        "eval_after": {k: ma.get(k) for k in ("tp", "fp", "fn", "unique_recall", "n_pred", "n_gt")},
        "recall_before": metric(mb, "recall"),
        "recall_after": metric(ma, "recall"),
        "tp_before": metric(mb, "tp"),
        "tp_after": metric(ma, "tp"),
        "note": (
            "FKIE optimizes geometric info gain; TP may not increase on a single step. "
            "new_observation asserts RGB-D+TF freshness, not discovery improvement."
        ),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(delta, indent=2))

    lines = [
        "# FKIE–REMANI perception delta",
        "",
        "goal_id: `%s`" % delta["goal_id"],
        "",
        "| Item | Value |",
        "|------|-------|",
        "| camera Δ (m) | %.4f |" % (delta["camera_delta_m"] or -1),
        "| rgb stamp Δ (s) | %s |" % delta["rgb_stamp_delta_s"],
        "| image MAD | %s |" % delta["image_mad"],
        "| new_observation | **%s** |" % delta["new_observation"],
        "| octomap data_len Δ | %s |" % delta["octomap_data_len_delta"],
        "| TP before→after | %s → %s |" % (delta["tp_before"], delta["tp_after"]),
        "| recall before→after | %s → %s |" % (delta["recall_before"], delta["recall_after"]),
        "",
        delta["note"],
        "",
        "PERCEPTION_DELTA_OK" if delta["new_observation"] else "PERCEPTION_DELTA_WEAK",
    ]
    Path(args.report).parent.mkdir(parents=True, exist_ok=True)
    Path(args.report).write_text("\n".join(lines) + "\n")
    print(json.dumps(delta, indent=2))
    print(lines[-1])


if __name__ == "__main__":
    main()
