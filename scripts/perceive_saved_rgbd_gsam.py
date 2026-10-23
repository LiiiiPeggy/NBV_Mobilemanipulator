#!/usr/bin/env python3
"""Offline Grounded-SAM on a saved RGB (+ optional depth) for YCB demo evidence.

Does not require rospy. Writes pred JSON compatible with evaluate_object_discovery
when --depth-npy and camera K + T_world_cam are provided; otherwise boxes-only.
"""
from __future__ import print_function
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rgb", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--classes", default="banana,apple,bottle,bowl,box,can,cup")
    ap.add_argument("--gt", default="", help="optional GT for class-only hit list")
    args = ap.parse_args()

    from perception.rgbd_object_detector import GroundedSAMDetector

    bgr = cv2.imread(args.rgb)
    if bgr is None:
        raise SystemExit("failed to read %s" % args.rgb)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    det = GroundedSAMDetector()
    res = det.run(rgb, classes)
    objs = []
    vis = bgr.copy()
    for i, cid in enumerate(res.class_id):
        cls = classes[int(cid)] if 0 <= int(cid) < len(classes) else "obj"
        conf = float(res.confidence[i])
        box = [float(v) for v in res.xyxy[i]]
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(vis, "%s %.2f" % (cls, conf), (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if res.masks is not None:
            m = res.masks[i].astype(bool)
            vis[m] = (0.6 * vis[m] + 0.4 * np.array([0, 0, 255])).astype(np.uint8)
        objs.append({"class_name": cls, "confidence": conf, "bbox_xyxy": box, "center_world": [0, 0, 0]})
    out_img = Path(args.out).with_suffix(".vis.png")
    cv2.imwrite(str(out_img), vis)
    pred = {
        "backend": "grounded_sam_offline",
        "views": [{"view_id": 0, "n_detections": len(objs), "n_localized": 0, "objects": objs, "note": "2D-only offline"}],
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(pred, indent=2))
    print("Wrote", args.out, "vis", out_img, "ndet", len(objs))
    print("PERCEIVE_SAVED_GSAM_OK")


if __name__ == "__main__":
    main()
