#!/usr/bin/env python3
"""Per-GT-object Grounded-SAM failure analysis (types A–H). Offline 2D + optional ROS localize.

Does not feed GT into the detector. GT is used only in this offline analyzer.
"""
from __future__ import print_function
import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# Frozen synonym map (eval-side only). Detector still outputs DEFAULT_GSAM_CLASSES tokens.
CLASS_ALIASES = {
    "banana": {"banana"},
    "apple": {"apple"},
    "bowl": {"bowl"},
    "bottle": {"bottle", "mustard bottle", "spray bottle", "windex"},
    "box": {"box", "cracker box", "sugar box", "cereal box", "cardboard box"},
    "can": {"can", "soup can", "tin can"},
    "cone": {"cone", "traffic cone"},
}


def classes_compatible(gt_cls, pred_cls):
    gt_cls = gt_cls.lower().strip()
    pred_cls = pred_cls.lower().strip()
    if gt_cls == pred_cls:
        return True
    for canon, aliases in CLASS_ALIASES.items():
        if gt_cls == canon or gt_cls in aliases:
            if pred_cls == canon or pred_cls in aliases:
                return True
    return False


def project_point(K, T_w_c, xyz_w):
    """T_w_c: 4x4 world<-cam? We have T_world_cam; p_cam = inv(T) @ p_w."""
    T_c_w = np.linalg.inv(T_w_c)
    pc = T_c_w.dot(np.array([xyz_w[0], xyz_w[1], xyz_w[2], 1.0]))
    if pc[2] <= 0.05:
        return None, pc
    u = K[0, 0] * pc[0] / pc[2] + K[0, 2]
    v = K[1, 1] * pc[1] / pc[2] + K[1, 2]
    return (float(u), float(v)), pc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rgb", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--pred-localized", default="", help="pred JSON with center_world if available")
    ap.add_argument("--pred-2d", default="", help="optional offline 2D pred; else run GSAM")
    ap.add_argument("--classes", default="banana,apple,bottle,bowl,box,can,cup")
    ap.add_argument("--dist-thresh", type=float, default=0.15)
    ap.add_argument("--out-json", required=True)
    ap.add_argument("--out-vis", default="")
    ap.add_argument("--K", default="", help="optional 3x3 row-major JSON; else estimate from image size")
    ap.add_argument("--T-world-cam", default="", help="optional 4x4 JSON; if missing visibility from FOV skipped")
    args = ap.parse_args()

    bgr = cv2.imread(args.rgb)
    if bgr is None:
        raise SystemExit("bad rgb")
    h, w = bgr.shape[:2]
    gt = json.loads(Path(args.gt).read_text())
    gts = gt["objects"]

    if args.pred_2d and Path(args.pred_2d).is_file():
        pred2d = json.loads(Path(args.pred_2d).read_text())
    else:
        from perception.rgbd_object_detector import GroundedSAMDetector

        classes = [c.strip() for c in args.classes.split(",") if c.strip()]
        det = GroundedSAMDetector()
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = det.run(rgb, classes)
        objs = []
        for i, cid in enumerate(res.class_id):
            cls = classes[int(cid)] if 0 <= int(cid) < len(classes) else "obj"
            objs.append(
                {
                    "class_name": cls,
                    "confidence": float(res.confidence[i]),
                    "bbox_xyxy": [float(v) for v in res.xyxy[i]],
                    "has_mask": res.masks is not None,
                }
            )
        pred2d = {"views": [{"view_id": 0, "objects": objs}]}

    pred3d = None
    if args.pred_localized and Path(args.pred_localized).is_file():
        pred3d = json.loads(Path(args.pred_localized).read_text())

    dets2d = pred2d["views"][0]["objects"]
    dets3d = pred3d["views"][0]["objects"] if pred3d else []

    K = None
    T = None
    if args.K:
        K = np.array(json.loads(args.K), dtype=float).reshape(3, 3)
    else:
        # D455 approximate for 640-ish; scale by image
        fx = 385.0 * (w / 640.0)
        fy = 385.0 * (h / 480.0)
        K = np.array([[fx, 0, w / 2.0], [0, fy, h / 2.0], [0, 0, 1.0]])
    if args.T_world_cam:
        T = np.array(json.loads(args.T_world_cam), dtype=float).reshape(4, 4)

    used_pred = set()
    rows = []
    for gi, g in enumerate(gts):
        row = {
            "gt_instance_id": g.get("instance_id") or g.get("gazebo_name") or ("gt_%02d" % gi),
            "gt_class": g["class_name"],
            "gt_xyz": g.get("center_xyz") or [g["spawn"]["x"], g["spawn"]["y"], g["spawn"]["z"]],
            "visible_in_fov": None,
            "proj_uv": None,
            "gdino_candidate": False,
            "gdino_class": None,
            "gdino_conf": None,
            "bbox_xyxy": None,
            "sam_mask_ok": None,
            "depth_valid": None,
            "pred_xyz": None,
            "err_m": None,
            "matched": False,
            "label": "FN",
            "failure_type": None,
        }
        xyz = row["gt_xyz"]
        visible = None
        if T is not None:
            uv, pc = project_point(K, T, xyz)
            if uv is None or pc[2] <= 0.1:
                visible = False
            else:
                u, v = uv
                visible = 0 <= u < w and 0 <= v < h and 0.3 < pc[2] < 4.5
                row["proj_uv"] = [u, v]
                row["depth_cam"] = float(pc[2])
        row["visible_in_fov"] = visible

        # Best 2D candidate by class + spatial proximity to projection
        best = None
        for pi, p in enumerate(dets2d):
            if not classes_compatible(g["class_name"], p["class_name"]):
                continue
            x1, y1, x2, y2 = p["bbox_xyxy"]
            cu, cv = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
            score = float(p.get("confidence", 0))
            if row["proj_uv"] is not None:
                du = cu - row["proj_uv"][0]
                dv = cv - row["proj_uv"][1]
                dist_px = math.hypot(du, dv)
                score = score - 0.001 * dist_px
                if dist_px > 120:
                    continue
            if best is None or score > best[0]:
                best = (score, pi, p)
        if best is not None:
            row["gdino_candidate"] = True
            row["gdino_class"] = best[2]["class_name"]
            row["gdino_conf"] = best[2].get("confidence")
            row["bbox_xyxy"] = best[2]["bbox_xyxy"]
            row["sam_mask_ok"] = bool(best[2].get("has_mask", True))
            row["_pred2d_idx"] = best[1]

        # 3D match
        best3 = None
        for pi, p in enumerate(dets3d):
            if pi in used_pred:
                continue
            if not classes_compatible(g["class_name"], p["class_name"]):
                continue
            err = float(np.linalg.norm(np.array(p["center_world"]) - np.array(xyz)))
            if best3 is None or err < best3[0]:
                best3 = (err, pi, p)
        if best3 is not None and best3[0] <= args.dist_thresh:
            used_pred.add(best3[1])
            row["matched"] = True
            row["label"] = "TP"
            row["err_m"] = best3[0]
            row["pred_xyz"] = best3[2]["center_world"]
            row["depth_valid"] = True
            row["failure_type"] = None
        else:
            # Assign failure type
            if visible is False:
                row["failure_type"] = "A_out_of_view"
            elif not row["gdino_candidate"]:
                row["failure_type"] = "B_gdino_miss" if visible is not False else "A_out_of_view"
            elif row["gdino_class"] and not classes_compatible(g["class_name"], row["gdino_class"]):
                row["failure_type"] = "C_class_mismatch"
            elif best3 is None:
                # had 2D but no 3D
                if row["gdino_candidate"]:
                    row["failure_type"] = "E_depth_invalid"
                    row["depth_valid"] = False
                else:
                    row["failure_type"] = "B_gdino_miss"
            elif best3[0] > args.dist_thresh:
                row["failure_type"] = "F_localization_error"
                row["err_m"] = best3[0]
                row["pred_xyz"] = best3[2]["center_world"]
            else:
                row["failure_type"] = "G_mapping_or_assoc"
            row["label"] = "FN"
        rows.append(row)

    # Extra preds → FP / H
    fps = []
    for pi, p in enumerate(dets3d if dets3d else dets2d):
        if pi in used_pred:
            continue
        fps.append(
            {
                "pred_class": p["class_name"],
                "confidence": p.get("confidence"),
                "center_world": p.get("center_world"),
                "bbox_xyxy": p.get("bbox_xyxy"),
                "failure_type": "H_false_positive",
            }
        )

    n_tp = sum(1 for r in rows if r["label"] == "TP")
    summary = {
        "n_gt": len(rows),
        "n_tp": n_tp,
        "n_fn": len(rows) - n_tp,
        "n_fp": len(fps),
        "n_det_2d": len(dets2d),
        "n_det_3d": len(dets3d),
        "failure_counts": {},
        "dist_thresh_m": args.dist_thresh,
        "class_aliases_frozen": {k: sorted(v) for k, v in CLASS_ALIASES.items()},
    }
    for r in rows:
        ft = r["failure_type"] or "TP"
        summary["failure_counts"][ft] = summary["failure_counts"].get(ft, 0) + 1
    summary["failure_counts"]["H_false_positive"] = len(fps)

    def _sanitize(o):
        if isinstance(o, dict):
            return {k: _sanitize(v) for k, v in o.items() if not str(k).startswith("_")}
        if isinstance(o, list):
            return [_sanitize(v) for v in o]
        if isinstance(o, (np.floating, float)):
            return float(o)
        if isinstance(o, (np.integer, int)):
            return int(o)
        if isinstance(o, (np.bool_, bool)):
            return bool(o)
        return o

    out = {"summary": summary, "per_object": rows, "false_positives": fps}
    Path(args.out_json).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out_json).write_text(json.dumps(_sanitize(out), indent=2))

    vis = bgr.copy()
    for r in rows:
        color = (0, 255, 0) if r["label"] == "TP" else (0, 0, 255)
        if r.get("bbox_xyxy"):
            x1, y1, x2, y2 = map(int, r["bbox_xyxy"])
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        if r.get("proj_uv"):
            u, v = map(int, r["proj_uv"])
            cv2.circle(vis, (u, v), 5, (255, 255, 0), -1)
            cv2.putText(vis, r["gt_class"][:6], (u + 4, v), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    out_vis = args.out_vis or str(Path(args.out_json).with_suffix(".vis.png"))
    cv2.imwrite(out_vis, vis)
    print(json.dumps(summary, indent=2))
    print("Wrote", args.out_json, out_vis)
    print("ANALYZE_GSAM_FAILURES_OK")


if __name__ == "__main__":
    main()
