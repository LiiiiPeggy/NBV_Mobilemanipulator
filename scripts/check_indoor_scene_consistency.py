#!/usr/bin/env python3
"""Ensure primitive_debug vs ycb scene trees are not mixed."""
from __future__ import print_function
import argparse
import hashlib
import json
from pathlib import Path


def kind_of(layout):
    if layout.get("scene_kind"):
        return layout["scene_kind"]
    models = [o.get("model") for o in layout.get("objects", [])]
    if models and all(m == "primitive_sphere" or m is None for m in models if m):
        if any(o.get("source") == "primitive" or o.get("model") == "primitive_sphere" for o in layout.get("objects", [])):
            return "primitive_debug"
    return "ycb_or_unknown"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[1]
    issues = []
    records = []
    paths = list((root / "data/scenes/indoor_objects").rglob("*.json"))
    for p in paths:
        try:
            d = json.loads(p.read_text())
        except Exception as e:
            issues.append({"path": str(p), "error": str(e)})
            continue
        if "objects" not in d and "furniture" not in d:
            continue
        sk = kind_of(d)
        rel = str(p.relative_to(root))
        h = hashlib.sha256(p.read_bytes()).hexdigest()[:12]
        rec = {"path": rel, "scene_id": d.get("scene_id"), "inferred_kind": sk, "hash": h, "n_objects": len(d.get("objects", []))}
        records.append(rec)
        if "primitive_debug" in rel and sk not in ("primitive_debug",):
            issues.append({"path": rel, "issue": "path says primitive_debug but content kind=%s" % sk})
        if "/ycb/" in rel and sk == "primitive_debug":
            issues.append({"path": rel, "issue": "ycb path contains primitive_debug content"})
        # legacy mixed path
        if rel == "data/scenes/indoor_objects/layouts/seed42.json":
            issues.append(
                {
                    "path": rel,
                    "issue": "legacy shared layouts/seed42.json — prefer primitive_debug/ or ycb/ trees",
                    "inferred_kind": sk,
                }
            )

    out = {"records": records, "issues": issues, "ok": len([i for i in issues if "legacy" not in i.get("issue", "")]) == 0}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps({"n_layouts": len(records), "n_issues": len(issues), "ok": out["ok"]}, indent=2))
    print("CHECK_INDOOR_SCENE_CONSISTENCY_OK" if out["ok"] else "CHECK_INDOOR_SCENE_CONSISTENCY_WARN")


if __name__ == "__main__":
    main()
