#!/usr/bin/env bash
# One-command: validate layout → (reuse or run) multiview → eval → report.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SEED="${SEED:-42}"
SKIP_CAPTURE="${SKIP_CAPTURE:-0}"
OUT="reports/object_discovery_benchmark"
MV="reports/multiview_ycb/seed${SEED}"
mkdir -p "${ROOT}/${OUT}"
COMMIT="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"

python3 "${ROOT}/scripts/validate_object_layout.py" \
  --layout "${ROOT}/data/scenes/indoor_objects/ycb/layouts/seed${SEED}.json" \
  --out "${ROOT}/${OUT}/seed${SEED}_layout_validate.json"

if [[ "${SKIP_CAPTURE}" != "1" ]]; then
  SEED="${SEED}" bash "${ROOT}/scripts/run_multiview_ycb_capture.sh"
fi

# Merge all pred3d into one pred JSON for evaluator
python3 - <<PY
import json
from pathlib import Path
root = Path("${ROOT}")
mv = root / "${MV}"
views = []
for vid in (0, 1, 2):
    p = mv / ("pred3d_view%d.json" % vid)
    if not p.is_file():
        continue
    data = json.loads(p.read_text())
    cam = json.loads((mv / ("cam_view%d.json" % vid)).read_text())
    for v in data.get("views", []):
        v["view_id"] = vid
        v["camera_pose_xyz"] = cam.get("camera_pose_xyz")
        views.append(v)
pred = {"backend": "grounded_sam+ros_localize", "views": views, "seed": ${SEED}}
(root / "${OUT}/seed${SEED}_pred_multiview.json").write_text(json.dumps(pred, indent=2))
print("n_views", len(views), "n_pred", sum(len(v.get("objects", [])) for v in views))
PY

python3 "${ROOT}/scripts/evaluate_object_discovery.py" \
  --gt "${ROOT}/data/scenes/indoor_objects/ycb/gt/seed${SEED}_gt.json" \
  --pred "${ROOT}/${OUT}/seed${SEED}_pred_multiview.json" \
  --memory-json "${ROOT}/${MV}/memory.json" \
  --dist-thresh 0.15 \
  --out-json "${ROOT}/${OUT}/seed${SEED}_eval.json" \
  --out-csv "${ROOT}/${OUT}/seed${SEED}_eval.csv"

export ROOT OUT SEED COMMIT MV
python3 - <<'PY'
import json, time, os
from pathlib import Path
root = Path(os.environ["ROOT"])
out = os.environ["OUT"]
seed = os.environ["SEED"]
commit = os.environ.get("COMMIT", "unknown")
mv_path = os.environ["MV"]
ev = json.loads((root / out / ("seed%s_eval.json" % seed)).read_text())
mv = {}
ms = root / mv_path / "multiview_summary.json"
if ms.is_file():
    mv = json.loads(ms.read_text())
report = {
    "title": "object_discovery_benchmark",
    "seed": int(seed),
    "git_commit": commit,
    "dist_thresh_m": 0.15,
    "diagnostic_thresh_m": [0.10, 0.35],
    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
    "eval": ev,
    "multiview_summary": mv,
}
(root / out / ("seed%s_benchmark.json" % seed)).write_text(json.dumps(report, indent=2))
mem = ev.get("memory") or {}
conf = ev.get("memory_confirmed") or {}
lines = [
    "# Object discovery benchmark",
    "",
    "Date: %s" % time.strftime("%Y-%m-%d"),
    "Commit: `%s`" % commit,
    "Seed: %s" % seed,
    "",
    "## Formal @0.15 m (multiview accumulate)",
    "",
    "| Metric | Value |",
    "|--------|------:|",
    "| n_gt | %s |" % ev.get("n_gt_total", ev.get("n_gt")),
    "| TP/FP/FN | %s/%s/%s |" % (ev["tp"], ev["fp"], ev["fn"]),
    "| Precision | %.3f |" % ev["precision"],
    "| Unique recall | %.3f |" % ev["unique_recall"],
    "| Mean err (TP) | %s |" % ev.get("mean_center_error_m"),
    "",
    "## Memory",
    "",
    "| set | TP | FP | FN | Prec | Recall |",
    "|-----|---:|---:|---:|-----:|-------:|",
    "| all | %s | %s | %s | %.3f | %.3f |" % (mem.get("tp"), mem.get("fp"), mem.get("fn"), mem.get("precision") or 0, mem.get("unique_recall") or 0),
    "| confirmed | %s | %s | %s | %.3f | %.3f |" % (conf.get("tp"), conf.get("fp"), conf.get("fn"), conf.get("precision") or 0, conf.get("unique_recall") or 0),
    "",
    "Artifacts: `reports/object_discovery_benchmark/`, `reports/multiview_ycb/seed%s/`." % seed,
]
(root / "reports/object_discovery_benchmark.md").write_text("\n".join(lines) + "\n")
print("Wrote reports/object_discovery_benchmark.md")
print("RUN_OBJECT_DISCOVERY_BENCHMARK_OK")
PY
