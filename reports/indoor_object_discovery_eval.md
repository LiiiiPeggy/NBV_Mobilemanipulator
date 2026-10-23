# Indoor object discovery eval (P3-B / P5)

Date: 2026-10-23

## Protocol

- GT: Gazebo-exported poses under `data/scenes/indoor_objects/ycb/` (not `primitive_debug/`)
- Pred: `scripts/perceive_rgbd_objects.py` (formal: `--backend grounded_sam`)
- Memory: `perception/object_memory.py` (assoc 0.25 m)
- Metrics: `scripts/evaluate_object_discovery.py`
  - Primary threshold: **0.15 m**
  - Diagnostics: **0.10 m**, **0.35 m**
- Matching: greedy class + distance; does **not** use Gazebo instance IDs as detector input

## Scene

- Layout: `data/scenes/indoor_objects/ycb/layouts/seed42.json` (table + shelf bay + floor, 11 YCB/local objects + occluder)
- Consistency: `scripts/check_indoor_scene_consistency.py` → `CHECK_INDOOR_SCENE_CONSISTENCY_OK`

## Status

| Item | Status |
|------|--------|
| YCB layout + kind separation | verified |
| Grounded-SAM independence + GPU smoke | verified |
| Full RGB-D live discovery numbers (precision/recall @0.15) | **pending** live `grounded_sam` capture against spawned YCB GT |

Re-run when aisle_safe sim has YCB spawned:

```bash
./scripts/run_object_perception.sh --out data/metrics/indoor_objects/perc_gsam_seed42.json --views 3
python3 - <<'PY'
from perception.object_memory import ObjectMemory
import json
perc=json.load(open('data/metrics/indoor_objects/perc_gsam_seed42.json'))
m=ObjectMemory(); m.ingest_perception_json(perc); m.save('data/metrics/indoor_objects/memory_seed42.json')
PY
python3 scripts/evaluate_object_discovery.py \
  --gt data/scenes/indoor_objects/ycb/gt/seed42_gt.json \
  --pred data/metrics/indoor_objects/perc_gsam_seed42.json \
  --memory-json data/metrics/indoor_objects/memory_seed42.json \
  --dist-thresh 0.15 --diagnostic-thresh 0.10 0.35 \
  --out-json data/metrics/indoor_objects/eval_seed42.json
```
