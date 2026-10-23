# Multiview object memory (Mode A offline)

Date: 2026-10-23  
Seed: 42 (same layout/GT; three baked arm poses — not autonomous exploration)

## Views

| view_id | tag | arm YAML |
|--------:|-----|----------|
| 0 | front | `arm_observe_aisle_front.yaml` |
| 1 | high | `arm_observe_aisle_high.yaml` |
| 2 | oblique | `arm_observe_aisle_oblique.yaml` |

Bringup: `AGX_ARM_POSE=... SPAWN_PROFILE=aisle_safe_ycb` via `scripts/run_multiview_ycb_capture.sh`.

## Metrics @0.15 m

| Method | TP | FP | FN | Prec | Recall |
|--------|---:|---:|---:|-----:|-------:|
| View0 front | 4 | 7 | 7 | 0.36 | 0.36 |
| View1 high | 3 | 9 | 8 | 0.25 | 0.27 |
| View2 oblique | 5 | 2 | 6 | 0.71 | 0.45 |
| Naive accumulate | 5 | 25 | 6 | 0.17 | 0.45 |
| **Memory (all)** | **5** | **6** | **6** | **0.45** | **0.45** |
| Memory confirmed (≥2 obs) | 4 | 4 | 7 | 0.50 | 0.36 |

Memory reduced duplicate FPs (25→6) while keeping unique TP=5. `n_duplicates_reduced=19`.

## Memory schema

`perception/object_memory.py`: `object_id`, `class_id`, `class_evidence`, `position_world`, `position_uncertainty`, `observation_count`, `last_observed_time`, `last_camera_pose`, `observation_quality`, `confirmation_status`, `association_conflict`. Online IDs only (`mem_*`); GT unused online.

## Artifacts

`reports/multiview_ycb/seed42/` — RGB/depth/cam per view, pred2d/3d, `memory.json`, `multiview_summary.json`.
