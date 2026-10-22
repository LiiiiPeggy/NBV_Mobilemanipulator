# Indoor multi-object min loop (P2 / P3 / P5 / P4-A)

**Date**: 2026-10-22  
**Branch**: `cursor/fkie-ssmi-effective-validation`  
**Baseline commit**: `8810e5d` (Mode A warehouse NBV goals=1 — re-verified this session)

## Commands (container `nbv_fkie`)

```bash
# Layout (seed-reproducible)
python3 scripts/generate_object_scene.py --seed 42

# After AGX Mode A warehouse is up:
# Visible ROI for Mode A Gazebo camera was found empirically near (-6.5,-7.0,1.0)
# (see PROCESS: Mode A TF vs Gazebo joint freeze)

python3 scripts/export_object_gt.py \
  --layout data/scenes/indoor_objects/layouts/seed42.json \
  --out data/scenes/indoor_objects/gt/seed42_gt.json

python3 scripts/perceive_rgbd_objects.py \
  --backend color --world-tf-mode calib \
  --calib-json data/metrics/indoor_objects/T_world_cam_calib.json \
  --out data/metrics/indoor_objects/seed42_pred.json

python3 scripts/evaluate_object_discovery.py \
  --gt data/scenes/indoor_objects/gt/seed42_gt.json \
  --pred data/metrics/indoor_objects/seed42_pred.json \
  --dist-thresh 0.35 \
  --out-json data/metrics/indoor_objects/seed42_eval.json

python3 scripts/nbv_to_remani_ee_goal.py \
  --cam-xyz -4.419 -6.198 1.609 --cam-rpy 0 0.35 0 \
  --out data/metrics/nbv_remani/ee_goal_seed42.json
```

## Results (evidence-backed)

| Stage | Result |
|-------|--------|
| P2 spawn | 10 objects + optional furniture/occluder; `EXPORT_OBJECT_GT_OK` n=10 |
| P2 visibility | `reports/indoor_objects_shots/multi_objects_visible.png`, `found_red.png` |
| P3 detect+localize | `PERCEIVE_RGBD_OK`; 16 color detections → localized |
| P5 eval (thresh **0.35 m**, pre-experiment) | tp=2, fp=14, fn=8, precision=0.125, unique_recall=0.2, mean_err≈0.178 m |
| P4-A transform | `NBV_TO_REMANI_EE_GOAL_OK`; EE ≈ `(-4.435,-6.123,1.571)` quat ok |
| P4-B execute | **Not done** — Mode A static; REMANI planner node not launched |
| FKIE regression | Warehouse NBV still `goals=1` after restart this session |

Frozen eval threshold for this pre-experiment: **0.35 m** (not 0.1 m). Formal experiments must freeze threshold before scoring.

## Artifacts

- Layout/GT: `data/scenes/indoor_objects/`
- Scripts: `generate_object_scene.py`, `spawn_object_scene.py`, `spawn_visible_primitives.py`, `export_object_gt.py`, `perceive_rgbd_objects.py`, `evaluate_object_discovery.py`, `nbv_to_remani_ee_goal.py`, `set_agx_observation_pose.py`, `fetch_object_models.sh`
- Models: YCB under `third_party/gazebo_objects/` (gitignored); local table/cone/occluder in `configs/fkie/agx_adapt/models/`
- Copies under `reports/indoor_objects_*` for git-tracked evidence

## Known blockers

1. **Mode A `<static>true>` freezes Gazebo arm joints**; `/joint_states` TF used by RViz/NBV does **not** move the Gazebo D455. Object placement for RGB visibility must follow the Gazebo camera frustum, not TF alone.
2. Grounded-SAM-2 / YOLO not installed in `nbv_fkie` (no torch); color/depth backends used.
3. P4-B REMANI execution requires Mode B / REMANI stack; only pose conversion dry-run completed.
