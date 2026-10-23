# YCB Gazebo visibility validation

Date: 2026-10-23  
Commit baseline before fix: `d05f07b`  
Branch: `cursor/fkie-ssmi-effective-validation`

## Diagnosis (before fix)

| Check | Result |
|-------|--------|
| `fkie_agx_warehouse_nbv.sh` auto-spawns YCB? | **No** |
| `run_indoor_objects_min_loop.sh` default | `SCENE_KIND=primitive_debug` |
| Live Gazebo `mo_*` count after AGX bringup | **0** (`MO_COUNT 0`) |
| Classification | **A — spawn never executed** |

World had warehouse shelves/boxes + `agx_ranger_cr10` only.

## Manual probe

Spawned `simple_table`, `mustard_bottle`, `cracker_box` via `scripts/manual_spawn_probe.py`:

- All `spawn_ok=true`, `exists=true`, assets present (`manual_probe.json`)
- Meshes/textures under `third_party/gazebo_objects/gazebo_ycb/models/*/meshes/` OK

## Root cause of “D455 sees nothing” after spawn

First layout placed the table at `(-2.55,-6.55)` while aisle_safe NBV arm optical axis looked roughly **opposite** (~140° off table).  
Models **existed in Gazebo** but were outside the D455 FOV (**case F**).

Fix:

1. Profile `spawn_warehouse_aisle_safe_ycb.yaml` + `arm_observe_table.yaml` (base unchanged).
2. FOV-aligned layout: table ~`(-4.05,-4.70)`, objects along optical ray (table ang ≈5.5°).

## One-click entry

```bash
FORCE_RESTART=1 ./scripts/start_fkie_agx_ycb_demo.sh
# defaults: SCENE_KIND=ycb SEED=42 SPAWN_PROFILE=aisle_safe_ycb AGX_MODE=static
```

`fkie_agx_warehouse_nbv.sh` remains the empty-warehouse FKIE regression (no YCB hardcode in world). Supports `SKIP_NBV_ACTION=1`.

## Spawn / verify (seed42)

```
SPAWN_OBJECT_SCENE_OK 13 / 13
VERIFY_GAZEBO_OBJECT_SCENE_OK  # n_targets_exist=11, n_exist=13, n_below_ground=0
EXPORT_OBJECT_GT_OK            # n=11 expected=11 missing=[]
```

Artifacts:

- `data/scenes/indoor_objects/ycb/layouts/seed42.json`
- `data/scenes/indoor_objects/ycb/layouts/seed42.spawned.json`
- `data/scenes/indoor_objects/ycb/gt/seed42_gt.json`
- `data/metrics/indoor_objects/ycb/seed42_verify.json`

## Visual evidence

| Shot | Path |
|------|------|
| D455 RGB (table+YCB visible) | `reports/ycb_gazebo_shots/d455_rgb_seed42.png` |
| D455 Depth | `reports/ycb_gazebo_shots/d455_depth_seed42.png` |
| Desktop after spawn | `reports/ycb_gazebo_shots/desktop_after_fov_spawn.png` |
| GSAM overlay | `reports/ycb_gazebo_shots/seed42_pred_gsam_offline.vis.png` |

D455 RGB clearly shows table, sugar box, can, bowl, banana, apple, gelatin box, traffic cone.

## Grounded-SAM (NBV-local, no DovSG)

- Offline detect on saved RGB: **7** boxes (`seed42_pred_gsam_offline.json`)
- ROS depth/TF localize: `seed42_pred_gsam.json`
- Eval @0.15 m: **tp=2 fp=5 fn=9**, precision≈0.29, unique_recall≈0.18, **mean_err≈0.023 m**
- Diagnostics written with 0.10 / 0.35 in eval JSON

Color heuristic is debug-only (high FP); formal numbers above use grounded_sam.

## Hardened tooling

- `spawn_object_scene.py` — fail-closed, post-existence + pose check, asset URI check
- `verify_gazebo_object_scene.py` — runtime acceptance
- `export_object_gt.py` — fail-closed if targets missing
- `scripts/start_fkie_agx_ycb_demo.sh` — one-click AGX+YCB

## FKIE regression note

Empty-warehouse aisle_safe NBV (`goals=1`) remains a separate path via `SPAWN_PROFILE=aisle_safe` without YCB spawn. YCB demo uses `aisle_safe_ycb` observe arm bake and does not embed objects into the world SDF.
