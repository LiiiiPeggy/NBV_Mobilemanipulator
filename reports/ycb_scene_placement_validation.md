# YCB scene placement validation

Date: 2026-10-23  
Branch: `cursor/fkie-ssmi-effective-validation`  
Scope: table + floor only (no fake shelf-board heights); warehouse world unchanged.

## Root causes addressed

1. FOV-aligned table at `(-4.05,-4.70)` overlapped `Shelves high` OBB.
2. Layout used fake `z=1.05/1.35` without shelf boards → floating props.
3. Uniform spawn Z ignored per-model collision bottoms.

## Fixes

| Piece | Role |
|-------|------|
| `scripts/ycb_model_contact_z.py` | SDF box/cyl / commented-box / mesh fallback → `z_bottom` |
| `scripts/generate_object_scene.py` | `spawn_z = support_z - z_bottom + eps`; table top from `simple_table` (`table_top_z=0.74`) |
| `scripts/validate_object_layout.py` | Offline: shelf OBB, near-box distance, support Z, on-table XY, object spacing |
| `verify_gazebo_object_scene.py` | Live support-surface Z check |
| `arm_observe_aisle_table.yaml` | Bake look-at clear table `(-3.90,-7.50)` |
| `start_fkie_agx_ycb_demo.sh` | `FORCE_RESTART=1` stops stale gz before rebake |

## Commands

```bash
python3 scripts/generate_object_scene.py --seed 42 --out data/scenes/indoor_objects/ycb/layouts/seed42.json
python3 scripts/validate_object_layout.py --layout data/scenes/indoor_objects/ycb/layouts/seed42.json \
  --out data/metrics/indoor_objects/ycb_placement/seed42_layout_validate.json
# likewise seed7, seed99
FORCE_RESTART=1 SPAWN_PROFILE=aisle_safe_ycb ./scripts/start_fkie_agx_ycb_demo.sh
```

## Results

| Seed | layout validate | spawn | verify | GT export |
|------|-----------------|-------|--------|-----------|
| 42 | OK (0 issues) | 13/13 | OK | 11/11 |
| 7 | OK | 13/13 | OK | 11/11 |
| 99 | OK | 13/13 | OK | 11/11 |

Live seed42: table `(-3.90,-7.50)`; objects on table use `z≈0.74 - z_bottom`; floor props `z≈0.001`; no `mo_*` with mid-air shelf Z.

## Evidence

- D455: `reports/ycb_placement/d455_rgb_seed42.png`
- GUI top/side/close: `reports/ycb_placement/gui_{top,side,close}_seed42.png`
- Desktop: `reports/ycb_placement/desktop_seed42.png`
- Metrics: `data/metrics/indoor_objects/ycb_placement/`

## Note

Nearest warehouse `big_box` remains ~1.9 m from table (background in D455). Table does **not** intersect `Shelves high` OBB.
