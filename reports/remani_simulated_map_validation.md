# C2: REMANI `environment_mode=simulated` map

Date: 2026-10-25 07:52:23 UTC

## Semantics
`simulated` + `use_load_map` + `fsm/global_plan` → GridMap loads filtered `/map_generator/global_cloud` into inflate/ESDF. Launch sets `grid_map/use_global_map` for warehouse.

## Validation
`validate_remani_warehouse_map.py` → **WAREHOUSE_MAP_C2_OK** (n_points=3676)

| Probe | Count | OK |
|-------|-------|----|
| free_aisle_near_robot | 0 | True |
| table_region | 454 | True |
| robot_body_proxy | 0 | True |
| shelf_region | 0 | False |

Start footprint vs filtered cloud: **0 hits** — no false self-collision at start.

## Planning
FKIE Action EE under simulated map reached FSM **EXEC_TRAJ** (see one-step report). Earlier tiny hand-crafted deltas sometimes aborted mid-traj with `BACKEND COLLISION` when the optimizer drove toward table FOV — that is map+planner interaction, not disabled collision. B4 `static_empty` remains motion-pipeline isolation only.

Kinematic bridges (not full wheel dynamics).
