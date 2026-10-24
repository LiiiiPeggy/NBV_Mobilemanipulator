# REMANI warehouse map (B3)

## Interface

REMANI GridMap (simulated) consumes **`/map_generator/global_cloud`** (PointCloud2), not Octomap.

Feeder: `scripts/warehouse_cloud_to_remani.py` (depth → world + optional synthetic shelves).

## Result (partial)

`data/metrics/fkie_agx_warehouse_mode_b/warehouse_map_b3.json`

- Cloud publishes (~10k pts with synthetic shelves)
- Free aisle probe near robot: 0 pts (good)
- Table region: hits present
- Earlier full depth feed caused **BACKEND COLLISION** at start (robot body in cloud) → planning abort

## B4 choice

Motion proof used `environment_mode:=static_empty` to avoid start-state collision while B3 filtering is incomplete. Warehouse ESDF with robot-body filter remains next work — collision was **not** disabled to force a trajectory.
