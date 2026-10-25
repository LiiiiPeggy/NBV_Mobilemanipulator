# C1: REMANI depth self-filter + stamp TF

Date: 2026-10-25 07:52:23 UTC

## Pipeline
`/realsense/depth/points2` → TF at `msg.header.stamp` (skip if missing; never silent Time(0)) → geometric link proxies (`scripts/robot_self_filter.py`) → voxel accum → `/map_generator/global_cloud`

**Not** a 0.9 m XY radius wipe. Floor returns rejected with `z_min≈0.15` (floor ESDF caused false car collisions at z≈0.05).

## Live result
`data/metrics/fkie_agx_warehouse_mode_b/self_filter_c1.json` → **SELF_FILTER_C1_OK**

| Check | Result |
|-------|--------|
| stamp TF | OK (`use_sim_time=true`) |
| synthetic body on live link TFs removed | 18/18 |
| table/shelf anchors kept | 4/4 |
| near-robot raw (current FOV) | 0 |

Approx: `Mesh→capsule/box: base deck box ±4 cm; wheel spheres +3 cm; CR10 links capsules ≈ REMANI sample envelopes + manipulator_thickness(0.06)+inflate; gripper/camera boxes +3 cm. Not a full convex hull.`

## Debug topics
`/warehouse_map/cloud_raw_world`, `/warehouse_map/cloud_filtered`, `/warehouse_map/self_filter_stats`
