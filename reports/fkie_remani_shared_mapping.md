# FKIE–REMANI 统一环境观测

Date: 2026-10-25

## 权威观测

```
D455 /realsense/depth/points2
  → stamp TF + robot_self_filter (+ accum purge)
  → /map_generator/global_cloud (world, filtered)
       ├─ REMANI GridMap (use_load_map + use_global_map)
       └─ (parallel) octomap_server ← optical cloud + TF
```

FKIE 使用 `/robot/octomap_*`；REMANI 使用同一深度源经几何自过滤后的全局点云，再构建 inflate/ESDF。坐标系均为 `world`。

## 关键修复

1. **`use_global_map` 被 yaml 覆盖**  
   `remani_planner_param.yaml` 默认 `false` → 订阅光学 `/realsense/depth/points2` 当局部图。  
   修复：项目内 `advanced_param_sim_agx.xml` 在节点 rosparam **之后** 强制 `grid_map/use_global_map` / `use_load_map` / `fsm/global_plan`。

2. **自过滤累积幽灵**  
   VoxelAccum 永久保留历史点；机器人附近 ~0.3 m 残留导致 `IK failed: collision`。  
   修复：每帧/定时按当前连杆 TF 再 purge（`accum_purge_extra≈0.18`），并放大底盘代理至 L=1.10×W=0.90。

3. **no odom 竞态**  
   全局云只加载一次；若首帧早于 odom 会丢。重启脚本先等 `/mm/car/odom` 再起 REMANI，并确认 inflate 非空。

## 验证

- `WAREHOUSE_MAP_C2_OK`：aisle 近身 0、table 有点、start_self_ok  
- simulated 小 EE：`SINGLE_EE_EXEC_OK`（见 collision 诊断报告）
