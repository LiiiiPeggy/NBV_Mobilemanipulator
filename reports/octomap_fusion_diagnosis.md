# E1: D455 累计 OctoMap 融合诊断

Date: 2026-10-25

## 数据链（已核实）

```
/realsense/depth/points2  (d455_depth_optical_frame, ~10 Hz)
        ↓ remap cloud_in
/robot/octomap_server
        ↓ TF ray origin at optical frame
/robot/octomap_full | occupied_cells_vis_array | free_cells_vis_array (~10 Hz)
        ↓
FKIE (/robot/fkie_nbv_planner) + RViz
```

要点：点云保持在 optical 帧，由 `octomap_server` 经 TF 取相机原点做射线更新（见 `octomap_agx.launch` 注释）。`publish_free_space:=true` 后自由体素 MarkerArray 正常发布。

## 对照实验（同一次 Gazebo 运行）

固定 seed42 aisle，AGX 原地 yaw +25° 再恢复：

| 指标 | Before | After | Δ |
|------|--------|-------|---|
| occupied Marker 点数 | 2055 | 2258 | +203 |
| free Marker 点数 | 4913 | 6857 | +1944 |
| `octomap_full` bytes | 43000 | 55765 | +12765 |

证据：`data/metrics/fkie_agx_warehouse_mode_b/octomap_fusion_move.json`

静止等待 8s 无运动时 Δ≈0（`octomap_fusion_e1.json`）——说明增长来自新视角，而非每帧清空重建。

## RViz

`configs/fkie/agx_adapt/rviz/agx_nbv.rviz`：

- `DepthCloud_current` ← `/realsense/depth/points2`
- `Octomap_occupied_accumulated` ← `/robot/octomap_full`
- `Octomap_occupied_cells` ← `/robot/occupied_cells_vis_array`（默认开）
- `Octomap_free_accumulated` ← `/robot/free_cells_vis_array`（默认关，避免卡顿）
- Debug_* 组默认隐藏部分 RRT 文字

## 结论

累计 OctoMap 融合链路正确；历史已知空间在视角改变后保留并扩展。无需额外视觉 SLAM（当前依赖 Gazebo 位姿 + TF）。

## 未决

- `/robot/octomap_point_cloud_centers` 本轮未见到发布（可选显示）。
- 快照脚本若订阅 `/octomap_binary`（无 ns）会超时；应使用 `/robot/octomap_binary`。
