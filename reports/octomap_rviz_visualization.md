# OctoMap / 点云 RViz 显示

Date: 2026-10-25

Fixed Frame: `world`

| Display | Topic | 默认 |
|---------|-------|------|
| DepthCloud_current | `/realsense/depth/points2` | 开 |
| Octomap_occupied_accumulated | `/robot/octomap_full` | 开 |
| Octomap_occupied_cells | `/robot/occupied_cells_vis_array` | 开 |
| Octomap_free_accumulated | `/robot/free_cells_vis_array` | 关 |
| EE_Planned / EE_Actual / Base_Actual | `/nbv/trajectory/*` | 开 |
| Debug_RRT 等 | `/robot/viz_*` | 多关闭 |

配置文件：`configs/fkie/agx_adapt/rviz/agx_nbv.rviz`

说明：只开占据体素时，已扫描的自由空间看起来像「没建图」——需切换 free 显示或看 `octomap_full` 体积增长。
