# REMANI 轨迹可视化（P1）

更新：2026-10-25  
参考：`third_party/wbc/TopAY` `moma_vis` CAD Marker + `remani_real` `PreviewPublisher`（`/remani/candidate_ee_path`）。

## 实现

脚本：`scripts/visualize_remani_trajectory.py`

| 话题 | 颜色（RViz） | 来源 |
|------|--------------|------|
| `/nbv/trajectory/ee_planned` | 黄 | `/planning/trajectory` PolynomialTraj → URDF FK → `cr10_Link6` |
| `/nbv/trajectory/ee_actual` | 青 | TF `world`→`cr10_Link6`（Gazebo/RSP） |
| `/nbv/trajectory/base_actual` | 蓝 | `/ground_truth/state` |
| `/nbv/visualization/nbv_goal` | 绿 Arrow | `/nbv/goal_pose` 或 `/ee_goal` |

RViz：`configs/fkie/agx_adapt/rviz/agx_nbv.rviz` 已增加 Path/Marker 显示。

## 坐标注意

- 规划末端用 URDF FK 到 `cr10_Link6`，与 Gazebo TF 对齐。  
- REMANI `/ee_current_pose` 相对 Link6 约有 `base_z` 偏移，**禁止**与 TF 轨迹直接叠色比较而不标注。

## 实测

- `ee_actual` / `base_actual`：有路径点发布（静止时 base 累积、ee 因 5 mm 下采样可能很少）。  
- `ee_planned`：依赖成功的 `ACTION_ADD` 轨迹。本轮 aisle 上 REMANI 报 `[EE GOAL] IK failed: collision`（simulated 地图占用），规划未发出 → planned 为空。属地图/自碰问题，不是可视化节点故障。
