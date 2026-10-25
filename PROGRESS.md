# PROGRESS.md — 当前状态

更新时间：2026-10-25  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX + FKIE/SSMI；YCB 多对象基准；REMANI Mode B 闭环。

**当前阶段**：C1 自过滤 + C2 simulated 地图验收已通过；C3/C4 完成一次 FKIE Action→REMANI EXEC_TRAJ→新 RGB-D（`ONE_STEP_OK` / `PERCEPTION_DELTA_OK`）。B4 `static_empty` 仍为运动管线隔离测试。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| Mode B B1–B4 | 已验证 | `reports/agx_mode_b_physics.md` 等 |
| Mode B + seed42 YCB | 已验证 | `reports/mode_b_ycb_scene.md` |
| C1 自身点云过滤+stamp TF | 已验证 | `reports/remani_self_filter_validation.md` |
| C2 simulated 地图（起始无误自碰） | 已验证 | `reports/remani_simulated_map_validation.md` |
| C3 FKIE 完整目标传递 | 已验证 | `reports/fkie_goal_semantics_validation.md` |
| C4 单步 NBV 新观测 | 已验证 | `reports/fkie_remani_one_step_execution.md`, `fkie_remani_perception_delta.md` |

## 阻塞 / 未完成

| 项 | 说明 |
|----|------|
| 完整轮式动力学 | Mode B 仍为运动学桥 |
| 深度地图非全图先验 | 仅 D455 FOV 累积；货架区可能未入图 |
| Octomap 快照话题 | `/octomap_binary` 本轮超时未取到 |

## 下一步

1. 增强局部自由空间/射线融合，减少中途规划撞桌误路径。  
2. 在稳定 simulated 地图上重复多步 NBV + GSAM 记忆更新。  
3. Mode A 回归抽检。
