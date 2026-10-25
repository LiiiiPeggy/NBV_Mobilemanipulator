# PROGRESS.md — 当前状态

更新时间：2026-10-25  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX + FKIE/SSMI；YCB 多对象基准；REMANI Mode B 闭环。

**当前阶段**：P0 Mode B 全身抖动已定量修复并通过静止门禁；P1 轨迹可视化节点与 RViz 配置已接入；P2 到达门禁→GSAM 触发链路已实测通过。C1–C4 单步闭环仍有效。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| Mode B B1–B4 | 已验证 | `reports/agx_mode_b_physics.md` 等 |
| Mode B + seed42 YCB | 已验证 | `reports/mode_b_ycb_scene.md` |
| C1–C4 自过滤+FKIE 单步 | 已验证 | `reports/fkie_remani_one_step_execution.md` 等 |
| **P0 抖动修复** | **已验证（静止）** | `reports/agx_mode_b_jitter_diagnosis.md`，`jitter_before_after.csv`：`after_idle_v5` / `expB_both` / `expC_post` → `ok_idle=True` |
| **P1 轨迹可视化** | **节点已验证发布** | `reports/remani_trajectory_visualization.md`；planned 依赖 REMANI 出轨 |
| **P2 到达后 GSAM** | **门禁+推理已验证** | `reports/gsam_nbv_arrival_trigger.md`，`GSAM_ARRIVAL_OK` |

## 阻塞 / 未完成

| 项 | 说明 |
|----|------|
| 完整轮式动力学 | Mode B 仍为运动学桥；idle 依赖权威 `set_model_state` |
| REMANI aisle EE | 本轮 snap 后仍见 `[EE GOAL] IK failed: collision`（simulated 图）；planned Path 空 |
| Octomap 快照话题 | `/octomap_binary` 曾超时 |
| 多步 NBV+YCB+GSAM 记忆 | 到达触发已通；完整对象记忆融合待带 YCB 场景重跑 |

## 下一步

1. 清理/重建 simulated 地图或 aisle 自碰，恢复 REMANI 小目标 `EXEC_TRAJ` 与 `ee_planned`。  
2. `LOAD_YCB_SCENE=1` 下跑到达→GSAM→object memory→下一 NBV。  
3. Mode A / B4 抽检回归。
