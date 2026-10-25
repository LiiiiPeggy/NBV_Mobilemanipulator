# PROGRESS.md — 当前状态

更新时间：2026-10-25  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX + FKIE/SSMI；YCB 多对象基准；REMANI Mode B 闭环。

**当前阶段**：E0–E3 保持；**F1–F3 主路径已接通**（空目标诊断 + 同帧 GSAM + 单次在线对象闭环）。F4 调度器已落地，连续到位仍受 FKIE 朝向/REMANI 可达性限制。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| E0–E3（OctoMap/REMANI/轨迹） | 已验证 | 见既有 `reports/octomap_*`、`remani_simulated_*` |
| **F1 FKIE goals=[] 分类** | **已验证** | `reports/fkie_empty_goals_diagnosis.md`；主因常为 `G_BASE_POSE`；低 z 判 `D_FILTERED` |
| **F2 同步快照 GSAM** | **已验证** | `reports/gsam_synchronized_snapshot.md`；同 `snapshot_id` 检测+定位 |
| **F3 单次在线对象闭环** | **已验证** | `reports/fkie_online_gsam_one_step.md`；`SINGLE_EE_EXEC_OK` ee_err≈1.4 cm；记忆 TP 差分 |
| F4 多步调度器 | 部分 | `reports/fkie_multistep_exploration.md`；可合法终止；未凑满 3 次到位 |
| F5 曲线/HUD | 部分 | `summarize_online_exploration.py` + `/nbv/online/status_text` |

## 阻塞 / 未完成

| 项 | 说明 |
|----|------|
| 连续 3–5 次全部到位 | FKIE 多为 base-pose；部分目标 REMANI 跟踪失败 |
| FKIE `goals[]` 臂分支 | RRT best_branch 常被增益过滤 → `request_base_pose` |
| 正式召回—步数曲线 | 需更多成功到位步；禁止假目标填充 |
| 完整轮式动力学 | Mode B 仍为运动学桥 |

## 下一步

1. 提升 FKIE 视点朝向（optical 俯仰）或筛选高增益臂目标。  
2. 在同一 seed42 上凑齐 ≥3 次 `SINGLE_EE_EXEC_OK` + GSAM 记忆曲线。  
3. Mode A / idle jitter 抽检回归。
