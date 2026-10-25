# PROGRESS.md — 当前状态

更新时间：2026-10-26  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX + FKIE/SSMI；YCB 多对象基准；REMANI Mode B 闭环。

**当前阶段**：E0–E3 / F1–F3 保持；**可达性简化 + G_BASE_POSE 分流 + seed42 连续 3 步在线发现已验证**。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| E0–E3（OctoMap/REMANI/轨迹） | 已验证 | 见既有 `reports/octomap_*`、`remani_simulated_*` |
| F1–F3 空目标/同帧 GSAM/单步在线 | 已验证 | `reports/fkie_empty_goals_diagnosis.md` 等 |
| **取消 0.85–1.75 高度硬过滤** | **已验证** | `save_fkie_action_result.py` 仅结构检查；step2 z≈1.34–1.53 进 REMANI |
| **G_BASE_POSE 底盘执行** | **已验证** | `execute_base_pose_goal.py`；run `…_seed42_reach` step1/3 `BASE_POSE_EXEC_OK` |
| **连续 3 步 NBV + GSAM 记忆** | **已验证** | `reports/fkie_multistep_reachability_and_discovery.md`；2×base + 1×camera；confirmed Recall≈0.27 |

## 阻塞 / 未完成

| 项 | 说明 |
|----|------|
| seed7 / seed99 复现 | 待跑 |
| FKIE 相机目标朝向质量 | yaw-only 视场仍常见；靠多候选 REMANI |
| 完整轮式动力学 | Mode B 仍为运动学桥 |
| 实机 | 未开展 |

## 下一步

1. 同流程扩展 seed7、seed99。  
2. 视需要将 MAX_STEPS 提到 5（保持 fail-closed）。  
3. Mode A / idle jitter 抽检回归。
