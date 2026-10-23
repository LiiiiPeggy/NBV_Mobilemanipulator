# PROGRESS.md — 当前状态

更新时间：2026-10-23  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX + FKIE/SSMI；YCB 多对象基准；REMANI Mode B 闭环。

**当前阶段**：Stage 0–3 已提交推送（`7c35686`）。Stage 4：REMANI sim `/ee_goal` **committed** 已验证；warehouse Mode B / Stage 5 闭环 **阻塞**（双仿真未融合）。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| 摆放接触高度 + 多 seed | 已验证 | `reports/ycb_scene_placement_validation.md` |
| GSAM 失败分析 | 已验证 | `reports/grounded_sam_object_failure_analysis.md` |
| 多视角记忆 | 已验证 | `reports/multiview_object_memory.md` |
| 发现率基准 | 已验证 | `reports/object_discovery_benchmark.md` |
| REMANI `/ee_goal` 订阅+committed | 已验证 | `reports/remani_mode_b_integration.md`（独立 master + remani_sim） |

## 阻塞

| 项 | 说明 |
|----|------|
| Warehouse Mode B 真运动 | REMANI fake_mm ≠ FKIE Gazebo；未接动态 AGX 控制 |
| FKIE→REMANI→新视角感知闭环 | 见 `reports/fkie_remani_closed_loop.md` |

## 下一步

1. 设计单 master 下 Gazebo AGX dynamic + REMANI ESDF/控制桥。  
2. 单目标真运动后再接 FKIE NBV 一次闭环。  
