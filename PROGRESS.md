# PROGRESS.md — 当前状态

更新时间：2026-10-23  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX + FKIE/SSMI 主动探索；独立 Grounded-SAM；YCB 多对象基准；REMANI Mode B 闭环。

**当前阶段**：Stage 0–3（摆放 / GSAM 失败分析 / 多视角记忆 / 评测）已有可复现证据。Stage 4–5（REMANI 真运动）进行中。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| 桌出货架 OBB + 接触高度 Spawn | 已验证 | `reports/ycb_scene_placement_validation.md`；seed42/7/99 validate+spawn+verify OK |
| GSAM 逐对象失败分析 | 已验证 | `reports/grounded_sam_object_failure_analysis.md`；TP=4@0.15（单帧） |
| 多视角记忆去重 | 已验证 | `reports/multiview_object_memory.md`；naive FP25→memory FP6，TP=5 |
| 对象发现基准一键评测 | 已验证 | `reports/object_discovery_benchmark.md` |
| aisle_safe NBV / Grounded-SAM vendor | 已验证 | 既有报告 |

## 部分完成 / 阻塞

| 项 | 说明 |
|----|------|
| REMANI Mode B | wbc 未编译；`/ee_goal` 无订阅者；待 Stage 4 |
| FKIE→REMANI 真运动闭环 | 依赖 Stage 4 |

## 下一步

1. 编译 `third_party/wbc/remani_planner`，验证 `/ee_goal` committed。  
2. Mode B dynamic URDF 单目标执行；再接 FKIE 一次闭环。  
