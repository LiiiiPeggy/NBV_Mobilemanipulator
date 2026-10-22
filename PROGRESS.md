# PROGRESS.md — 当前状态

更新时间：2026-10-22  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX（Ranger+CR10）+ FKIE/SSMI 仿真主动探索；后续 REMANI 运动后端与多对象评测。

**当前阶段**：P0/P1 **已验证**（AGX Mode A 静态感知 + 仓库非空 NBV + 最小 world 回归）。P2–P5 仅有脚手架/接口草案，未完成正式多对象实验。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| FKIE / SSMI Docker 与有效基线 | 已验证 | 既有 metrics；最小 world 回归 `goals=2` |
| **AGX Mode A 直立稳定** | **已验证** | roll=pitch=0，z=0.36；单路 joint_states + static TF |
| **AGX 仓库非空 NBV** | **已验证** | `FKIE_AGX_WAREHOUSE_NBV_OK` goals=1；GOAL `(-4.419,-6.198,1.609)` |
| CamView 无 TF 遮挡配置 | 已交付 | `agx_camera.rviz` / `agx_debug.rviz` |
| 仓库货箱可视化 world | 已用 | `warehouse_icra_large_viz.world`；60 boxes + AGX |
| 集成报告 | 已写 | `reports/fkie_agx_integration.md` + desktop 截图 |

## 部分完成 / 边界说明

| 项 | 说明 |
|----|------|
| Mode A | **静态**模型：证明感知与 NBV，**不是**全身运动控制 |
| NBV 目标 | 「有效视角目标」≠「REMANI 已执行可达」 |
| P2 对象资产 | 仅 `scripts/fetch_gazebo_ycb_objects.sh` 清单/可选 clone |
| P3–P5 | 接口草案 `reports/nbv_remani_interface.md`；无 Grounded-SAM / 评测闭环 |

## 未完成

- Mode B 物理控制与 REMANI 单目标联通  
- 8–12 对象室内场景 + 真值导出 + 最小评测  
- Grounding DINO + SAM2 感知前端  
- 正式 baseline 对比  

## 下一步

1. P2：拉取 YCB 子集并做 8–12 对象可复现场景。  
2. P4：对照 `wbc` REMANI 实际接口做单目标联通。  
3. P3/P5：冻结感知前端并实现最小对象召回评测。
