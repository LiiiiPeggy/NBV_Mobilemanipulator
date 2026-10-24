# PROGRESS.md — 当前状态

更新时间：2026-10-24  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX + FKIE/SSMI；YCB 多对象基准；REMANI Mode B 闭环。

**当前阶段**：Warehouse Mode B **B1–B4 已通过**（动态 AGX + REMANI 真执行 + 相机位移）。S5-A 转换工具已就绪；S5-B 完整「FKIE Action→执行→感知更新」一键尚未跑完。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| Mode B 动态稳定 + 关节可控 | 已验证 | `reports/agx_mode_b_physics.md`, `stability_b1.json` |
| REMANI←Gazebo 状态对齐 | 已验证 | `reports/remani_agx_state_alignment.md` |
| Warehouse 点云地图（部分） | 部分 | `reports/remani_warehouse_map.md` |
| 单 EE 目标规划+执行+相机动 | 已验证 | `reports/remani_single_goal_execution.md` → `SINGLE_EE_EXEC_OK` |
| S5-A FKIE JSON→EE（含 align） | 工具已验证 | `nbv_to_remani_ee_goal.py --from-fkie-json --align-remani-ee` |

## 阻塞 / 未完成

| 项 | 说明 |
|----|------|
| B3 机体点云滤除 | 深度灌入易 BACKEND COLLISION；B4 暂用 `static_empty` |
| S5-B 端到端 FKIE Action 闭环 | 脚本 `run_fkie_remani_one_step.sh` 已备，待在稳定 Mode B 上跑通并记感知差分 |
| REMANI FK vs Link6 Z≈0.42 m | 已用 `--align-remani-ee` 适配；长期应修正 mount 高度参数 |

## 下一步

1. 过滤 `/map_generator/global_cloud` 中机器人邻域，用 `environment_mode=simulated` 重做碰撞验收。  
2. 跑 `run_fkie_remani_one_step.sh` + 运动前后 GSAM/memory 差分。  
3. Mode A 回归（`AGX_MODE=static` aisle_safe NBV）抽检。  
