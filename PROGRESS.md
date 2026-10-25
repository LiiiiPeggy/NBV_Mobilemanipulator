# PROGRESS.md — 当前状态

更新时间：2026-10-25  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX + FKIE/SSMI；YCB 多对象基准；REMANI Mode B 闭环。

**当前阶段**：E0–E3 主路径已打通（累计 OctoMap + simulated 非空 REMANI 轨迹）。E4/E5（在线 GSAM 多步）待接。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| Mode B B1–B4 / P0 抖动 | 已验证 | `reports/agx_mode_b_jitter_diagnosis.md`；`/joint_states` 仅 Gazebo 发布 |
| Mode B + seed42 YCB | 已验证 | 本轮 world：`mo_obj_00..10` + table + occluder |
| **E1 累计 OctoMap** | **已验证** | `reports/octomap_fusion_diagnosis.md`：yaw+25° 后 free+1944 / occ+203 / full_bytes+12765 |
| **E2 simulated 地图+规划** | **已验证** | `reports/remani_simulated_collision_diagnosis.md`；`SINGLE_EE_EXEC_OK` |
| **E3 轨迹可视化填充** | **已验证** | planned Path 非空；`ee_err≈0.003 m` |
| P2 GSAM 到达门禁 | 已验证 | `reports/gsam_nbv_arrival_trigger.md` |

## 阻塞 / 未完成

| 项 | 说明 |
|----|------|
| FKIE Action `goals=[]` | 本轮 one-step 调用返回空目标；与 REMANI 地图修复独立，需 ROI/地图边界复查 |
| E4 在线 GSAM+对象记忆 | 门禁已通；未完成 NBV 到达后记忆差分 |
| E5 连续 3–5 次 NBV | 依赖 E4 + 非空 FKIE goals |
| 完整轮式动力学 | Mode B 仍为运动学桥；`max_time_scale=64` 仅服务运动学执行 |

## 下一步

1. 恢复 FKIE 非空 NBV（检查 octomap/ROI/Action 边界）。  
2. `FKIE→REMANI→到达→GSAM→object memory` 单步差分（E4）。  
3. seed42 连续 3 次自主 NBV（E5）。  
4. Mode A / idle jitter 抽检回归。
