# PROGRESS.md — 当前状态

更新时间：2026-10-22  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX（Ranger+CR10）+ FKIE/SSMI 仿真主动探索；后续 REMANI 运动后端与多对象评测。

**当前阶段**：P0/P1 已验证。P2/P3/P5 **最小闭环有运行证据**（召回/精确率可量化）；P4-A 完成相机→EE 变换 dry-run；P4-B 未做。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| FKIE / SSMI Docker 与有效基线 | 已验证 | 最小 world 回归 `goals=2`；本轮仓库 NBV 仍 `goals=1` |
| **AGX Mode A 直立稳定** | **已验证** | roll=pitch=0，z=0.36 |
| **AGX 仓库非空 NBV** | **已验证** | `FKIE_AGX_WAREHOUSE_NBV_OK` goals=1 |
| **P2 多对象场景+真值** | **已验证（最小）** | seed42：10 对象 spawn + `EXPORT_OBJECT_GT_OK`；截图 `reports/indoor_objects_shots/` |
| **P3 RGB-D 检测定位** | **部分验证** | color backend + calib T；`PERCEIVE_RGBD_OK` |
| **P5 最小评测器** | **已验证（预实验）** | tp=2 / recall=0.2 / precision=0.125 / mean_err≈0.178 m（thresh=0.35 m） |
| **P4-A NBV→EE 变换** | **dry-run 验证** | `NBV_TO_REMANI_EE_GOAL_OK`；未调用 REMANI 规划节点 |

## 部分完成 / 边界说明

| 项 | 说明 |
|----|------|
| Mode A | **静态**模型：Gazebo 关节冻结；`/joint_states` TF ≠ Gazebo D455 位姿 |
| P3 定位 | 正式 TF 链路未对齐；当前用经验标定 `T_world_cam_calib.json` |
| Grounded-SAM-2 | 未装入 FKIE 容器（无 torch）；预留独立感知容器接口 |
| P4-B | 需 Mode B / REMANI 执行；禁止用改模型位姿冒充运动 |

## 未完成

- Mode B 物理控制与 REMANI 单目标规划成功（非 dry-run）  
- Grounding DINO + SAM2 感知前端  
- 正式 baseline 对比与冻结 0.1 m 阈值实验  

## 下一步

1. 固化 Mode A 感知：Gazebo 相机外参标定流程脚本化，或 Mode B 关节一致化。  
2. 独立感知容器装 Grounded-SAM-2 / YOLO，接口保持 ROS RGB-D → detections JSON。  
3. 在 WBC REMANI 栈上做 `/ee_goal` 单目标规划（仍可不执行）。  
