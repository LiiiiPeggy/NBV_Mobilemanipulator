# PROGRESS.md — 当前状态

更新时间：2026-10-23  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX（Ranger+CR10）+ FKIE/SSMI 仿真主动探索；独立 Grounded-SAM 感知；后续 REMANI 运动后端与多对象评测。

**当前阶段**：正确光学几何下 aisle_safe FKIE 非空 goals 已恢复；GDINO+SAM2 已 vendored 并在 NBV 自有环境独立推理；`/ee_goal` 桥接已发布，REMANI 执行仍待接。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| FKIE / SSMI Docker 与有效基线 | 已验证（最小 world） | 最小 world 回归 `goals=2`（历史） |
| **AGX Mode A 直立 + 关节烘焙** | **已验证** | bake fixed joints；gripper TF↔Gazebo ~1e-6 m |
| **货架外安全 spawn（aisle_safe）** | **已验证** | OBB clear；`spawn_clearance_topdown.png` |
| **轮胎贴地 spawn Z** | **已验证** | `z=0.42`；见 `reports/agx_scene_geometry_debug.md` |
| **D455 Gazebo 朝向 = optical TF** | **已验证** | `reports/d455_pointcloud_tf_debug.md` |
| **P1-R 正确光学下 aisle_safe NBV** | **已验证** | `goals=1`，GOAL≈(-3.020,-5.294,1.609)；`reports/fkie_corrected_optical_nbv.md` |
| **YCB 布局与 primitive 分离** | **已落地** | `ycb/layouts/seed42.json`（桌+架+地）；consistency OK |
| **P3 Grounded-SAM 独立 vendoring** | **已验证** | `perception/vendor/*` + `checkpoints/`；`PERCEPTION_INDEPENDENCE_OK`；GPU smoke LOAD≈3s / INFER≈0.76s / ~2.7GB |
| **P4 `/ee_goal` 桥接发布** | **半验证** | 已 publish；无 REMANI 订阅者；`reports/mode_b_remani_ee_goal.md` |

## FKIE 仓库 NBV（边界）

| 配置 | 结果 | 说明 |
|------|------|------|
| 错误 90° 传感器 pose（历史） | `goals=1` | **几何无效**，禁止回退 |
| 修光轴后未调 ROI（旧） | `goals=0` | 已由 aisle_safe 臂姿+ROI 东界重测取代 |
| **修光轴 + aisle_safe 臂/ROI** | **`goals=1`** | `camera_pose`=`d455_depth_optical_frame` |

## 部分完成 / 阻塞

| 项 | 说明 |
|----|------|
| 正式 YCB+GSAM 发现率评测 | 布局/工具就绪；待 spawn YCB + 多视角 `grounded_sam` 抓数 |
| Mode B 物理 bringup | 脚本就绪；本轮未切 dynamic（保留 Mode A FKIE） |
| REMANI 执行闭环 | wbc 在仓；`/ee_goal` 无执行节点订阅 |
| GDINO CUDA `_C` 扩展 | 未编译；GPU 上走 PyTorch MSDA fallback（可用） |

## 下一步

1. aisle_safe 下 spawn YCB → `grounded_sam` 多视角 → eval@0.15 (+0.10/0.35)。  
2. 启动 REMANI 接收 `/ee_goal`，Mode B 单目标执行。  
3. （可选）编译 GroundingDINO CUDA ops 以加速。  
