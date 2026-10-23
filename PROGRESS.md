# PROGRESS.md — 当前状态

更新时间：2026-10-23  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX（Ranger+CR10）+ FKIE/SSMI 仿真主动探索；后续 REMANI 运动后端与多对象评测。

**当前阶段**：几何/TF/点云基础链路已按证据修复。P4-B / Grounded-SAM / REMANI 执行仍未做。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| FKIE / SSMI Docker 与有效基线 | 已验证（最小 world） | 最小 world 回归 `goals=2`（历史） |
| **AGX Mode A 直立 + 关节烘焙** | **已验证** | bake fixed joints；gripper TF↔Gazebo ~1e-6 m |
| **货架外安全 spawn（aisle_safe）** | **已验证** | OBB clear；俯视图 `reports/agx_geometry_shots/spawn_clearance_topdown.png` |
| **轮胎贴地 spawn Z** | **已验证** | `z=0.42`；胎底≈+0.007 m（旧 0.36 穿地约 5 cm）；见 `reports/agx_scene_geometry_debug.md` |
| **D455 Gazebo 朝向 = optical TF** | **已验证** | 探针绿球中心≈(320,240)，深度≈1.25 m；见 `reports/d455_pointcloud_tf_debug.md` |
| **GT 深度+TF 定位** | **已验证** | median_err≈0.069 m，9/10≤0.10 m |
| **场景 primitive / YCB 分离** | **已落地** | `primitive_debug/` 与 `ycb/` 目录；min_loop `SCENE_KIND` |
| **P4-A NBV→EE** | dry-run | 历史证据保留 |
| P4-B / Grounded-SAM | 未做 | — |

## FKIE 仓库 NBV（重要边界）

| 配置 | 结果 | 说明 |
|------|------|------|
| 历史 a728102 + 旧传感器 pose | `goals=1` | **几何无效**（相机相对 TF 偏 90°） |
| 相机 pose 修复后 `baseline` | Action OK，`goals=0`，`complete_exploration=True` | 见 `logs/.../console_baseline.log` |
| 相机 pose 修复后 `aisle_safe` | 同上 `goals=0` | 见 `console.log`；修前曾 goals=1 亦不可信 |

**不**为非空 goals 放宽碰撞/伪造位姿。下一步需在**正确相机几何**下重调/重测 planner ROI 与观测。

## 部分完成 / 边界

| 项 | 说明 |
|----|------|
| Mode A | 静态；臂连杆合并；仅固定构型感知；默认 `--world-tf-mode tf`（calib 弃用） |
| 颜色检测评测 | TP=4 / recall=0.4 @0.15 m；≠真实识别；正式阈值未冻结 |
| RGB-D 同步 | dt≈0.1 s WARN；建议 message_filters |

## 下一步

1. 在正确 D455 朝向下重做仓库 FKIE 非空 NBV（baseline + aisle_safe）。  
2. YCB 桌面场景 spawn + TF 定位评测（勿与 primitive_debug 混用）。  
3. 独立感知容器 Grounded-SAM / YOLO。  
4. Mode B + REMANI 单目标（非 dry-run）。  
