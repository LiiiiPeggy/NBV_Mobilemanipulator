# PROGRESS.md — 当前状态

更新时间：2026-10-23  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX + FKIE/SSMI 主动探索；独立 Grounded-SAM；真实 YCB 多对象场景可视化与评测；后续 REMANI。

**当前阶段**：YCB 对象已在 Gazebo **真实 Spawn 且 D455 可见**；一键入口 `start_fkie_agx_ycb_demo.sh` 可用。REMANI 执行仍待接。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| AGX Mode A / aisle_safe / D455 optical | 已验证 | 既有 geometry 报告 |
| P1-R aisle_safe NBV `goals=1` | 已验证 | `reports/fkie_corrected_optical_nbv.md` |
| Grounded-SAM NBV 独立 vendor | 已验证 | `reports/grounded_sam_integration.md` |
| **YCB 真实 Spawn + GUI/D455 可见** | **已验证** | `SPAWN 13/13`；`d455_rgb_seed42.png` 可见桌面糖盒/罐头/碗/香蕉/苹果等；`reports/ycb_gazebo_visibility_validation.md` |
| YCB@0.15 GSAM 基础发现率 | 初步 | tp=2 recall≈0.18 precision≈0.29 mean_err≈0.023 m |
| `/ee_goal` 桥接 | 半验证 | 已 publish；无 REMANI 订阅 |

## 部分完成 / 阻塞

| 项 | 说明 |
|----|------|
| 货架层物体高度 | 部分沿板高度近似，可能略浮空；需按货架板实测再精调 |
| 实时 `grounded_sam` ROS 节点 | 当前为 offline GSAM + ROS 深度定位；nbv_perception 无 rospy |
| Mode B / REMANI 执行 | 未验收 |

## 下一步

1. 精调货架 z；提高 GSAM 类别词表/阈值后重评。  
2. 为 nbv_perception 加 rospy 或 JSON 桥，跑 live `grounded_sam`。  
3. REMANI 订阅 `/ee_goal` + Mode B。  
