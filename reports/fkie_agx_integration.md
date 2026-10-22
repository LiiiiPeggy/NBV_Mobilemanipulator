# FKIE + AGX 仓库集成报告

日期：2026-10-22  
分支：`cursor/fkie-ssmi-effective-validation`  
模式：**Mode A 静态感知**（`AGX_MODE=static`）— 非全身运动闭环

## 验收摘要

| 项 | 结果 | 证据 |
|----|------|------|
| P0 整机不倾倒 | **通过** roll=0 pitch=0，z=0.36 | `data/metrics/fkie_agx_warehouse_nbv/model_pose.txt` |
| P0 TF 单一权威 | **通过** world→base 静态 TF；单路 `/joint_states` | diag TF `world->base_link=(-5.36,-5.685,0.36)` |
| P0 CamView 无 TF 遮挡 | **通过**（现已合并为单 RViz：`agx_nbv.rviz` 用 Image 面板，Camera overlay 关闭） | `configs/fkie/agx_adapt/rviz/agx_nbv.rviz` |
| P0 Gazebo 可见 AGX+货箱 | **通过** model `agx_ranger_cr10`；60 boxes | 运行时 inventory；`warehouse_icra_large_viz.world` |
| P1 AGX 仓库非空 NBV | **通过** goals=1 | `FKIE_AGX_WAREHOUSE_NBV_OK`；GOAL `(-4.419,-6.198,1.609)` |
| 最小 world 回归 | **通过** goals=2 | `FKIE_EFFECTIVE_NBV_OK goals=2` |

截图：`reports/fkie_agx_integration_shots/desktop.png`

## 根因与修复（P0）

1. **倾倒**：spawn `z=0.05` 过低，轮心相对 base≈−0.258，陷入地面后弹起；无保持力矩时臂/底盘姿态与 RViz 双路 `/joint_states` 冲突。  
   **修复**：Mode A `<static>true</static>`；spawn `z=0.36`；`-J` + `set_model_configuration`；**唯一** `agx_joint_states_authority`；去掉 gazebo js/p3d 插件与静态 TF 竞争。

2. **CamView 坐标轴**：`agx_nbv.rviz` 的 Camera 与 TF 同配置叠加。  
   **修复**：曾拆成 `agx_debug` / `agx_camera`；现合并为 `agx_nbv.rviz`（3D 保留 TF，相机用 Image 面板避免坐标轴遮挡）。

3. **货箱不可见**：原 world DAE 贴图易失败。  
   **沿用**：`warehouse_icra_large_viz.world`（实体盒材质）。

## 根因与修复（P1 goals=0）

在姿态稳定 + 相机朝向货架 + octomap `max_range=5`、`filter_ground=false`、`gain_r_max=5` 后，Action 返回 **goals=1**，`complete_exploration=False`。  
此前失败主因是**机器人倾倒/TF 漂移 + 过短感知距离 + 观测构型不佳**导致无有效 frontiers，而非单纯 RRT 参数。

## 启动

```bash
bash scripts/setup_x11.sh
docker compose -f docker-compose.fkie.yml up -d
docker exec nbv_fkie bash /workspace/scripts/fkie_stop_sim.sh
docker exec -d nbv_fkie bash -lc \
  'nohup bash /workspace/scripts/fkie_agx_warehouse_nbv.sh >> /workspace/logs/fkie_agx_warehouse_nbv/console.log 2>&1'
```

## 明确未完成

- Mode B 物理关节控制 / REMANI 执行闭环  
- 多对象 YCB 场景、Grounded-SAM 感知、正式评测 baseline  
- 「有效视角目标」≠「已验证全身运动可达」
