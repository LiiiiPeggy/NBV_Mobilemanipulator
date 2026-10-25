# AGX Mode B 全身抖动诊断与修复（P0）

更新：2026-10-25  
证据：`reports/agx_mode_b_jitter_before_after.csv`，`data/metrics/fkie_agx_warehouse_mode_b/jitter_*.json`

## 预声明静止阈值（不事后放宽）

| 指标 | 阈值 |
|------|------|
| base XY std | ≤ 0.002 m |
| base Z std | ≤ 0.003 m |
| yaw std | ≤ 0.5 deg |
| CR10 关节 std | ≤ 0.005 rad |
| D455 XY std | ≤ 0.005 m |
| `/joint_states` Hz | ≤ 200 |

## 根因（有实测）

1. **`/joint_states` 自循环**  
   `agx_mode_b_joint_bridge.py` 订阅并重新发布 `/joint_states`，与 Gazebo 插件叠加。  
   before：`js_hz≈7457`，`dup_ratio≈0.96`。  
   修复后：仅 Gazebo 发布，`js_hz≈23–50`，`dup_ratio=0`。

2. **关节桥对车轮/转向无意义强制写入**  
   与 p3d / car 位姿写冲突。改为 **ARM-only** `set_model_configuration` @ 50 Hz。

3. **底盘运动学桥 vs ODE 接触**  
   - 持续 50 Hz lerp：可见高频抖动。  
   - soft-hold @ 2 Hz snap：锯齿抬高 XY std（v2）。  
   - **零写入**：Ranger 轮未制动，自由滚动（v4：`base_x_std≈0.15 m`）。  
   **结论**：Mode B 运动学植物在 idle 必须权威保持位姿；settled 后对固定目标做 **exact snap + twist=0**（spawn latch + 到达后每周期精确写）。

## 实验对照

| tag | js_hz | base_x_std | yaw_std_deg | arm_j2_std | ok_idle |
|-----|-------|------------|-------------|------------|---------|
| before_idle | 7457 | 0.00158 | 0.079 | 0.00220 | False（JS） |
| after_idle_v2 soft-hold | 23.2 | 0.00407 | 0.304 | 0.00135 | False |
| after_idle_v4 zero-hold | 24.7 | **0.149** | 0.783 | 0.00534 | False |
| **after_idle_v5** exact-hold | 23.5 | **0.00010** | **0.008** | **0.00022** | **True** |
| expB_joint_only | 27.2 | 0.090 | 0.685 | 0.00016 | False（滚动） |
| expB_car_only | 27.0 | 0.00032 | 0.055 | **0.123** | False（臂掉） |
| expB_both | 24.8 | 0.00020 | 0.023 | 0.00053 | **True** |

实验 B：底盘保持与机械臂保持缺一不可。  
实验 D：Gazebo `model_states` 与 P3D/TF/相机同量级收敛 → **真实仿真状态稳定**，非仅 RViz。

## 架构限制（明确记录）

当前 Mode B 仍是 **运动学桥**（`set_model_state` / `set_model_configuration`），不是轮式动力学+关节力矩控制器。  
idle 稳定依赖单一权威外部位姿/关节写入；若改真实动力学，需 ROS Control / 轮制动方案，不能靠关闭重力或 static 模型掩盖。

## 代码变更

- `scripts/agx_mode_b_joint_bridge.py`：禁止 JS 重发布；ARM-only hold  
- `scripts/agx_mode_b_car_cmd_bridge.py`：spawn latch + settled exact snap + twist 清零  
- `scripts/fkie_stop_sim.sh`：忽略僵尸 gz；不误杀 Mode B launcher  
- `scripts/diagnose_agx_mode_b_jitter.py`：定量门禁  
- `scripts/restart_mode_b_bridges.sh`：`HOLD_HZ` 默认 50
