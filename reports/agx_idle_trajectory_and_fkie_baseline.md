# AGX 空闲轨迹 / 微抖回归 + FKIE 五步 baseline

更新：2026-10-27  
分支：`cursor/fkie-ssmi-effective-validation`  
基准：`2fad1ff` → 本轮提交

## 1. 空闲青色 `EE_Actual` 线团

### 根因
`visualize_remani_trajectory.py` 在 **20 Hz TF 定时器**上持续追加 EE 点；5 mm 下采样挡不住空闲微振的**长时间累积**，RViz 青线（颜色 `0;220;220`）叠成线团。与探索程序无关。

### 修改
仅在真实运动窗口累计：

| 触发开始记录 | 停止 |
|--------------|------|
| `/planning/start`、`/planning/trajectory`、`/ee_goal` | `/planning/finish` + settle 1.5 s |
| `car_cmd` 目标跳变（底盘 lerp） | 15 s 超时 settle |

- 新段开始清空旧 Path（latched 空 Path，RViz 立即清掉历史线团）
- 运动中仍保留 `EE_Planned`（黄）vs `EE_Actual`（青）对比
- 下采样改为运动窗口内 2 cm

### 验收
空闲约 10 s 采样：`ee_actual_max_poses_during_idle=0` → **`IDLE_PATH_OK`**  
截图：`reports/agx_idle_trajectory_shots/desktop_after_idle.png`

## 2. Mode B 静止门禁回归

脚本：`diagnose_agx_mode_b_jitter.py --duration 20`  
产物：`data/metrics/fkie_agx_warehouse_mode_b/jitter_idle_v6.json`

| 门禁 | 结果 | 实测 std（量级） |
|------|------|------------------|
| base XY | OK | ≈1e-4 m |
| base Z | OK | ≈7e-5 m |
| yaw | OK | ≪0.5° |
| 臂关节 | OK | ≤2.3e-4 rad |
| D455 XY | OK | ≤7e-5 m |
| js_hz | OK | ≈24 Hz，dup=0 |

**`ok_idle=true` / `JITTER_IDLE_OK`** — 与 `after_idle_v5` 同级。  
**未改** hold 桥 / 重力 / 接触；可见微振视为 Mode B 运动学植物可接受残差，视觉线团已由任务 1 消除。

## 3. 底盘路径安全检查

`execute_base_pose_goal.py` 在 lerp 前：

1. 读 `/warehouse_map/cloud_filtered`（备选 global_cloud / octomap centers）
2. 直线采样 XY，`footprint_radius=0.70 m`，`z∈[0.18,1.80]`
3. 碰撞 → **`BASE_PATH_UNSAFE`**（退出码 12），不执行  
4. 地图不可用 → fail-closed 拒绝（不声称无碰撞）

**不是 Nav2**；只拒绝明显穿货架/桌子的直线插值。

实测：

| 目标 | 结果 |
|------|------|
| 不安全 `(-4.5,-5.0)`（货架侧） | `footprint_obstacle_collision`，拒绝 |
| 安全 `(-2.4,-5.5)`（通道） | `path_safety.ok=true`，`BASE_POSE_EXEC_OK` |

在线步脚本：不安全记 `terminate.reason=base_path_unsafe`。

## 4. seed42 最多 5 步在线探索

命令：

```bash
docker exec nbv_fkie bash -lc '
  export SEED=42 MAX_STEPS=5 ENSURE_YCB=1 ENSURE_MAP=1 \
         RESET_OCTOMAP=1 RESTART_FKIE=1 \
         RUN_ID=online_20261026_221739_seed42_s5
  bash /workspace/scripts/run_fkie_remani_exploration.sh
'
```

产物：`data/metrics/online_exploration/online_20261026_221739_seed42_s5/`

| 步 | 动作 | 执行 | 记忆 confirmed TP/FP / unique Recall |
|----|------|------|--------------------------------------|
| 0 | 初始观测 | — | 2/0；memory Recall≈0.36 |
| 1 | **camera_nbv** | OK（ee_err≈2.6 mm） | confirmed 2/0；Recall≈0.36 |
| 2 | camera_nbv（3 候选） | **全部 REMANI 失败** | — |

**终止（真实）**：`remani_all_candidates_fail` @ step 2（未伪造候选凑满 5 步）。  
有效 NBV 执行：**1** 次相机目标。  
曲线：`reports/agx_idle_trajectory_shots/curves.png`、`curve_recall_vs_time.png`  
回归对照：此前 `…_seed42_reach` 已完成 3 步（2×base+1×camera）。

## 5. 未决

- 五步跑中途 REMANI 对部分 FKIE 相机目标跟踪失败（无 traj / 大误差）— 执行层 fail-closed，未关碰撞。  
- seed7/99 未跑。  
- 底盘安全检查为本轮直线 footprint 云图粗筛，非完整运动学规划。

## 6. 验收对照

| 项 | 结果 |
|----|------|
| 空闲无青色线团 | **通过**（IDLE_PATH_OK） |
| 静止门禁 | **通过**（JITTER_IDLE_OK） |
| 底盘不穿已知障碍 | **通过**（unsafe 拒绝 / safe 执行） |
| seed42 ≤5 步真实尝试 + 曲线 | **通过**（合法提前终止于 step2） |
