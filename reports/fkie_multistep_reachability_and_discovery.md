# FKIE 多步可达性简化与在线对象发现

更新：2026-10-26  
运行：`online_20261025_173649_seed42_reach`  
分支：`cursor/fkie-ssmi-effective-validation`  
场景：Mode B aisle_safe + YCB seed42（在线算法不读 GT；GT 仅事后评测）

## 1. 旧高度过滤修改

| 项 | 旧行为 | 本轮 |
|----|--------|------|
| `save_fkie_action_result.py` | 相机 z∉[0.85,1.75] → 拒绝 `usable_nbv`（`D_FILTERED`） | 仅结构合法性（有限 xyz、非退化四元数）；**无固定高度硬拒绝** |
| `nbv_to_remani_ee_goal.py` | 易把 base 当 EE | `action_type=base_pose` 直接拒绝转 EE |
| aisle_safe yaml | `arm_height_min` 曾抬到 0.95 作采样带 | RRT 采样带 `0.35–1.65`（非 EE 拒绝门） |
| 可达性 | 高度门 + 臆测 | 基础合法性 → REMANI / 底盘执行验证 |

证据：step002 `camera_goal_xyz` z≈1.53（及候选 z≈1.34–1.53）均进入 REMANI 多候选尝试，未因高度被丢弃。

## 2. FKIE 底盘 vs 相机目标语义

源码（`NBVPlanner.cpp`）：

- `goals[]` 非空 → 臂/相机路径（本仓 `action_type=camera_nbv`）
- `goals[]` 空且有 frontier → `request_base_pose=true`，`goal_pose_3d=best_frontier`（`action_type=base_pose`）
- 无 frontier / 无 best frontier → `complete_exploration=true`

适配：

| 类型 | 执行器 | 说明 |
|------|--------|------|
| `base_pose` | `execute_base_pose_goal.py`（SetModelState lerp + `car_cmd`） | 只用 XY+yaw；**不做**高度过滤；**不**转 EE |
| `camera_nbv` | `nbv_to_remani_ee_goal.py` → REMANI；最多 3 个 `goals[]` 候选 | 失败记录原因后试下一候选 |

## 3. 连续执行加固

| 问题 | 处理 |
|------|------|
| REMANI 旧 `car_cmd` 把底盘拖出 ROI → 假 `A_COMPLETE` | `ensure_agx_aisle_hold.py`：跑前强制贴回 aisle；仅当相机出多边形才恢复（不撤销合法 base 移动） |
| 重启 planner 后 `cached_nodes` 空 → 立刻 complete | 探索脚本 FKIE **warm-up** Action；yaml `sample_in_unknown: true`，`utility_weight_visited_cell: 40` |
| 假 complete / 地图未就绪 | 每步最多 3 次 settle-retry（A_COMPLETE / 无 frontier / TF/map） |
| base 到位后 hold 被抢 | 执行后短时 re-latch `car_cmd`；到位后 settle 4 s 再采图 |

## 4. seed42 多步结果（MAX_STEPS=3）

命令：

```bash
docker exec nbv_fkie bash -lc '
  export SEED=42 MAX_STEPS=3 ENSURE_YCB=1 ENSURE_MAP=1 \
         RESET_OCTOMAP=1 RESTART_FKIE=1 \
         RUN_ID=online_20261025_173649_seed42_reach
  bash /workspace/scripts/run_fkie_remani_exploration.sh
'
```

产物：`data/metrics/online_exploration/online_20261025_173649_seed42_reach/`

| 步 | 动作 | 执行 | 备注 |
|----|------|------|------|
| 0 | 初始观测 | — | GSAM n_det=10；confirmed TP=2 FP=0 |
| 1 | **base_pose** | OK | 路径 ≈1.75 m，xy_err≈0.10 m |
| 2 | **camera_nbv** | OK（候选 2） | 候选 0/1 REMANI 失败（ee_err 0.40/0.82 m）；候选 2 ee_err≈5 mm |
| 3 | **base_pose** | OK | 路径 ≈2.11 m，xy_err≈0.12 m |

终止：跑满 3 次 NBV，`terminated=false`（非假成功凑数）。

### 累计对象指标（GT 事后评测）

| 阶段 | 记忆 TP/FP | unique Recall | confirmed TP/FP | confirmed Recall |
|------|------------|---------------|-----------------|------------------|
| step0 | 3/4 | 0.273 | 2/0 | 0.182 |
| step1 | 4/4 | 0.364 | 3/2 | 0.273 |
| step2 | 4/4 | 0.364 | 3/2 | 0.273 |
| step3 | 4/5 | 0.364 | 3/2 | 0.273 |

最终：`n_confirmed=5`，`n_memory_objects=9`。  
运动成本：底盘累计 XY ≈ **3.87 m**，底盘执行时间 ≈ **4.4 s**（不含规划/感知墙钟）。

### 曲线

- 累计 Recall / 正确对象数 vs NBV 步：`reports/fkie_multistep_reachability_shots/curves.png`
- Recall vs 墙钟：`reports/fkie_multistep_reachability_shots/curve_recall_vs_time.png`

### 截图

- `rgb_step000.png` / `gsam_step000.png` — 初始观测  
- `rgb_step001_after_base.png` / `gsam_step001.png` — 底盘 NBV 后  
- `rgb_step002_after_camera.png` — 相机 NBV 后  

## 5. 失败与终止说明

- 本轮 **未** 因空 NBV 提前终止。  
- step2 两候选 REMANI 真实失败后改试下一 FKIE 目标（未放宽碰撞、未瞬移、未发明 EE）。  
- 历史失败（本轮前）：底盘被旧 `car_cmd` 拖到 ≈(8.4,7.5) 出多边形 → `A_COMPLETE`；已用 aisle hold + warm-up 消除。  
- seed7 / seed99：未跑（seed42 先稳定）。

## 6. 验收对照

| 标准 | 结果 |
|------|------|
| 取消 0.85–1.75 硬过滤 | 通过 |
| `G_BASE_POSE` / 相机目标分流 | 通过（step1/3 base；step2 camera） |
| 运动后状态再规划 + 失败处理 | 通过（3 次真实执行；多候选回退） |
| 在线多步 + Recall 曲线 | 通过（本报告 + curves） |
