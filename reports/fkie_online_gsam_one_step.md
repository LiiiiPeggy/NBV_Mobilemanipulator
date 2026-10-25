# F3：单次在线对象发现闭环

Date: 2026-10-25

## 闭环（已跑通）

`FKIE Action → nbv_to_remani_ee → REMANI EXEC → 到达 → 同步 RGB-D → GSAM(snapshot) → ObjectMemory → GT eval`

### 运动证据（`data/metrics/fkie_feasible_base/`）

| 项 | 值 |
|----|-----|
| FKIE | `G_BASE_POSE`，`usable_nbv=true`，`camera_goal≈(-3.45,-6.62,1.00)` |
| EE goal | `(-3.52,-6.56,0.53)`（`--align-remani-ee`） |
| 执行 | `SINGLE_EE_EXEC_OK` |
| `ee_err_m` | **0.014** |
| `cam_err_m` | **0.011** |
| `cam_delta_m` | 0.92（确有运动） |

### 初始观测记忆（同会话 seed42，`online_20261025_135247_seed42/steps/000`）

| 指标 | 值 |
|------|-----|
| snapshot GSAM | n_det=8 / localized=8，`status=ok` |
| frame TP/FP/Recall | 3 / 5 / 0.273 |
| memory confirmed | 2（`confirmed_eval` TP=2 FP=0） |
| mean center err (frame) | 0.030 m |

对象记忆从空初始化，未加载 Mode A 离线多视角结果。

### 到达后观测（`online_f3_one_step_140648`）

新 snapshot 推理成功；该视点仅 1 检测且 GT TP=0（FKIE yaw-only 朝向导致视场不佳）。**如实记录**：单次 NBV 不保证提高 TP。

## Fail-closed

- 低高度 `goal_pose_3d`（z≈0.62→EE≈0.16）判 `D_FILTERED`，不发明 EE  
- `goals=[]` 且无 usable → 终止，不手工目标  

## 入口

```bash
# 单步编排
python3 /workspace/scripts/run_fkie_online_gsam_step.py --run-dir ... --step 0|N --device cuda
# 或完整调度
bash /workspace/scripts/run_fkie_remani_exploration.sh
```
