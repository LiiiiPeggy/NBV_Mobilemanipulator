# REMANI aisle `IK failed: collision` / BACKEND COLLISION 诊断

Date: 2026-10-25

## 根因分层

| 阶段 | 现象 | 根因 | 处置 |
|------|------|------|------|
| A | `IK failed: collision` | 累积图含自身回波；`checkcollision` 拒绝对终端 | accum purge + 底盘代理对齐 |
| B | IK 提交成功后 `max_time_scale` reject | 小目标 RRT 种子轮速极大，scale≈46 > 2 | Mode B `max_time_scale:=64` |
| C | `BACKEND COLLISION type=car` @ ~(-2.4,-6.8) | 轨迹擦桌膨胀；safe_dist≈0.25，净空≈0.26 | `obstacles_inflation:=0.05`，`safe_margin:=0.05`（碰撞仍开启） |

未关闭环境碰撞；`static_empty` 仅作运动链隔离（`SINGLE_EE_EXEC_OK`）。

## 验收（simulated）

```
[EE GOAL] committed EE_POSE terminal (Δxy≈0.02)
GEN_NEW_TRAJ → EXEC_TRAJ → WAIT_TARGET
SINGLE_EE_EXEC_OK  ee_err≈0.003 m  traj_received=true
/nbv/trajectory/ee_planned 非空
```

证据：`data/metrics/nbv_remani/e2_simulated_margin05.json`

对照隔离：`e2_static_empty_iso.json`（不得当作仓库避障成功证据）。

## 仍注意

- 前端路径仍可能绕向桌子一侧；当前靠略松的 margin/inflate 通过擦边。  
- FKIE Action 本轮曾 `goals=[]`（地图/ROI）；与 REMANI 地图修复独立。  
- REMANI EE z 与 TF Link6 仍有固定高度差，轨迹误差应用同一契约（`/ee_current_pose`）。
