# F4：连续多步 NBV 调度

Date: 2026-10-25

## 入口

`scripts/run_fkie_remani_exploration.sh` → `run_fkie_online_gsam_step.py`

状态机：INITIAL_OBS → FKIE → REMANI → CAPTURE → GSAM → MEMORY → EVAL → NEXT

每步独立目录：`data/metrics/online_exploration/<run_id>/steps/NNN/`  
共享：`object_memory.json`（同 run 不重置）

## 安全

- 每次一个 REMANI 目标；前次结束后再发  
- `goals=[]` / 不可达：合法终止（exit 10/11），**不无限重试**  
- 冷启动假 complete：最多 **1 次** settle-retry（12s）  
- 禁止手工 EE、禁止复用旧 RGB-D  

## 运行结果

### `online_20261025_141932_seed42`（MAX_STEPS=3）

| 步 | 结果 |
|----|------|
| 000 | 同步 GSAM ok，n_det=2，memory 初始化 |
| 001 try1 | `A_COMPLETE` → settle retry |
| 001 try2 | `G_BASE_POSE` usable，`camera_z≈1.49` |
| 001 exec | traj_received=true，但 `ee_err≈0.66` → `remani_exec_fail` 合法终止 |

### 更早成功的真实运动

`fkie_feasible_base`：同接口下 `SINGLE_EE_EXEC_OK`（ee_err≈1.4 cm）。说明调度与转换正确；本 run 该目标在碰撞/朝向上未跟住。

## 未决

- FKIE 常返回 `request_base_pose` 而非 `goals[]`（RRT best_branch 被增益过滤）  
- yaw-only 姿态对桌面物体召回不稳定  
- warehouse feeder `ready` 在稀疏体素下可能长期 <500（已放宽 explore/restart 等待）  
