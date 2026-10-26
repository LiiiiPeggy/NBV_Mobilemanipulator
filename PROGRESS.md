# PROGRESS.md — 当前状态

更新时间：2026-10-27  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：AGX + FKIE/SSMI；YCB 多对象基准；REMANI Mode B 闭环。

**当前阶段**：空闲轨迹线团已修；静止门禁回归通过；底盘直线路径粗筛上线；seed42 五步尝试合法终止于 REMANI 失败。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| 可达性简化 + G_BASE 分流 + 3 步发现 | 已验证 | `reports/fkie_multistep_reachability_and_discovery.md` |
| **空闲无 EE_Actual 线团** | **已验证** | `IDLE_PATH_OK`；`reports/agx_idle_trajectory_and_fkie_baseline.md` |
| **Mode B 静止门禁 v6** | **已验证** | `jitter_idle_v6.json` → `ok_idle=true` |
| **底盘路径 footprint 粗筛** | **已验证** | unsafe→`BASE_PATH_UNSAFE` rc=12；safe→EXEC_OK |
| seed42 MAX_STEPS=5 尝试 | 部分 | 1×camera OK 后 step2 `remani_all_candidates_fail` |

## 阻塞 / 未完成

| 项 | 说明 |
|----|------|
| 连续凑满 5 次到位 | REMANI 对部分 FKIE 相机目标失败（fail-closed） |
| seed7 / seed99 | 未跑 |
| 完整 Nav2 / 轮式动力学 | 明确不做；仅直线云图粗筛 |

## 下一步

1. 分析 step2 REMANI 无 traj / 大误差原因（地图一致性 / 目标朝向）。  
2. 稳定后扩展 seed7、seed99。  
3. Mode A 抽检回归。
