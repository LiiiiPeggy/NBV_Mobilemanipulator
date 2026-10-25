# 在线对象发现基准（初步）

Date: 2026-10-25  
标签：**online_autonomous_nbv**（非离线多视角烘焙）

## 对比原则

| 类型 | 来源 | 可否并入正式曲线 |
|------|------|------------------|
| 离线多视角 Mode A | 烘焙相机位姿 | 否（仅对照） |
| 在线 FKIE→REMANI→GSAM | 本报告 | 是 |

## 在线样本

### A. 初始观测（`online_20261025_135247_seed42` step 000）

- frame unique_recall ≈ 0.27（TP=3/11）  
- confirmed TP=2 FP=0  

### B. 真实 NBV 到达（`fkie_feasible_base`）

- 到位误差：EE 1.4 cm，相机 1.1 cm  
- 到达后 GSAM：视点相关，可能 TP=0（不粉饰）

### C. 多步调度（`online_20261025_141932_seed42`）

- 步 0 观测完成；步 1 获 usable NBV 后 REMANI 未到位 → 合法终止  
- 曲线：`data/metrics/online_exploration/.../curves.png`（若已生成）

## 模块耗时（量级）

| 模块 | 典型 |
|------|------|
| FKIE Action | 10–25 s |
| REMANI 执行 | 30–120 s |
| GSAM snapshot | ~5 s（权重已加载后） |

## 结论

对象级在线链路已接通并有真实指标；连续 3–5 次全部到位的召回曲线仍受 FKIE 朝向与 REMANI 可达性限制，需后续视点质量改进，**不能**用假目标凑步数。
