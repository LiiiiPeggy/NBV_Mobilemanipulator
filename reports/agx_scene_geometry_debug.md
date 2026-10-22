# AGX 场景几何调试（货架重叠 / spawn）

时间：2026-10-23  
配置：`SPAWN_PROFILE=aisle_safe` vs `baseline`

## 根因

硬编码 spawn `(-5.36, -5.685, yaw=0)`（baseline）与货架 `Shelves high`（`-4.9,-5.0,yaw=0.727`）及附近货箱在 **OBB/SAT** 下重叠（`n_overlaps=8`）。  
旋转货架的世界 AABB 会过度保守；间隙判定必须用定向包围盒。

## 修复

| Profile | 文件 | spawn | OBB clear |
|---------|------|-------|-----------|
| `baseline` | `configs/fkie/agx_adapt/config/spawn_warehouse_baseline.yaml` | (-5.36,-5.685,0.36,yaw=0) | **否**（保留 goals=1 回归，勿覆盖） |
| `aisle_safe` | `configs/fkie/agx_adapt/config/spawn_warehouse_aisle_safe.yaml` | (-2.10,-5.89,0.36,yaw=2.833) | **是**（margin 0.15 m） |

统一入口：`scripts/fkie_agx_warehouse_nbv.sh` 读 `SPAWN_PROFILE` / `AGX_SPAWN_{X,Y,Z,YAW}`，不再多处硬编码。

## 证据

- 俯视图：`reports/agx_geometry_shots/spawn_clearance_topdown.png`
- OBB 检查：`data/metrics/agx_debug/clearance_baseline.json`、`clearance_aisle_safe.json`
- 工具：`scripts/check_agx_spawn_clearance.py`（`--live` 可选）
- 实测 Gazebo model：aisle_safe 时 `xyz=(-2.100,-5.890,0.360)` 与 TF `world→base_link` 一致

## Mode A 关节烘焙

静态模型忽略 `-J` / `set_model_configuration`。`generate_gazebo_urdf.py --arm-pose` 将 CR10 关节转为 **fixed 并烘焙角度**，使 Gazebo 合并连杆后的 EE/gripper 与 RSP TF 一致（gripper tip 误差 ~3e-6 m）。
