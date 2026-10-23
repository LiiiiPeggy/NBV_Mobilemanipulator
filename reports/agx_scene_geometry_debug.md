# AGX 场景几何调试（货架重叠 / spawn / 轮胎）

时间：2026-10-23（含 spawn Z 修正）  
配置：`SPAWN_PROFILE=aisle_safe` vs `baseline`

## 根因：XY 进入货架

硬编码 spawn `(-5.36, -5.685, yaw=0)`（baseline）与货架 `Shelves high`（`-4.9,-5.0,yaw=0.727`）及附近货箱在 **OBB/SAT** 下重叠。  
旋转货架的世界 AABB 会过度保守；间隙判定必须用定向包围盒。

## 修复：独立 spawn 配置

| Profile | 文件 | spawn | OBB clear |
|---------|------|-------|-----------|
| `baseline` | `configs/fkie/agx_adapt/config/spawn_warehouse_baseline.yaml` | (-5.36,-5.685,**0.42**,yaw=0) | **否**（保留历史 XY 回归，勿覆盖） |
| `aisle_safe` | `configs/fkie/agx_adapt/config/spawn_warehouse_aisle_safe.yaml` | (-2.10,-5.89,**0.42**,yaw=2.833) | **是**（margin 0.15 m） |

统一入口：`scripts/fkie_agx_warehouse_nbv.sh` 读 `SPAWN_PROFILE` / `AGX_SPAWN_{X,Y,Z,YAW}`。

## 根因：轮胎陷入地面

| 量 | 值 |
|----|-----|
| `fr_wheel_link.STL` 半径 | ≈ **0.153 m** |
| 旧 `base_z=0.36` 时轮心 z | ≈ 0.10 m |
| 旧胎底 | ≈ **−0.05 m**（穿地约 5 cm） |
| 新 `base_z=0.42` 时轮心 z | ≈ 0.16 m |
| 新胎底 | ≈ **+0.007 m**（贴地留隙） |

历史选 z=0.36 是为避免 z=0.05 穿地弹起，仍低估轮半径。实测（aisle_safe 重启后）：`model_xyz=(-2.10,-5.89,0.42)`，四轮 `bottom_z≈0.007…0.009`。

## 原仓库货架上是否有货？

`warehouse_icra_large_viz.world` **上层层板默认无货物**：全图无 `z>1.2` 的货模型。  
仅货架旁地面有 `big_box*`（z≈0.64）。层上/视锥内物体需另 spawn（`primitive_debug` / YCB），不是原 world 自带。

## 证据

- 俯视图：`reports/agx_geometry_shots/spawn_clearance_topdown.png`
- OBB 检查：`data/metrics/agx_debug/clearance_*.json`
- 工具：`scripts/check_agx_spawn_clearance.py`
- 启动日志：`profile=aisle_safe spawn=(-2.1,-5.89,0.42,yaw=2.833)`

## Mode A 关节烘焙

静态模型忽略 `-J` / `set_model_configuration`。`generate_gazebo_urdf.py --arm-pose` 将 CR10 关节转为 **fixed 并烘焙角度**，使 Gazebo 与 RSP TF 一致（gripper tip 误差 ~3e-6 m）。
