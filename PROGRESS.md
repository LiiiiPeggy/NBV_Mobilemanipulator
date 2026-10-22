# PROGRESS.md — 当前状态

更新时间：2026-10-22  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：在 A6000 上复现 FKIE NBV 与 SSMI 基线，建立可复用 Docker 仿真，后续接入运动规划后端（REMANI / TopAY）并服务移动操作主动视角实验。

**当前阶段**：Docker 仿真 + **有效**地图/规划验证已完成（FKIE 非空 goals；SSMI 地图增长 + 非空 path）。尚未进入官方平台完整探索闭环、运动后端集成或实机部署。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| 宿主环境盘点（22.04 / A6000 / Docker nvidia） | 已验证 | `reports/host_environment.md` |
| Git 仓库 `main` 初始化并推送 origin | 已验证 | commit `d90c250`，与 `origin/main` 一致 |
| 镜像 `nbv-fkie:noetic` / `nbv-ssmi:noetic` | 已验证 | 本地 images；容器可启动 |
| FKIE catkin 编译 | 已验证 | `ws_fkie/devel` |
| FKIE Action 调用 `/robot/nbv_rrt` | 已验证（调用层） | `logs/fkie_smoke_run.log` |
| **FKIE 有效 NBV（非空 goals）** | **已验证（适配仿真）** | `logs/fkie_effective_nbv_run6.log` → `FKIE_EFFECTIVE_NBV_OK goals=1`；`data/metrics/fkie_effective_nbv/RESULT.md` |
| SSMI catkin 编译（9 packages） | 已验证 | `logs/ssmi_build2.log` → `SSMI_BUILD_OK` |
| SSMI RGB-D + 语义相机持续出图 | 已验证 | `logs/ssmi_smoke_run.log` |
| **SSMI 地图增长 + 非空规划路径** | **已验证** | `logs/ssmi_effective/run3.log` → `SSMI_EFFECTIVE_OK`；path poses=10；`data/metrics/ssmi_effective/RESULT.md` |
| `/workspace` 挂载读写持久化 | 已验证 | 容器可见项目文件 |
| 共享记忆机制四文件 | 本轮更新 | `AGENTS.md` 等 |

## 部分完成 / 边界说明

| 项 | 说明 |
|----|------|
| FKIE 有效 NBV | 适配最小 world（非官方移动操作平台栈）；验证层级=规划调用成功 + 非空 goals，**不是**完整探索闭环/运动执行 |
| FKIE 室内仓库可视化复现 | `scripts/fkie_warehouse_nbv_demo.sh` 可起 warehouse + 深度相机 + GUI；**Action SUCCEEDED 但 goals=0** |
| **AGX + 仓库感知链路** | `scripts/fkie_agx_warehouse_nbv.sh`：整机 spawn、D455 depth=`32FC1`、单路 points2、TF/OctoMap 已通；**NBV 仍 goals=0** |
| SSMI 规划质量 | 路径非空与官方 `New path published!` 已确认；路径质量/闭环运动未做定量验收 |
| SSMI teleop | 无 TTY 下失败（预期） |

## 未完成 / 未验证

- AGX+仓库场景 FKIE **非空 goals**（frontiers/RRT 扩不开）
- AGX 运动执行 / 自主探索闭环
- REMANI / TopAY 运动后端接入
- 4070 Laptop 环境复用验证
- 任何**实机**部署

## 当前运行态（易变）

- 容器：`nbv_fkie`、`nbv_ssmi`（`network_mode: host`）
- 第三方 pin：FKIE `91091f8`；SSMI-example `750122b`
- 仓库模型资产：`scripts/fetch_fkie_gazebo_models.sh` → `third_party/gazebo_models/`（gitignore）

## 下一步计划

1. 修仓库场景 NBV（深度话题分离、地图/前沿、相机朝向）争取非空 goals。
2. 完善可视化（RViz 相机视图；可选示意 moma，非官方原模）。
3. 评估 REMANI / TopAY 接口与话题约定。
