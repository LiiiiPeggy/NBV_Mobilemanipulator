# PROGRESS.md — 当前状态

更新时间：2026-10-22  
证据优先于本文；冲突时以代码、`reports/`、容器内实测为准。

## 研究目标与阶段

**目标**：在 A6000 上复现 FKIE NBV 与 SSMI 基线，建立可复用 Docker 仿真，后续接入运动规划后端（REMANI / TopAY）并服务移动操作主动视角实验。

**当前阶段**：Docker 仿真环境已搭好并完成分层验证；**尚未**进入完整 FKIE 机器人探索闭环、运动后端集成或实机部署。

## 已完成（有验证证据）

| 项 | 验证状态 | 证据 |
|----|----------|------|
| 宿主环境盘点（22.04 / A6000 / Docker nvidia） | 已验证 | `reports/host_environment.md` |
| Git 仓库 `main` 初始化并推送 origin | 已验证 | commit `d90c250`，与 `origin/main` 一致 |
| 镜像 `nbv-fkie:noetic` / `nbv-ssmi:noetic` | 已验证 | 本地 images；容器可启动 |
| FKIE catkin 编译（含 msgs / ddynamic_reconfigure / planner） | 已验证 | `ws_fkie/devel`；会话构建日志 |
| FKIE Action 调用 `/robot/nbv_rrt` | 已验证（调用层） | `logs/fkie_smoke_run.log` → `VERIFY_FKIE_ACTION_OK` |
| FKIE Gazebo Classic 加载 `warehouse_icra_large.world` | 已验证（world 加载） | `logs/fkie_gazebo_verify*.log`；曾有 gzserver/gzclient |
| SSMI catkin 编译（9 packages） | 已验证 | `logs/ssmi_build2.log` → `SSMI_BUILD_OK` |
| SSMI RGB-D + 语义相机持续出图 | 已验证 | `logs/ssmi_smoke_run.log`（image/depth/semantic count≥3） |
| SSMI Semantic PCL / Octomap / 规划话题出现 | 已验证（话题层） | 同上：`/semantic_pcl/semantic_pcl`、`/octomap_full`、`/planner/path` |
| `/workspace` 挂载读写持久化 | 已验证 | 容器可见项目文件；`logs/mount_write_test.txt` |
| 共享记忆机制四文件 | 本轮更新 | `AGENTS.md` 等 |

## 部分完成 / 边界说明

| 项 | 说明 |
|----|------|
| FKIE「规划调用成功」 | Action 状态 SUCCEEDED，但 smoke 使用占位 octomap → **`goals=[]`**，不能当作有效 NBV 路径 |
| SSMI「规划器生成有效规划」 | 规划节点与 `/planner/path` 等话题已出现；**路径质量/闭环运动未做定量验收** |
| SSMI teleop | 无 TTY 下 `teleop_twist_keyboard` 失败（预期），不阻塞相机/地图 |

## 未完成 / 未验证

- FKIE 完整仿真探索闭环（RealSense 插件点云 → octomap → footprint → 非空 goals → 执行）
- REMANI / TopAY 运动后端接入
- 4070 Laptop（Ubuntu 20.04）上的环境复用验证
- 任何**实机**部署或实机数据采集验收
- 对象记忆模块（若后续引入）的设计落地与实验

## 当前运行态（易变）

- 容器名：`nbv_fkie`、`nbv_ssmi`（`network_mode: host`）
- 第三方 pin：FKIE `91091f8`；SSMI-example `750122b`（详见 `third_party/README.md`）
- 编译产物属主常为 root（容器内构建）；宿主机删改可能需经容器 `rm`

## 当前问题

1. host 网络下 FKIE（Gazebo Classic）与 SSMI（Ignition）完整仿真并行易抢端口/GPU。
2. A6000 上其他进程（如 VLLM）可能占用大量显存，影响相机渲染稳定性。
3. FKIE 官方平台栈未纳入本仓库，阻塞完整闭环。

## 下一步计划

1. 为 FKIE 接入完整仿真机器人与真实 octomap 输入，验证非空 `goals[]`。
2. 明确 SSMI 规划结果的定量验收（路径/探索指标）。
3. 评估 REMANI / TopAY 接口与话题约定。
4. 在 4070 Laptop 复用同一 Compose 做开发调试。
