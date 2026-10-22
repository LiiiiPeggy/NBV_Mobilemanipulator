# PROCESS.md — 工程历史与根因

保留可追溯技术决策。已有条目无故不删；细节证据可指向 `reports/`，摘要不搬迁到 `MEMORY.md`。

---

## 2026-10-21 环境搭建

### 决策：不复用他人 CUDA 容器

宿主机已有多人长期容器（`lxy/zxg/...`）与 `nvidia/cuda:*ubuntu20.04*` 镜像，但均无 ROS/Gazebo。选择新建 `osrf/ros:noetic-desktop-full` 衍生镜像 `nbv-fkie:noetic` / `nbv-ssmi:noetic`，避免干扰他人环境。

### 根因：catkin 包在容器内“消失”

`setup_fkie_ws.sh` 最初用绝对路径符号链接（`/home/ubuntu/...`）。容器内该路径不存在 → `catkin build` 报 `No packages were found`。  
**修复**：改为相对链接 `../../third_party/...`。

### 根因：SSMI 镜像构建失败（rosdep）

`rosdep update` 拉取 rosdistro 超时导致 `docker build` 失败。  
**修复**：`(rosdep update || true)`；运行时再 best-effort `rosdep install`。

### 根因：SSMI `ros_ign_gazebo` 编译失败

缺少 `gflags/gflags.h`。  
**修复**：镜像与容器安装 `libgflags-dev`。

### 根因：`pkill -f ign` 误杀验证脚本

命令行字符串包含 `ign`，`pkill -f ign` 匹配到自身 bash。  
**修复**：`ssmi_smoke_test.sh` 用 `rosnode kill` / job PID，避免宽泛 `pkill -f`。

### 观察：Gazebo Classic master 端口占用

`network_mode: host` 下，早先挂起的 `gzserver/gzclient` 占用 `11345`，新实例报 `Address already in use`。  
**缓解**：停残留进程，或设独立 `GAZEBO_MASTER_URI`；文档约定 FKIE/SSMI 完整仿真不同时运行。

### 观察：FKIE Action 返回空 goals

Smoke test 用空/占位 octomap 与假 camera_pose，Action 状态 SUCCEEDED，但 `goals=0` 且可能 `complete_exploration=True`。说明 **Action 通路已通**，不代表完整探索规划成功。完整闭环需真实点云→octomap→footprint。

### 观察：A6000 显存紧张

检查时 VLLM 占用约 35GB/49GB；仿真仍可跑，但 GPU 相机渲染可能受压。未改动其他进程。

### 决策：第三方源码不进主仓大提交

`third_party/*` 各含独立 `.git` 与大资源（demo.gif 等）。主仓用 `third_party/README.md` + `scripts/clone_third_party.sh` 钉 SHA，`.gitignore` 排除克隆目录，避免嵌套 gitlink 与大文件污染。

### 决策：权限收敛相对官方 SSMI Compose

官方示例使用 `privileged: true` 并建议 `xhost +local:docker`。本项目改为 nvidia runtime + `/dev/dri` + Xauth cookie + 项目 bind-mount，在 A6000 上已跑通相机与语义话题；若未来某驱动/渲染场景失败，应先对照官方权限再评估是否放宽，而不是默认改回 `xhost +`。

---

## 2026-10-22 共享项目记忆机制

### 背景

多 Agent（Cursor / Codex / Claude Code 等）需跨会话恢复同一项目状态，避免重复踩坑与误改已验证 Docker 方案。

### 方案

仅用仓库内四文件共享记忆：`AGENTS.md`（规则）、`PROGRESS.md`（最新状态）、`MEMORY.md`（长期知识）、`PROCESS.md`（本文件）。不引入向量库、Agent 私有 memory 或额外专属文件。

### 文档校正

- 旧 `PROGRESS.md` 仍写「远程待推送」；实际已于 `d90c250` 推送 `origin/main`。已按验证证据更正。
- 明确验证层级，防止将 Action/话题出现误记为完整闭环或实机成功。

---

## 2026-10-22 FKIE 有效 NBV + SSMI 地图/规划验证

### FKIE：深度插件与 ROS API

1. `libgazebo_ros_openni_kinect.so` 依赖 `libDepthCameraPlugin.so` → 需 `LD_LIBRARY_PATH` 含 gazebo-11 plugins。
2. 裸 `gzserver` 会报 ROS not initialized；改用 `rosrun gazebo_ros gzserver`。
3. planner 未 remap `octomap→octomap_full` 时日志 “No octomap received”，虽 `/robot/octomap_full` 有数据。
4. `verify_fkie_action` 原用原点 `±5` 边界，相机在 `(-5.36,-5.685)` 外 → “camera pose is not inside the polygon” → `goals=0`。改为官方 ROI 后 `goals=1`，`VERIFY_FKIE_NONEMPTY_GOALS`。

证据：`logs/fkie_effective_nbv_run6.log`，`data/metrics/fkie_effective_nbv/`。

### SSMI：octomap 增长与路径一次性发布

- T0/T1（约 15s）`/octomap_full` data_len 增长已验证。
- `/planner/path` 存在但常只发一次；后启动的 `wait_for_message` 易漏。改为 planner 启动即 recorder → 收到 10 poses，`SSMI_EFFECTIVE_OK`。

证据：`logs/ssmi_effective/run3.log`，`data/metrics/ssmi_effective/`。

---

## 2026-10-22 FKIE 室内仓库可视化复现

### 现象

- 最小 world 已能 `goals=1`；换 `warehouse_icra_large` + spawn 深度相机后，Action SUCCEEDED 但 **goals=0**（`complete_exploration=True`，planner：`No frontiers` / RRT 无法扩展）。
- Gazebo 可见货架；货箱模型在仿真中存在，但原 world 内嵌 DAE 贴图常导致货箱“看起来没有”。
- RViz 适配配置 initially 无 Image/Camera 面板；官方仓库无移动机械臂 URDF。

### 决策 / 产物

- 模型下载脚本 `scripts/fetch_fkie_gazebo_models.sh`；`gazebo_models/` gitignore。
- Demo：`scripts/fkie_warehouse_nbv_demo.sh`；适配 world `configs/fkie/adapt_test/warehouse_icra_large_viz.world`（货箱改实体盒材质）；示意 moma SDF/URDF（非官方）。
- GUI：`rosrun gazebo_ros gazebo`；`GAZEBO_MASTER_URI` 默认 `11355` 避冲突。

证据：`logs/fkie_warehouse_nbv/`，`data/metrics/fkie_warehouse_nbv/summary.txt`。

---

## 2026-10-22 AGX 接入 + Gazebo 进程堆积

### 决策

用用户 AGX（Ranger+CR10+AG95+D455）替代示意 moma；不改 agx 原 URDF，运行时生成 Gazebo 适配 URDF。

### 根因：测试叠多个 gzserver

`rosrun gazebo_ros gazebo` 会 fork `gzserver`/`gzclient`；只杀脚本 PID 会留孤儿。`fkie_stop_sim.sh` 若 `pkill` 自身 demo 脚本会自杀。  
**修复**：启动前 `STOP_SIM_KEEP_PID=$$` 调用 stop；退出 `killall gzserver gzclient`；zombie 需 `compose down/up`。

### 现状

AGX 仓库场景感知/TF/OctoMap 已通；NBV 仍 `goals=0`（No frontiers / RRT 无法扩展）。证据：`logs/fkie_agx_warehouse_nbv/`。
