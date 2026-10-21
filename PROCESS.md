# PROCESS.md — 工程历史与根因

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
