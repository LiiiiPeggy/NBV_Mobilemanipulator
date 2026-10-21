# SSMI 复现报告

日期：2026-10-21

## 源码

| 仓库 | Commit |
|------|--------|
| ExistentialRobotics/SSMI-example | `750122b8dbb193dcf67af5429be6c8df3b3b462e` |
| ExistentialRobotics/SSMI（子模块） | `727d8152990629bc44ca500015ecdb83668f7180` |
| gazebosim/ros_gz（子模块） | `28a0a75160794451c027b125a6edef1bf919d342` |

路径：`third_party/SSMI-Example/`（官方 Dockerfile/Compose 保留作参考；本项目使用根目录独立 Compose）。

## 版本核实

| 项 | 官方声明 | 本环境 |
|----|----------|--------|
| 容器 OS | Ubuntu 20.04（ros:noetic） | Ubuntu 20.04.6 Focal |
| ROS | Noetic | Noetic |
| 仿真 | Gazebo Fortress | Ignition Fortress（`ign gazebo` 6.17.1） |
| 宿主机 Docker 测试 | Ubuntu 22.04 + NVIDIA | Ubuntu 22.04 + A6000 |

**结论**：与「容器 Ubuntu 20.04」约束兼容；未改系统版本。

## 镜像与配置

- 基础：`osrf/ros:noetic-desktop-full` + `ignition-fortress` + `libgflags-dev`
- 镜像：`nbv-ssmi:noetic`
- Dockerfile：`docker/ssmi/Dockerfile`（相对官方：不 COPY 源码进镜像，挂载 `/workspace`；去掉 privileged）
- Compose：`docker-compose.ssmi.yml`

## 构建

首次失败：`rosdep update` 超时 → 改为非致命。  
二次失败：缺 `gflags` → 安装 `libgflags-dev` 后 `All 9 packages succeeded`。

证据：`logs/ssmi_build2.log` 含 `SSMI_BUILD_OK`。

## 运行验证（`scripts/ssmi_smoke_test.sh`）

证据：`logs/ssmi_smoke_run.log`

### Gazebo / 图形

- `glxinfo`：NVIDIA RTX A6000，OpenGL 4.6，direct rendering Yes（`logs/ssmi_glxinfo.txt`）
- `seg_husky.launch` 拉起 Ignition + ros_ign_bridge + rviz

### 深度与语义相机（持续有效数据）

5 秒窗口内收到消息：

| Topic | 类型 | count |
|-------|------|-------|
| `/rgbd_camera/image` | Image | ≥3 |
| `/rgbd_camera/depth_image` | Image | ≥3 |
| `/semantic/colored_map` | Image | ≥3 |
| `/semantic/labels_map` | Image | ≥6 |
| `/semantic_pcl/semantic_pcl` | PointCloud2 | ≥3 |

### Semantic 地图 / 规划

出现话题：`/octomap_full`、`/octomap_color`、`/occupancy_map_2D`、`/planner/path`、`/planner/position_cmd`、`/planner/collision`。

### 已知非阻塞问题

- `teleop_twist_keyboard` 无 TTY → termios 错误（预期）
- `XDG_RUNTIME_DIR` 警告

## 相对官方 Compose 的权限收敛

| 官方 | 本项目 |
|------|--------|
| `privileged: true` | 未使用 |
| 注释建议 `xhost +local:docker` | `setup_x11.sh` cookie |
| 无项目挂载 | `.:/workspace:rw` |
| 源码 COPY 进镜像 | 仅安装依赖，源码挂载编译 |
