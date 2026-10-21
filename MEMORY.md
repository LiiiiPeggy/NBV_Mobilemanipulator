# MEMORY.md — 长期技术知识

## 宿主与容器基线

- 宿主机：Ubuntu 22.04.5，RTX A6000，Driver 550.163.01，CUDA 12.4（驱动侧），Docker 27.3.1，nvidia runtime 可用。
- 容器基镜像：`osrf/ros:noetic-desktop-full` → Ubuntu 20.04.6 + ROS Noetic + Gazebo Classic 11。
- SSMI 额外：`ignition-fortress`（`IGNITION_VERSION=fortress`，实测 `ign gazebo` 版本 6.17.1）。
- GPU 透传：Compose `runtime: nvidia` + `NVIDIA_VISIBLE_DEVICES=all` + `NVIDIA_DRIVER_CAPABILITIES=all`。
- X11：挂载 `/tmp/.X11-unix` 与 `/tmp/.docker.xauth.nbv`；由 `scripts/setup_x11.sh` 写入 FamilyWild cookie；禁止 `xhost +`。

## FKIE

- 源码 SHA：`fkie-nbv-planner` `91091f8a…`；依赖 `fkie_ddynamic_reconfigure`、`fkie_environmental_measurements`。
- Action 名：节点 ns=`robot` 下 `nbv_rrt` → `/robot/nbv_rrt`（见 `NBVPlanner.cpp`）。
- 规划输入：`camera_pose`、`octomap`（launch 中 remap 为 `octomap_full`）、`robot_bbx`（MBF footprint）。
- 官方 `run_planner.launch` 使用 gdb 前缀；Docker 验证用 `configs/fkie/run_planner_nogdb.launch` 或直接 `rosrun`。
- catkin 工作空间：`ws_fkie/src` 使用**相对**符号链接到 `third_party/...`（绝对链接在容器内无效）。

## SSMI

- 官方仓名：`ExistentialRobotics/SSMI-example`（小写 example）。
- 官方需求：Ubuntu 20.04 + ROS Noetic + Gazebo Fortress；Docker 在 22.04 宿主机上测过 —— 与本机一致。
- 关键话题：`/rgbd_camera/{image,depth_image,camera_info}`，`/semantic/{colored_map,labels_map}`，`/semantic_pcl/semantic_pcl`，`/octomap_full`，`/planner/path`。
- 编译需 `libgflags-dev`（`ros_ign_gazebo` 的 `create.cpp`）。
- `teleop_twist_keyboard` 在无 TTY 的 `docker exec` 下会因 `termios` 失败；不影响相机/地图。

## Compose / 运维

- 项目名：`name: nbv_fkie` / `name: nbv_ssmi`，容器名 `nbv_fkie` / `nbv_ssmi`。
- 均 `network_mode: host` + `ipc: host`；完整仿真不要两套同时跑（Gazebo Classic master `11345` 等冲突）。
- 可用 `GAZEBO_MASTER_URI=http://127.0.0.1:<port>` 隔离 Classic 实例。
- 镜像内不 COPY 项目源码；编译产物落在挂载目录并 gitignore。
