# PROGRESS.md

更新时间：2026-10-21

## 当前状态

| 项 | 状态 |
|----|------|
| 宿主环境检查 | 完成 → `reports/host_environment.md` |
| Git 仓库初始化 | 完成（远程曾为空，待推送） |
| FKIE Docker 镜像 `nbv-fkie:noetic` | 已构建 |
| FKIE catkin 编译 | 成功（`fkie_nbv_planner` 等） |
| FKIE Action `/robot/nbv_rrt` | 已调用并 SUCCEEDED |
| FKIE Gazebo Classic world | 已加载 `warehouse_icra_large.world`（含 GUI 进程证据） |
| FKIE 完整机器人探索闭环 | 未完成（缺官方移动操作平台/RealSense 插件栈） |
| SSMI Docker 镜像 `nbv-ssmi:noetic` | 已构建（含 Ignition Fortress + libgflags） |
| SSMI catkin 编译 | 成功（9 packages） |
| SSMI RGB-D / 语义相机 | 持续收消息（depth/image/semantic maps） |
| SSMI Semantic OctoMap / PCL | `/octomap_full`、`/semantic_pcl/semantic_pcl` 已出现 |
| SSMI 规划话题 | `/planner/path`、`/planner/position_cmd` 已出现 |
| 卷映射 `/workspace` | FKIE/SSMI 均已验证读写持久化 |

## 下一步

1. 为 FKIE 接入完整仿真机器人（RealSense Gazebo 插件 + octomap 真值点云 + MBF footprint），验证非空 `goals[]` 的规划结果。
2. 评估 REMANI / TopAY 作为运动规划后端的接口。
3. 在 4070 Laptop（Ubuntu 20.04）上复用同一套 Compose 做开发调试。
4. 约定 FKIE/SSMI 不同时跑完整仿真（`network_mode: host` 下 Gazebo master 端口冲突）。
