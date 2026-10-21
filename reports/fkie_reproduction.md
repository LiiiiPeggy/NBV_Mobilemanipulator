# FKIE 复现报告

日期：2026-10-21

## 源码

| 仓库 | Commit |
|------|--------|
| fkie/fkie-nbv-planner | `91091f8a58301a69bbf240bce073059092e6f606` |
| fkie/fkie_ddynamic_reconfigure | `7613e5319d815741fedc762cb39d19b3cc3cb6af` |
| fkie/fkie_environmental_measurements | `75d331770c4c43fc0247876a192e5dd6809cd759` |

路径：`third_party/` + `ws_fkie/src` 相对符号链接。

## 镜像与依赖

- 基础：`osrf/ros:noetic-desktop-full`（Ubuntu 20.04 + ROS Noetic + Gazebo Classic 11.15.1）
- 本项目镜像：`nbv-fkie:noetic`
- Dockerfile：`docker/fkie/Dockerfile`
- Compose：`docker-compose.fkie.yml`（`name: nbv_fkie`，`.:/workspace`，nvidia runtime，Xauth，无 privileged）

## 构建与编译

```bash
source scripts/setup_x11.sh
./scripts/setup_fkie_ws.sh
docker compose -f docker-compose.fkie.yml build
docker compose -f docker-compose.fkie.yml run --rm --entrypoint bash fkie -lc \
  'source /opt/ros/noetic/setup.bash && cd /workspace/ws_fkie && \
   catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release && \
   catkin build fkie_ddynamic_reconfigure fkie_measurement_msgs fkie_nbv_planner'
```

结果：`All 4 packages succeeded`（含 prebuild）。证据：编译日志会话内确认 `FKIE_BUILD_OK`，`rospack find fkie_nbv_planner` 成功。

## Action Server 验证

命令：`bash /workspace/scripts/fkie_smoke_test.sh`

证据（`logs/fkie_smoke_run.log`）：

- Action topics：`/robot/nbv_rrt/{goal,feedback,result,status,cancel}`
- `VERIFY_FKIE_ACTION_OK`，Action state=3（SUCCEEDED）
- 结果示例：`complete_exploration=True goals=0`（占位 octomap，**非**完整规划成功）

## Gazebo / 地图链路

- `gazebo --version` → 11.15.1
- `warehouse_icra_large.world`：曾成功启动 `gzserver`+`gzclient`（长时间运行于 DISPLAY=:1）；后续用 `GAZEBO_MASTER_URI=http://127.0.0.1:11355` 复现 `Loading world file [...]warehouse_icra_large.world`
- 官方完整链路还需：RealSense Gazebo 插件点云 → `depth_image_proc` → `octomap_server` → planner；本仓库未包含完整机器人描述，**完整探索闭环未宣称完成**

## 卷映射

- 宿主机写 `logs/mount_write_test.txt` / 容器写回：已验证
- 容器可见 `docker-compose.fkie.yml` 等项目文件：已验证

## 未解决问题

1. 无官方移动操作平台仿真时，无法验证非空 `goals[]` 与真实探索闭环。
2. host 网络下与 SSMI/残留 Gazebo 可能争用 master 端口。
