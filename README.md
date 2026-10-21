# NBV_Mobilemanipulator

A6000 工作站上的 FKIE NBV Planner 与 SSMI Docker 仿真环境（容器内 Ubuntu 20.04 + ROS Noetic）。宿主机保持 Ubuntu 22.04。

## 目录结构

```text
NBV_Mobilemanipulator/
├── docker/fkie|ssmi/     # 独立 Dockerfile
├── docker-compose.fkie.yml
├── docker-compose.ssmi.yml
├── scripts/              # 构建/启停/验证
├── third_party/          # 官方源码（各自保留 .git）
├── ws_fkie/              # FKIE catkin 工作空间（相对符号链接）
├── configs/ data/ logs/ reports/
└── AGENTS.md PROGRESS.md MEMORY.md PROCESS.md
```

项目根目录挂载为容器内 `/workspace`，宿主机编辑即时可见。

## 快速开始

```bash
cd /home/ubuntu/locomani/NBV_Mobilemanipulator

# X11 cookie（不使用 xhost +）
source scripts/setup_x11.sh

# FKIE
./scripts/build_fkie.sh
./scripts/start_fkie.sh
docker exec -it nbv_fkie bash
# 容器内：bash /workspace/scripts/fkie_smoke_test.sh

# SSMI（独立栈，勿与 FKIE 同时跑完整仿真抢 Gazebo master / GPU）
./scripts/build_ssmi.sh
./scripts/start_ssmi.sh
docker exec -it nbv_ssmi bash
# 容器内：bash /workspace/scripts/ssmi_smoke_test.sh

./scripts/stop_fkie.sh
./scripts/stop_ssmi.sh
```

Compose 校验：

```bash
docker compose -f docker-compose.fkie.yml config
docker compose -f docker-compose.ssmi.yml config
```

## 环境要点

| 项 | 说明 |
|----|------|
| 宿主机 | Ubuntu 22.04，RTX A6000，Driver 550，Docker nvidia runtime |
| 容器 | Ubuntu 20.04，`osrf/ros:noetic-desktop-full` |
| FKIE | Gazebo Classic 11 + `fkie_nbv_planner` Action `/robot/nbv_rrt` |
| SSMI | Ignition Fortress + 官方 `SSMI-example` |
| GPU/X11 | `runtime: nvidia`，挂载 `/tmp/.X11-unix` + Xauth cookie，无 `privileged` / `xhost +` |

## 复现报告

见 `reports/`：`host_environment.md`、`fkie_reproduction.md`、`ssmi_reproduction.md`、`reproduction_summary.md`。

## 注意

- 不要停止/修改他人容器或执行 `docker system prune`。
- 编译产物在 `ws_fkie/{build,devel}` 与 `third_party/SSMI-Example/ssmi_ws/{build,devel}`，已 gitignore。
- FKIE 官方完整探索闭环还依赖其移动操作平台栈（RealSense Gazebo 插件、MBF 等），本仓库已验证 Action Server 与 world 加载；完整机器人闭环需后续接入。
