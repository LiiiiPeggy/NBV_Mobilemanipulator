# A6000 宿主环境检查报告

检查时间：2026-10-21  
项目根目录：`/home/ubuntu/locomani/NBV_Mobilemanipulator`

## 1. 操作系统与硬件

| 项目 | 结果 |
|------|------|
| OS | Ubuntu 22.04.5 LTS (jammy) |
| CPU | 32 核 |
| 内存 | 62 GiB（可用约 39 GiB） |
| 磁盘 `/` | 3.6T 总量，已用约 89%，可用约 386G |
| GPU | NVIDIA RTX A6000，49140 MiB |
| 驱动 | 550.163.01 |
| CUDA（驱动报告） | 12.4 |
| DISPLAY | `:1` |
| X11 | `/tmp/.X11-unix/X1` 存在；`XAUTHORITY=/run/user/1000/gdm/Xauthority` |

**约束确认**：宿主机保持 Ubuntu 22.04，不重装、不降级；容器目标为 Ubuntu 20.04 + ROS Noetic。

## 2. Docker / NVIDIA Container Toolkit

| 项目 | 结果 |
|------|------|
| Docker | 27.3.1 |
| Compose | v2.29.7 |
| nvidia-ctk | 1.13.5 |
| Runtimes | `runc`, `nvidia`, `io.containerd.runc.v2` |
| 默认 Runtime | runc |
| Docker Root | `/home/docker` |
| 用户组 | `ubuntu` 属于 `docker` |

GPU 透传验证（未改驱动/宿主机配置）：

```bash
docker run --rm --gpus all nvidia/cuda:11.3.1-cudnn8-devel-ubuntu20.04 nvidia-smi
```

结果：容器内可见 A6000，驱动 550.163.01，CUDA 兼容层正常。

## 3. 已有容器（他人/其他项目，禁止触碰）

| Names | Image | Status |
|-------|-------|--------|
| docker-scene-graph-run-1e1d1e670150 | scene_graph:latest | Up |
| lxy | d8ce63b20c71 | Up |
| zxg | 1256fa5b1b7d | Up |
| zfy | d8ce63b20c71 | Up |
| hkp | d8ce63b20c71 | Up |
| chn | d8ce63b20c71 | Up |
| cjf | d8ce63b20c71 | Up |
| gly | 120076121985 | Up |
| yly | 1256fa5b1b7d | Up |

原则：不停止、不修改、不复用这些容器；不执行 `docker system prune`。

## 4. 已有镜像与复用分析

| Repository:Tag | 基础系统（核实） | 是否适合作为本项目基础 |
|----------------|------------------|------------------------|
| nvidia/cuda:11.3.1-cudnn8-devel-ubuntu20.04 | Ubuntu 20.04.6 | 有 GPU，但无 ROS/Gazebo，不直接复用为 FKIE/SSMI 运行镜像 |
| nvidia/cuda:11.3.1-cudnn8-devel-ubuntu20.04_with_ssh_and_nvcc | Ubuntu 20.04（同族） | 他人常用基础；无 ROS，不复用 |
| nvidia/cuda:*ubuntu22.04* | Ubuntu 22.04 | 与项目「容器内 20.04」目标不符 |
| scene_graph:latest | 未知（他项目） | 不复用 |
| hello-world | — | 无关 |

**结论**：宿主机无现成 ROS Noetic / Gazebo 镜像。FKIE 与 SSMI 均以官方兼容的 `osrf/ros:noetic-desktop-full`（Ubuntu 20.04 + ROS Noetic）为新建镜像基础，通过 Compose 启用 NVIDIA runtime / GPU，不修改他人容器与宿主机驱动。

## 5. GPU 占用注意

检查时 A6000 显存约 **36696 / 49140 MiB** 已被占用，其中 `VLLM::EngineCore` 约 34.9 GiB。仿真与相机渲染可能受显存压力影响；本项目不修改其他进程，仅在报告中记录风险。

## 6. GitHub 认证

```text
ssh -T git@github.com
# Hi LiiiiPeggy! You've successfully authenticated...
```

远程仓库 `git@github.com:LiiiiPeggy/NBV_Mobilemanipulator.git` 当时为空（`git ls-remote` 无 refs）。

## 7. 未执行的操作

- 未修改 NVIDIA 驱动 / CUDA / 其他项目配置
- 未停止或删除任何已有容器 / 镜像 / 网络 / 数据卷
- 未执行 `docker system prune`
- 未使用 `xhost +`
