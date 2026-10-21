# AGENTS.md — 工作规范

## 范围

- 唯一项目根：`/home/ubuntu/locomani/NBV_Mobilemanipulator`
- 禁止在其他目录创建第二份项目副本
- 宿主机保持 Ubuntu 22.04；容器目标 Ubuntu 20.04 + ROS Noetic

## 修改原则

1. 不修改宿主机 NVIDIA 驱动、CUDA 或其他项目配置。
2. 不停止、不修改、不复用他人容器；不执行 `docker system prune`。
3. FKIE 与 SSMI 使用独立 Dockerfile / Compose / 容器。
4. 源码通过 `.:/workspace` 挂载，不把项目源码只 COPY 进镜像。
5. 不使用 `xhost +`；用 `scripts/setup_x11.sh` 生成 Xauth cookie。
6. 官方依赖无法在 20.04 合理运行时，先分析报告，不得擅自改系统版本。
7. 完成状态必须有验证证据；不得把计划写成已完成。

## 记忆文件

| 文件 | 用途 |
|------|------|
| `PROGRESS.md` | 当前状态与下一步 |
| `MEMORY.md` | 长期有效技术知识 |
| `PROCESS.md` | 工程历史与根因 |
| `reports/*` | 复现证据 |

更新记忆前先读现有内容，保留有效信息。
