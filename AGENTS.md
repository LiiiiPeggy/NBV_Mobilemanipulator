# AGENTS.md — 工作规则

本文件供 Cursor、Codex、Codex Desktop、Claude Code 等 AI Agent **共享**使用。  
公共记忆仅使用仓库内四个文件：`AGENTS.md`、`PROGRESS.md`、`MEMORY.md`、`PROCESS.md`。  
**不**创建 Agent 专属记忆文件，**不**依赖各产品私有 memory。

## Project Memory

Before substantial work:
- Read PROGRESS.md.
- Read relevant MEMORY.md sections.
- Read PROCESS.md when historical reasoning is needed.
- Trust current code, configuration, tests, and runtime evidence over outdated documents.

After substantial work:
- Update PROGRESS.md when project state changes.
- Update MEMORY.md when durable knowledge is discovered.
- Update PROCESS.md when important engineering history should be preserved.

日常约定：
- 开始重要任务前：读本文件与 `PROGRESS.md`；按需读 `MEMORY.md` / `PROCESS.md`。
- 完成后只更新实际变化的文件；避免同一内容重复写入多个文件；不要保存完整聊天记录。
- 缺乏证据的结论在记忆中标记为「待验证」，禁止编造。

## 项目范围

- 唯一项目根：`/home/ubuntu/locomani/NBV_Mobilemanipulator`
- 禁止在其他目录创建第二份项目副本
- GitHub：`git@github.com:LiiiiPeggy/NBV_Mobilemanipulator.git`
- 目标算法基线：FKIE NBV Planner、SSMI；后续运动后端候选：REMANI / TopAY

## 环境与 Docker 管理

1. 宿主机 A6000 保持 **Ubuntu 22.04**；不重装、不降级。
2. 本项目 Docker 统一以 **Ubuntu 20.04 + ROS Noetic** 为目标。
3. FKIE 与 SSMI 使用**独立** Dockerfile、Compose、容器（`docker-compose.fkie.yml` / `docker-compose.ssmi.yml`）。
4. 不修改宿主机 NVIDIA 驱动、CUDA 或其他项目配置。
5. 不停止、不修改、不复用他人容器；不执行 `docker system prune`；不删除他人镜像/网络/卷。
6. 源码通过 `.:/workspace` 挂载，不把项目源码只 COPY 进镜像。
7. GPU / DISPLAY / X11 用最小必要权限；**禁止** `xhost +`；使用 `scripts/setup_x11.sh`。
8. 不删除 `third_party/*` 内第三方仓库自己的 `.git`；不覆盖未提交的重要工作。
9. 官方依赖无法在 20.04 下合理运行时：先分析并写入报告/记忆，**不得擅自改系统版本**。

## 修改原则（已验证方案保护）

- 修改已验证方案前，必须检查相关记忆（`MEMORY.md` / `PROCESS.md`）与验证证据（`reports/`、`logs/`、测试脚本输出）。
- **不要**仅因为新方案看起来更简洁或优雅，就替换已有有效实现。
- 信任当前代码、配置、测试与运行时证据，胜过过时文档。
- 完成状态必须有验证证据；不得把计划写成已完成；不得把仿真结果写成实机验证。

## 验证层级（必须区分）

记录与宣称成功时，严格区分：

1. **编译成功**（catkin build）
2. **规划器/节点启动成功**
3. **规划调用成功**（如 Action goal 被接受并返回）
4. **完整仿真闭环成功**（传感→地图→规划→控制/运动）
5. **实机验证成功**（当前阶段尚未开展）

「Action SUCCEEDED 但 `goals=[]`」≠ 完整规划成功。

## 测试要求

- 改 Docker / 启动脚本后：至少跑对应 compose `config`，并在容器内确认 `/workspace` 挂载。
- 改 FKIE 规划相关：优先 `scripts/fkie_smoke_test.sh` 或等价 Action 调用。
- 改 SSMI 传感/地图相关：优先 `scripts/ssmi_smoke_test.sh`，确认 depth / semantic / octomap / planner 话题有数据。
- FKIE 与 SSMI **完整仿真**不要并行排查同一问题（host 网络下 Gazebo master 等端口易冲突）。
- 关键实验应记录：运行环境、代码 commit、命令、验证结果（可写入 `reports/`，要点写入记忆）。

## 文档维护

| 文件 | 职责 |
|------|------|
| `AGENTS.md` | 工作规则（本文件） |
| `PROGRESS.md` | 当前状态与下一步（只保留最新） |
| `MEMORY.md` | 长期有效知识 |
| `PROCESS.md` | 工程历史与根因（不得无故删除） |
| `reports/*` | 详细复现证据（非记忆正文） |

- 更新记忆前先读现有内容并保留有效信息。
- 不要把完整日志、临时任务或未验证猜测写入 `MEMORY.md`。
- 不要把全文历史复制到 `MEMORY.md`；历史留在 `PROCESS.md`。
