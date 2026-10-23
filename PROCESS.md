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

### 现状（已更新）

见下一节：Mode A 修复后仓库 **goals=1**。

---

## 2026-10-22 AGX Mode A：倾倒修复与仓库非空 NBV

### 根因：倾倒 / RViz 姿态错乱

1. spawn `z=0.05` 过低（轮心相对 base≈−0.258）→ 穿地弹起。  
2. Gazebo joint_state 插件与 fallback `/joint_states` **双发布**覆盖。  
3. `set_model_configuration` 一次性置角无保持力矩 → 动态模式下臂塌。  
4. p3d 与静态 TF 竞争 world→base。

### 修复（Mode A）

- URDF 生成加 `<static>true</static>`，去掉 js/p3d 插件。  
- spawn `z=0.36` + `-J` 观测构型；单一 joint_states 发布器；静态 TF。  
- RViz 拆 debug / camera；octomap 加长感知距离。

### 结果

- 直立 roll=pitch=0；AGX+60 boxes 在 world。  
- 仓库 Action **goals=1**，GOAL `(-4.419,-6.198,1.609)`。  
- 最小 world 回归 **goals=2**。  

证据：`reports/fkie_agx_integration.md`，`data/metrics/fkie_agx_warehouse_nbv/`，`logs/fkie_effective_nbv_regression/`。

---

## 2026-10-22 室内多对象最小闭环（P2–P5 / P4-A）

### 决策：不改已验证 Mode A 姿态与 FKIE 配置

对象场景与感知在适配层脚本中完成；`arm_view_pose.yaml` / warehouse NBV 脚本保持。本轮重启后仓库 NBV 仍 `goals=1`。

### 根因：物体在 TF 视锥内但 RGB 看不见

Mode A URDF `<static>true>` 冻结 Gazebo 关节。`/joint_states` 驱动的 TF（RViz/NBV）与 Gazebo `cr10_Link6` / D455 传感器位姿不一致；`SetModelConfiguration` 无法真正改静态模型关节。  
**后果**：按 TF 摆放的 YCB/球体投影进图像但深度/RGB 为天空；按 Gazebo 可见 ROI（约 `(-6.5,-7.0,1.0)`）摆放后相机能看到。

### 决策：P3 定位用经验标定外参

在可见 ROI 上用彩色斑块+深度与 GT 做 Umeyama，得到 `T_world_cam_calib.json`；`--world-tf-mode calib`。这是 Mode A 权宜之计，**不是**最终外参方案。Mode B / 关节一致后应改回 TF。

### 决策：感知先用 color/depth，不装 Grounded-SAM 进 FKIE 容器

容器无 torch；避免破坏 FKIE。后续独立感知容器 + ROS JSON 接口。

### P4-A

`nbv_to_remani_ee_goal.py` 用 live TF 将 FKIE 相机目标转为 `/ee_goal` PoseStamped（位置+姿态）。未启动 REMANI 规划节点；Mode A 不能执行运动。

### 预实验指标

thresh=0.35 m：tp=2，unique_recall=0.2，precision=0.125，mean_err≈0.178 m。  
证据：`reports/indoor_objects_min_loop.md`，`reports/indoor_objects_shots/`。

---

## 2026-10-23 几何 / TF / 点云修复

### 根因：baseline spawn 在货架内

`(-5.36,-5.685)` 与 `Shelves high` + 货箱 OBB 重叠。世界 AABB 对长货架过保守。  
**修复**：`SPAWN_PROFILE` + `spawn_warehouse_{baseline,aisle_safe}.yaml`；间隙用 OBB/SAT（`check_agx_spawn_clearance.py`）。baseline 保留作历史配置，默认改 aisle_safe。

### 根因：Mode A 下 Gazebo 臂 ≠ TF

`<static>true>` 忽略关节重配置。  
**修复**：`generate_gazebo_urdf.py --arm-pose` 将 CR10 关节 bake 为 fixed；gripper tip TF↔Gazebo 误差 ~1e-6 m。

### 根因：点云相对 TF「歪 90°」

Gazebo depth 沿传感器 **+X**；旧 `<pose>… -π/2 0 -π/2` 把传感器系叠到 optical 上 → 视线沿 optical +X，而 TF 以 +Z 为前向（夹角 90°）。探针：TF 光轴上绿球不可见。  
**修复**：传感器 pose 改 identity（挂在 `d455_link`）；修正后绿球落在图像中心、深度≈1.25 m。  
几何定位 median_err≈0.069 m（`gt_depth_tf_localization.json`）。弃用 calib 作默认。

### 后果：仓库 NBV goals

修复相机后 baseline 与 aisle_safe 均 `goals=0` + `complete_exploration=True`。历史 `goals=1` 在错误朝向下取得，不能再当几何真理。需在正确相机下重测 FKIE。

### 场景分离

`primitive_debug/` vs `ycb/`；min_loop `SCENE_KIND` + 正式 `DIST_THRESH=0.15`。

---

## 2026-10-23 spawn Z 轮胎穿地

`z=0.36` 时 `*_wheel_link` 中心≈0.10 m，而 `fr_wheel_link.STL` 半径≈0.153 m → 胎底约 −0.05 m。  
**修复**：spawn YAML 改为 `z=0.42`；实测胎底≈+0.007 m。历史选 0.36 是为避免 z=0.05 穿地弹起，仍低估了轮半径。

---

## 2026-10-23 P1-R / 独立 Grounded-SAM / REMANI 桥

### P1-R：正确光学下恢复非空 NBV

修光轴后曾 `goals=0`。  
**措施**：aisle_safe 专用臂姿、planner 东界放宽、`camera_pose` 用 `d455_depth_optical_frame`、诊断脚本。  
**结果**：`goals=1`，GOAL≈(-3.020,-5.294,1.609)。证据 `reports/fkie_corrected_optical_nbv.md`。

### 决策：从 DovSG 复制而非运行时依赖

推翻「host 挂 `conda dovsg`」方案。允许复制 GDINO/SAM2/权重到 `perception/vendor` + `checkpoints/`；禁止 PYTHONPATH/挂载/改 DovSG。env：`.conda_envs/nbv_perception`（不复用 dovsg）。独立性脚本校验 checksum + import 路径。

### GroundingDINO `_C` 未编译

CUDA 扩展缺失时原代码仍走 `MultiScaleDeformableAttnFunction` 导致 `NameError`。  
**修复**：`ms_deform_attn.py` 在 `_C is None` 时走 PyTorch fallback。GPU smoke：LOAD≈3s，INFER≈0.76s，~2.7GB。

### P4：`/ee_goal` 已发布、REMANI 未接

live FKIE 目标经 `nbv_to_remani_ee_goal.py --publish` 成功；`rostopic info /ee_goal` 无 REMANI 订阅者。Mode A 保留；Mode B dynamic 全量 bringup 留待执行节点就绪。

---

## 2026-10-23 YCB「看不见」根因与修复

### 根因 A：从未 Spawn

`fkie_agx_warehouse_nbv.sh` 不加载对象；min_loop 默认 `primitive_debug`。仅跑仓库脚本时 `mo_*` 数为 0。

### 根因 F：Spawn 成功但在相机背后

首版布局桌在 `(-2.55,-6.55)`，aisle_safe NBV 臂光轴偏角约 140°。Gazebo 内模型存在，D455 RGB 仍只有货架。

### 修复

- `spawn_object_scene.py` fail-closed + 存在/位姿复核；`verify_gazebo_object_scene.py`；GT 缺目标不 OK。
- `spawn_warehouse_aisle_safe_ycb.yaml` + `arm_observe_table`；布局对准光轴（桌 ~(-4.05,-4.70)）。
- 一键：`start_fkie_agx_ycb_demo.sh`。证据：`reports/ycb_gazebo_visibility_validation.md`，D455 可见糖盒/罐头/碗/香蕉/苹果等。

## 2026-10-23 桌塞货架底 / 物体悬空

### 根因

1. **FOV 对齐把桌推进了货架**：`(-4.05,-4.70)` 与 `Shelves high` OBB/SAT 重叠（旧 clearance 只保 AGX 底座，不保家具）。
2. **假货架层高**：`generate_object_scene.py` 用 `shelf_z=1.05/1.35`，仓库货架模型无对应板面 → 物体漂浮。
3. **`FORCE_RESTART` 未杀旧 gz**：旧光轴仍朝货架，新桌移开后画面仍像「塞进货架」。

### 修复

- 桌 → `(-3.90,-7.50)`（shelf≈2.7 m，nearest big_box≈1.9 m）；物体仅 `z∈{0,0.72}`。
- `arm_observe_aisle_table.yaml`（解析 URDF FK 搜索）；profile 指向该臂姿。
- `start_fkie_agx_ycb_demo.sh`：`FORCE_RESTART=1` 先 `fkie_stop_sim` / 清 gz 再 bringup。
- 证据：`VERIFY_GAZEBO_OBJECT_SCENE_OK`；live `mo_*` 无 `z>0.85`；optical ang≈16°；`reports/ycb_gazebo_shots/d455_rgb_seed42.png`。

## 2026-10-23 Stage 0–3 基准

- 接触高度：`ycb_model_contact_z.py`；`table_top_z=0.74`；seed42/7/99 布局校验与 spawn/verify OK → `reports/ycb_scene_placement_validation.md`。
- GSAM 单帧失败分析：TP=4 FN 主因 B（漏检）与 F（定位/关联）→ `reports/grounded_sam_object_failure_analysis.md`。
- 三视角 bake 记忆：naive FP25→memory FP6，TP=5 → `reports/multiview_object_memory.md`。
- 一键评测：`run_object_discovery_benchmark.sh` → `reports/object_discovery_benchmark.md`。
