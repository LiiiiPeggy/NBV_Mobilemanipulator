# MEMORY.md — 长期知识

只记录经代码/配置/运行验证、能避免重复排查或错误修改的知识。  
完整日志与一次性实验细节见 `reports/`、`logs/`；决策过程见 `PROCESS.md`。

## 1. 稳定环境约束

| 约束 | 内容 |
|------|------|
| 宿主机 | Ubuntu 22.04.5，RTX A6000，Driver 550.163.01，CUDA 12.4（驱动侧） |
| Docker | 27.3.1，Compose v2.29.7，runtime 含 `nvidia` |
| 容器目标 | Ubuntu 20.04 + ROS Noetic；基镜像 `osrf/ros:noetic-desktop-full` |
| 项目根 | `/home/ubuntu/locomani/NBV_Mobilemanipulator` → 容器 `/workspace` |
| 他人环境 | 宿主机上多人容器长期运行；**禁止**停改删他人资源 |

GPU Compose 要点：`runtime: nvidia`，`NVIDIA_VISIBLE_DEVICES=all`，`NVIDIA_DRIVER_CAPABILITIES=all`，挂载 `/dev/dri`。  
X11：`/tmp/.X11-unix` + `XAUTH_FILE=/tmp/.docker.xauth.nbv`（`scripts/setup_x11.sh`）；**禁止** `xhost +` / 默认不使用 `privileged`。

## 2. Docker 栈划分

| 栈 | Compose | 镜像 | 容器名 | 仿真 |
|----|---------|------|--------|------|
| FKIE | `docker-compose.fkie.yml`（`name: nbv_fkie`） | `nbv-fkie:noetic` | `nbv_fkie` | Gazebo Classic 11 |
| SSMI | `docker-compose.ssmi.yml`（`name: nbv_ssmi`） | `nbv-ssmi:noetic` | `nbv_ssmi` | Ignition Fortress（`ign` 6.17.x） |

- 两栈均 `network_mode: host` + `ipc: host`。
- **完整仿真不要并行**；Gazebo Classic master 默认 `11345`。可用 `GAZEBO_MASTER_URI=http://127.0.0.1:<port>` 隔离 Classic。
- 镜像只装系统依赖；源码与 catkin 产物在挂载目录（已 gitignore）。
- 第三方源码本地克隆、各自保留 `.git`；版本钉在 `third_party/README.md`；用 `scripts/clone_third_party.sh`。

## 3. FKIE（已验证接口）

**Pin**：`fkie-nbv-planner` `91091f8a…`；依赖 `fkie_ddynamic_reconfigure`、`fkie_environmental_measurements`。

- Action：`/robot/nbv_rrt`（`NBVPlanner.cpp` 中 `planning_as_(..., "nbv_rrt", ...)`，节点 ns=`robot`）。
- 订阅：`camera_pose`、`octomap`（launch remap → `octomap_full`）、`robot_bbx`（MBF footprint）。
- 启动：官方 `run_planner.launch` 带 **gdb**；容器内用 `configs/fkie/run_planner_nogdb.launch` 或 `rosrun` + `scripts/fkie_smoke_test.sh`。
- 工作空间：`ws_fkie/src` 必须用**相对**符号链接到 `../../third_party/...`（绝对 `/home/ubuntu/...` 在容器内失效）。
- World：`third_party/fkie-nbv-planner/worlds/warehouse_icra_large.world` 等可被 Classic 加载。
- **官方未开源整机**：`fkie-nbv-planner` 无 URDF/xacro/平台 Gazebo 模型；仅约定话题（octomap / camera_pose / MBF footprint）与 RealSense 插件依赖。
- 仓库依赖模型：`shelves_high`、`big_box3/4`、`Floor_room` → `scripts/fetch_fkie_gazebo_models.sh`；compose 设 `GAZEBO_MODEL_PATH`。
- **易错**：Action SUCCEEDED + 空 `goals` ≠ 有效 NBV；完整闭环还需 RealSense 点云链路与 footprint（官方平台栈未进本仓）。
- **有效 NBV 适配验证（2026-10-22）**：`scripts/fkie_effective_nbv_test.sh` + `configs/fkie/adapt_test/nbv_minimal.world` → `VERIFY_FKIE_NONEMPTY_GOALS`，goals=1。要点：
  1. `LD_LIBRARY_PATH` 含 `/usr/lib/x86_64-linux-gnu/gazebo-11/plugins`（`libDepthCameraPlugin.so`）
  2. 必须 `rosrun gazebo_ros gzserver`（加载 `libgazebo_ros_api_plugin.so`），裸 `gzserver` 相机插件不初始化 ROS
  3. planner 需 `octomap:=octomap_full`
  4. Action boundary 须覆盖相机 ROI（官方约 `[-8.36,-2.36]×[-8.69,-2.69]`），原点 `±5` 方框会判相机在外 → `goals=0`
  5. 证据：`data/metrics/fkie_effective_nbv/RESULT.md`，`logs/fkie_effective_nbv_run6.log`
- **仓库场景（部分）**：`scripts/fkie_warehouse_nbv_demo.sh`；GUI 用 `rosrun gazebo_ros gazebo`（勿拆开 gzserver+gzclient）。证据：`data/metrics/fkie_warehouse_nbv/` → goals=0。

## 4. SSMI（已验证接口）

**Pin**：`SSMI-example` `750122b8…`（注意 GitHub 仓名小写 `example`）；子模块 SSMI `727d815…`。

- 启动：`roslaunch semantic_segmentation_husky seg_husky.launch` → mapping → planning →（可选）`jackal_pd`；或 `ssmi_ws/scripts/launch_ssmi_example.sh` / `scripts/ssmi_smoke_test.sh`。
- 关键话题（已实测有数据）：
  - `/rgbd_camera/{image,depth_image,camera_info}`
  - `/semantic/{colored_map,labels_map}`
  - `/semantic_pcl/semantic_pcl`
  - `/octomap_full`、`/octomap_color`
  - `/planner/path`、`/planner/position_cmd`
- 编译硬依赖：`libgflags-dev`（`ros_ign_gazebo`）。
- `rosdep update` 可能超时：Dockerfile 中非致命即可。
- 无 TTY 时 `teleop_twist_keyboard` 会 termios 失败；可忽略。
- **有效地图/规划验证（2026-10-22）**：`scripts/ssmi_effective_map_plan_test.sh` → `SSMI_EFFECTIVE_OK`。octomap `data_len` 15s 内增长（如 71028→91152）；`/planner/path` 为 `nav_msgs/Path`、非空 poses（实测 10）。路径常一次性发布，需在 planner 启动时即订阅录制。证据：`data/metrics/ssmi_effective/RESULT.md`。

## 5. 常见故障（已定位）

| 症状 | 原因 | 处理 |
|------|------|------|
| catkin「No packages found」 | 宿主机绝对路径 symlink | 相对链接 `setup_fkie_ws.sh` |
| SSMI 镜像 build 失败于 rosdep | 网络超时 | `(rosdep update \|\| true)` |
| `gflags/gflags.h` missing | 未装 libgflags | `libgflags-dev` |
| `pkill -f ign` 自杀 | cmdline 含 `ign` | 用 PID / `rosnode kill` |
| Gazebo `Address already in use` | host 网络残留 gz* | kill 残留或换 `GAZEBO_MASTER_URI` |
| 宿主机删不掉 build | 容器 root 产物 | 在容器内 `rm` 或改属主 |
| `libDepthCameraPlugin.so` not found | ROS 相机插件 dlopen Gazebo 插件 | `LD_LIBRARY_PATH` 加 gazebo-11/plugins |
| Kinect 插件：ROS not initialized | 未加载 gazebo_ros_api | `rosrun gazebo_ros gzserver` |
| FKIE goals=0 且相机在 ROI 外 | Action boundary 与相机位姿不重合 | 用官方 `planner_boundary_*` ROI |
| 仓库 mesh 缺失 / 空场景 | 未拉 PX4 shelves/big_box + Floor_room | `fetch_fkie_gazebo_models.sh` + `GAZEBO_MODEL_PATH` |
| 仓库有货架难见货箱 | world 内嵌 `big_box*.dae` 贴图易失败 | 适配 world `warehouse_icra_large_viz.world` 用实体盒可视化（待验收） |
| `/realsense/depth/image_raw` 为 rgb8 | openni 插件 RGB/depth 同 topic | SDF 中 rgb 与 depth 话题分离 |
| 仓库 NBV goals=0（ROI 内） | frontiers/RRT 扩不开（地图/朝向/自由空间） | 待查；≠完整探索成功 |
| Gazebo/RViz 无机械臂 | 官方未发布平台模型 | 需自备平台或示意模型 |
| 分启 gzserver+gzclient | 双 `/gazebo` 抢 master | `rosrun gazebo_ros gazebo` 一体启动 |
| 多次测试叠多个 gzserver | 未停旧仿真 / cleanup 未杀子进程 | 先 `scripts/fkie_stop_sim.sh`；zombie 则 recreate 容器 |
| SSMI `/planner/path` 订阅不到 | 路径一次性发布、后订错过 | planner 启动即 recorder |

## 8. AGX 适配（Mode A 已验证 + 2026-10-23 几何修复）

- 源码：`third_party/agx`（gitignore）；URDF `rangerboxcr10lidar_description/urdf/rangercr10lidar.urdf`。
- Mode A（默认）：`<static>true</static>`；spawn **`z≈0.42`**（`fr_wheel` STL 半径≈0.153 m；旧 z=0.36 时轮心≈0.10→穿地约 5 cm）；**臂关节须 bake 为 fixed**；静态 `world→base_link`；无 gazebo js/p3d。
- Spawn 配置（勿混用）：`SPAWN_PROFILE=baseline|aisle_safe` → `configs/fkie/agx_adapt/config/spawn_warehouse_*.yaml`；`AGX_SPAWN_{X,Y,Z,YAW}` 可覆盖。baseline XY 与货架 OBB 重叠（仅历史回归）；**aisle_safe** `(-2.10,-5.89,yaw=2.833)` 底盘 OBB 无重叠。
- 仓库 `warehouse_icra_large_viz.world`：**上层货架层板默认无货物**（全图无 z>1.2 的货模型）；仅货架旁地面有 `big_box*`（z≈0.64）。层上物体需另 spawn（primitive/YCB），不是原 world 自带。
- **D455 传感器**：Gazebo depth 沿传感器 **+X** 看。`<pose>` 不得再用 optical 的 `(-π/2,0,-π/2)`（会相对 TF optical **+Z** 偏 90°）。正确：`d455_link` 上 `<pose>0 0 0 0 0 0</pose>` + `frameName=d455_depth_optical_frame`（link +X = optical +Z）。证据：`reports/d455_pointcloud_tf_debug.md`。
- 定位默认 `--world-tf-mode tf`；`T_world_cam_calib.json` 仅历史调试，禁止再当权威外参。
- Mode B（2026-10-24 验收 B1–B4）：`AGX_MODE=dynamic` → 独立 URDF `rangercr10lidar_gazebo_dynamic.urdf`（js+p3d，无 static）；**禁用** Mode A 假 JS/静态 TF；`agx_mode_b_p3d_tf.py` + 运动学 `joint_bridge`/`car_cmd_bridge`（对齐 REMANI fake_mm，非轮式动力学）。入口 `scripts/start_agx_warehouse_mode_b.sh`。`gazebo_ros_control` 曾致 NaN，勿默认启用。
- Mode B + YCB（2026-10-25）：`LOAD_YCB_SCENE=1` 在 `/gazebo/spawn_sdf_model` 就绪后调用 `scripts/load_ycb_scene_into_gazebo.sh`；**默认布局** `data/scenes/indoor_objects/ycb/layouts/seed42.json`（已验收，**禁止启动时 regenerate/覆盖**）；`ONLY_MISSING=1` / `spawn_object_scene.py --only-missing` 可在已跑 Mode B 上补缺失，不删非本布局模型、不重启 AGX/REMANI/Gazebo。`LOAD_YCB_SCENE=0`（默认）保留纯运动 / B4 回归。验收：`verify_gazebo_object_scene.py` fail-closed + `diagnose_ycb_gazebo_visibility.py`；证据 `reports/mode_b_ycb_scene.md`、`reports/mode_b_ycb_shots/`。S5-B：`run_fkie_remani_one_step.sh` 默认 `ENSURE_YCB=1`，要求同世界含 `agx_ranger_cr10`+`mo_table_01`+≥8 `mo_obj_*`。
- REMANI warehouse：`configs/fkie/agx_adapt/launch/remani_agx_warehouse.launch`（无 fake_mm）；`remani_init_aisle_observe.yaml` 覆盖 `fsm/init_state`；EE 目标须对齐 `/ee_current_pose`（相对 TF Link6 约 **-0.42 m Z**，多构型测量≈−base_z）。B4 **`static_empty`=运动管线隔离**；仓库避障用 **`environment_mode=simulated`** + 过滤后的 `/map_generator/global_cloud`。
- 点云 feeder（2026-10-25 C1）：`warehouse_cloud_to_remani.py` 用 **stamp TF** + `robot_self_filter.py` 连杆 capsule/box（禁止 0.9 m 半径抹除）；`z_min≈0.15` 去地面假障碍；体素累积。验收：`validate_self_filter_live.py` / `validate_remani_warehouse_map.py`。启动 simulated：`scripts/restart_remani_simulated.sh`。
- FKIE→REMANI（C3/C4）：`save_fkie_action_result.py` 结构化 Action JSON（禁正则/默认 RPY）；`nbv_to_remani_ee_goal.py --from-fkie-json --align-remani-ee --publish` 只发一次；`execute_single_ee_goal.py --monitor-only`；一键 `run_fkie_remani_one_step.sh`。证据：`reports/fkie_remani_one_step_execution.md`。
- 启动：`scripts/fkie_agx_warehouse_nbv.sh`；停仿：`scripts/fkie_stop_sim.sh`。带物体：`LOAD_YCB_SCENE=1 bash scripts/start_agx_warehouse_mode_b.sh`；仅补物体：`ONLY_MISSING=1 bash scripts/load_ycb_scene_into_gazebo.sh`。
- RViz：单窗口 `agx_nbv.rviz`（`DepthCloud_optical_frame` vs `Octomap_world_fused` 分开展示）。
- Octomap：`configs/fkie/agx_adapt/launch/octomap_agx.launch`（max_range=5，filter_ground=false）。
- 仓库 NBV：**正确光学** + aisle_safe 臂姿/ROI 后已恢复非空：`goals=1`，GOAL≈(-3.020,-5.294,1.609)；证据 `reports/fkie_corrected_optical_nbv.md`。禁止回退 90° 传感器 pose。
- aisle_safe 专用：`arm_view_pose_aisle_safe.yaml`；planner `nbv_planner_agx_warehouse_aisle_safe.yaml`（东界 x=-1.80）；`camera_pose` 发布 `d455_depth_optical_frame`。

## 6. 主动视角 / 运动后端（设计原则，实现多为待验证）

以下为项目方向上的稳定约定，**具体算法实现尚未在本仓落地验证**：

- **对象记忆 / NBV**：规划输入应显式区分几何占用（octomap）与任务相关观测（语义/测量估计）；FKIE 的 `NbvPlanner.action` 已体现 boundary + estimations 分离。
- **Baseline 对比**：FKIE 与 SSMI 保持独立栈，避免为「统一方便」合并 Compose 导致依赖/仿真引擎纠缠。
- **运动后端（REMANI / TopAY）**：应以规划输出位姿/路径话题或 Action 为边界对接；未验证前不要改 FKIE/SSMI 核心消息定义来迁就后端。
- **验证阶梯**：先保证地图与规划接口稳定，再接运动执行；不要跳过话题/Action 层直接宣称闭环。

## 7. 多对象场景 / 感知 / 评测（2026-10-22 起）

| 项 | 内容 |
|----|------|
| 对象资产 | `scripts/fetch_object_models.sh` → `third_party/gazebo_objects/gazebo_ycb`（gitignore）；本地 `configs/fkie/agx_adapt/models/{simple_table,traffic_cone,cardboard_occluder}` |
| 布局/真值 | YCB 桌+地面：`(-3.90,-7.50)`；权威布局 **`ycb/layouts/seed42.json`（勿无条件 regenerate）**；`ycb_model_contact_z.py` 按 collision 底面算 Spawn Z；`validate_object_layout.py` fail-closed；Mode A 一键 `start_fkie_agx_ycb_demo.sh`；Mode B：`LOAD_YCB_SCENE=1` / `load_ycb_scene_into_gazebo.sh` |
| 感知 | `perceive_rgbd_objects.py` / offline `perceive_saved_rgbd_gsam.py`；失败分析 `analyze_gsam_failures.py` |
| Grounded-SAM | NBV vendor + checkpoints；`.conda_envs/nbv_perception`；禁止 DovSG 运行时依赖 |
| 对象记忆 | `perception/object_memory.py`（扩展字段 + confirm）；多视角 `run_multiview_ycb_capture.sh`（`AGX_ARM_POSE` bake） |
| 评测 | `evaluate_object_discovery.py` + `run_object_discovery_benchmark.sh`；正式 **0.15 m**；召回分母=全场景 GT |
| REMANI | `/ee_goal` PoseStamped `world`；FSM 需 odom+js、`target_type!=PRESET`；wbc@`remani-real-implementation`（gitignore）。**已验证**：独立 master `remani_sim` committed；**Warehouse Mode B**：同 master + 动态 AGX + `mm_controller` 执行，单目标 `SINGLE_EE_EXEC_OK`（相机 TF 位移），见 `reports/remani_single_goal_execution.md` |


**Mode A 约束**：static 冻结关节 → 须 URDF bake；连杆合并后用 model×TF 与 gripper tip 校验。相机朝向已与 optical TF 对齐后，物体须落在真实视锥（光轴探针/GT 反投影），禁止用错误 TF 硬塞物体。

**感知独立性硬约束**：允许从 DovSG **复制**；禁止运行时 import DovSG、`PYTHONPATH` 含 DovSG、软链/挂载 DovSG、改 DovSG、复用 `conda dovsg`。移动/删除 DovSG 不应影响 NBV 感知（权重与 vendor 留在本仓）。

## 8. 禁止轻易改动的已验证点

- `.:/workspace` 挂载与相对 symlink 策略  
- 双 Compose 独立、`name: nbv_fkie|nbv_ssmi`  
- Xauth cookie 方案（相对 `xhost +` / privileged）  
- SSMI 使用 Fortress + `ros_gz` 官方示例路径  
- 验证层级区分（编译 / 启动 / 调用 / 闭环 / 实机）  
- Mode A `arm_view_pose.yaml` / warehouse NBV 非空配置（不要为「更优雅」改掉）
