# FKIE 空目标（goals=[]）诊断

Date: 2026-10-25  
Commit: `dde6668` + 本轮脚本增强  
证据：`data/metrics/fkie_empty_diag_135111/fkie_action_result.json`

## 结论

`goals=[]` **不等于** FKIE 失败。本轮实测：

| 字段 | 值 |
|------|-----|
| action_state | SUCCEEDED |
| n_goals | 0 |
| request_base_pose | true |
| complete_exploration | false |
| goal_pose_3d | 有效相机位姿 `(-3.85,-6.50,0.63)` |
| empty_class | **G_BASE_POSE** |
| usable_nbv | **true**（接受 `goal_pose_3d`） |

分类语义（`scripts/save_fkie_action_result.py`）：

- **A_COMPLETE** — 探索完成
- **B_NO_FRONTIER** — 无 frontier / cached nodes
- **C_NO_BRANCH** — RRT 有点但 best_branch 空
- **D_FILTERED** — complete + cached 仍在（高度/增益过滤）
- **E_TF / E_MAP** — 相机 TF 或 OctoMap 不可用
- **G_BASE_POSE** — `goals[]` 空但 `request_base_pose` + 有效 `goal_pose_3d`（本轮主因）
- **H_ERROR** — Action / 字段异常

## 运行时上下文（同次调用）

- frontiers: 20 posed
- cached_nodes: 40 posed
- rrt: 227 posed
- best_branch: 0
- octomap: `world` res=0.1 data_len=537410
- D455: `(-3.20,-6.06,1.67)`

## 高度可行性（本轮补充）

FKIE `goal_pose_3d` 在 `request_base_pose` 路径上常为 yaw-only 采样。  
当 `z≈0.62` 时转换 EE 后 `z≈0.16` → REMANI `IK failed: pose_not_reached`。

`save_fkie_action_result.py` 现要求相机目标高度 ∈ **[0.85, 1.75]** 才标 `usable_nbv`；  
否则 `D_FILTERED` 并拒绝发明替代 EE。配置：`arm_height_min: 0.95`（aisle_safe yaml）。

重启规划器须 remap：`octomap:=octomap_full`（见 `restart_fkie_nbv_planner.sh`），否则 “No octomap received”。

## Fail-closed

- 无 `usable_nbv` 时退出码 5（除非 `--allow-empty`）
- **禁止**把空目标改写成手工 EE
- 在线循环遇到空目标合法终止（exit 10），不无限重试
- 冷启动假 `A_COMPLETE`：允许 **1 次** settle-retry（非无限）

## 命令

```bash
source /opt/ros/noetic/setup.bash
source /workspace/third_party/wbc/remani_planner/devel/setup.bash
source /workspace/ws_fkie/devel/setup.bash
export PYTHONPATH="/workspace/third_party/wbc/remani_planner/devel/lib/python3/dist-packages:${PYTHONPATH}"
python3 /workspace/scripts/save_fkie_action_result.py \
  --out /workspace/data/metrics/fkie_empty_diag_XXXX/fkie_action_result.json \
  --allow-empty
```
