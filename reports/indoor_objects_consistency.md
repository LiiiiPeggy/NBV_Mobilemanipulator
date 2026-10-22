# 室内多对象场景一致性

## 场景分离

| scene_kind | 路径 | 说明 |
|------------|------|------|
| `primitive_debug` | `data/scenes/indoor_objects/primitive_debug/{layouts,gt}/` | 彩色球调试；**不是** YCB 识别结果 |
| `ycb` | `data/scenes/indoor_objects/ycb/{layouts,gt}/` | `generate_object_scene.py` 默认输出 |

- 遗留 `data/scenes/indoor_objects/layouts/seed42.json` 勿再作为默认写入目标。  
- `run_indoor_objects_min_loop.sh`：`SCENE_KIND=primitive_debug|ycb`，`DIST_THRESH` 默认 **0.15**，报告同时附带 0.10/0.35 诊断。  
- `check_indoor_scene_consistency.py` 检查混用。

## 评测（相机位姿修复后，aisle_safe 光轴球）

权威几何指标见 `gt_depth_tf_localization`（median≈0.069 m）。  

颜色启发式 backend（≠ Grounded-SAM）：

| dist_thresh | tp | precision | unique_recall | mean_err |
|-------------|----|-----------|---------------|----------|
| 0.10 / 0.15 / 0.35 | 4 | 0.2 | 0.4 | ≈0.052 m |

正式阈值冻结前不得为抬召回事后放宽；0.35 仅早期调试参考。

## FKIE（几何修复后）

| Profile | spawn | NBV 结果 |
|---------|-------|----------|
| aisle_safe（修相机后） | (-2.1,-5.89,yaw=2.833) | Action OK，`goals=0`，`complete_exploration=True`（视锥/地图变化；**不**伪造非空） |
| aisle_safe（修相机前） | 同上 | 曾 `goals=1` GOAL≈(-3.03,-5.38,1.61) — 旧 90° 朝向，不可作几何真理 |
| baseline | (-5.36,-5.685,yaw=0) | 见 `logs/.../console_baseline.log`（回归保留重叠 spawn） |
