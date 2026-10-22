# D455 点云 / TF 调试

时间：2026-10-23

## 点云来源

`Gazebo D455 depth` → `/realsense/depth/points` → relay → `/realsense/depth/points2`  
- 类型：`sensor_msgs/PointCloud2`  
- `header.frame_id`：`d455_depth_optical_frame`  
- 深度图：`32FC1`，640×480  

RViz：`DepthCloud_optical_frame`（单帧点云）与 `Octomap_world_fused`（融合地图）分开显示。

## 点云“倾斜”根因（已证实）

**不是**单纯俯视导致的正常倾斜，而是 **Gazebo 传感器朝向与 ROS optical TF 差 90°**：

1. Gazebo depth camera 沿传感器坐标系 **+X** 看。  
2. 旧配置把 `<pose>` 设为 optical 的 `rpy(-π/2,0,-π/2)`，使传感器系 = optical 系，于是 Gazebo 沿 optical **+X（右）** 看，而 TF 认为前向是 optical **+Z**。  
3. 探针实验（修正前）：在 TF optical + 1.5·Z 处放绿球 → RGB 几乎不变、中心深度不变、绿像素=0。  
4. SDF 合成位姿与 TF 位置重合，但 Gazebo +X 与 TF optical Z **夹角 90.00°**。

## 修复

`generate_gazebo_urdf.py` 中传感器改为：

```xml
<pose>0 0 0 0 0 0</pose>  <!-- on d455_link; link +X == optical +Z -->
<frameName>d455_depth_optical_frame</frameName>
```

修正后探针：绿像素≈10227，质心≈(320,240)，中心深度≈1.25 m（半径 0.25 → 表面距离正确）。

## TF vs Gazebo（Mode A）

- Arm 连杆在 static+fixed 下合并进 `base_link`；`d455_link` 无独立 `get_link_state`。  
- 权威链：`T_world_optical = T_gazebo_model × T_base_optical(TF/URDF)`。  
- `diagnose_d455_tf.py`：`match_ok=true`，gripper tip TF↔Gazebo 误差 ~3e-6 m。  
- 经验标定 `T_world_cam_calib.json` **仅作历史证据**；默认感知 `--world-tf-mode tf`。

## 几何定位（独立于颜色检测）

光轴验证球 `seed42_aisle_safe`（`primitive_debug`）：

| 指标 | 值 |
|------|-----|
| n_ok / n_gt | 10/10 |
| median_err | **0.069 m** |
| within 0.10 m | 9/10 |
| within 0.15 m | 9/10 |

证据：`data/metrics/agx_debug/gt_depth_tf_localization.json`，截图 `reports/agx_geometry_shots/d455_gt_depth_tf_errors.png`。

## RGB-D 同步

`check_rgbd_sync.py`：分辨率一致；`max_dt_rgb_depth≈0.10 s`（10 Hz 量级，略超 0.05 阈值 → WARN）。OpenNI 同传感器发布，正式流水线建议 `message_filters` 近似同步。
