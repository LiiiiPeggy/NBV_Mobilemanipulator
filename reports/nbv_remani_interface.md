# NBV → REMANI 接口设计（草案，待联调）

状态：**设计级** — 尚未做单目标执行验证。

## 坐标约定

- 世界系：`world`（与 FKIE / Gazebo 一致）
- 相机光学系：`d455_color_optical_frame` / `d455_depth_optical_frame`
- 末端：以 AGX / WBC 中 CR10 法兰或夹爪基座 frame 为准（联调前从 `wbc` 分支核对真实名称）

## 位姿转换

给定 NBV 相机目标 `T_world_camera`：

```
T_world_ee = T_world_camera * inv(T_ee_camera)
```

`T_ee_camera` 来自 URDF 固定外参（`d455_joint` 链），**不可**只平移忽略姿态。

## 候选可达性流水线

1. NBV / UAV 适配器输出相机候选 `(x,y,z,rpy)`  
2. 高度裁剪到 AGX 实测可达带（由关节限位采样，**禁止写死未验证 1.8m**）  
3. 转为末端目标 → REMANI 规划  
4. 失败则邻近搜索 / 下一候选；记录原始目标与适配偏差  

## 公平对比约束（未来）

同一机器人、D455、场景、感知前端、运动后端与时间预算。
