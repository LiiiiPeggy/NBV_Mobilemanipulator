# AGX Mode B physics (B1)

Date: 2026-10-24  
Branch: `cursor/fkie-ssmi-effective-validation`

## Approach

- Separate URDF: `configs/fkie/agx_adapt/urdf/rangercr10lidar_gazebo_dynamic.urdf` (does not overwrite Mode A static URDF).
- Plugins: `gazebo_ros_joint_state_publisher` + `p3d` + D455. **No** `<static>true`.
- Mode A fake `/joint_states` + static `world→base_link` **disabled** in `AGX_MODE=dynamic`.
- Authority: `agx_mode_b_p3d_tf.py` (TF + `/mm/car/odom`), `agx_mode_b_joint_bridge.py` (kinematic `set_model_configuration` hold/track).

## Control honesty

`gazebo_ros_control` PositionJointInterface was tried; PID preload issues and NaN physics on this URDF. Mode B therefore uses **kinematic** joint/base bridges matching REMANI `fake_mm` semantics — not Ackermann wheel dynamics, not model teleport as a fake “success”.

## B1 evidence

`data/metrics/fkie_agx_warehouse_mode_b/stability_b1.json` → `AGX_MODE_B_STABILITY_OK`

- upright, TF↔Gazebo xy err ~1e-4 m
- arm hold drift ~1e-5 rad
- joint5 +0.15 rad via `/mm_controller_node/joint_cmd` tracked

Entry: `scripts/start_agx_warehouse_mode_b.sh` / `AGX_MODE=dynamic` warehouse script.
