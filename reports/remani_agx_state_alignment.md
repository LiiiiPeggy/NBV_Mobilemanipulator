# REMANI ↔ AGX state alignment (B2)

## Verified

`scripts/diagnose_remani_agx_state.py` → `REMANI_AGX_STATE_OK`

- `/mm/car/odom` fake_mm encoding matches Gazebo base xy/yaw (err ≪ 5 cm / 0.1 rad)
- `/mm/mani/joint_state` matches Gazebo CR10 joints
- Base nudge via `/mm_controller_node/car_cmd` moved AGX ~0.15 m (`BASE_NUDGE_OK`)

## FK frame offset (important)

`/ee_current_pose` (REMANI FK) vs TF `cr10_Link6` differs by **~0.42 m in Z** (= spawn base height). REMANI odom uses x,y,yaw only; mount height assumes ground z≈0.

**Rule:** Mode B EE goals must be expressed in REMANI EE frame (`/ee_current_pose` or `--align-remani-ee`).

## mm_controller init

Default `exp_ranger_cr10_param.yaml` `fsm/init_state` forces home joints on AUTO_STAY→CMD_CTRL. Override: `configs/fkie/agx_adapt/config/remani_init_aisle_observe.yaml` loaded by `remani_agx_warehouse.launch`.
