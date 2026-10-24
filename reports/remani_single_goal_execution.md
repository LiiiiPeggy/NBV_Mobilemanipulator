# Single EE goal execution (B4)

## Command path

`/ee_goal` → `remani_planner_node` → `/planning/trajectory` → `mm_controller_node` → bridges → Gazebo AGX

## Evidence

`data/metrics/fkie_agx_warehouse_mode_b/single_ee_exec.json` → **`SINGLE_EE_EXEC_OK`**

| Stage | Result |
|-------|--------|
| goal_published | true |
| traj_received | true |
| car_cmd_seen | true |
| joint_cmd_seen | true |
| camera_moved | true (~7.6 cm optical TF delta) |
| FSM | WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ → WAIT_TARGET |

Log: `[EE GOAL] committed EE_POSE` then `EXEC_TRAJ` in `logs/fkie_agx_warehouse_mode_b/remani_empty.log`.

Env: `environment_mode=static_empty`, init_state observe override, Mode B kinematic bridges.
