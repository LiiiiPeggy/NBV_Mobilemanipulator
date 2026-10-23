# FKIE NBV under corrected D455 optical geometry (P1-R)

Date: 2026-10-23

## Setup

- `SPAWN_PROFILE=aisle_safe` → `(-2.10,-5.89,0.42,yaw=2.833)`
- Arm: `configs/fkie/agx_adapt/config/arm_view_pose_aisle_safe.yaml` (Mode A bake)
- `camera_pose` frame: `d455_depth_optical_frame` (ROS optical +Z; **no** 90° sensor-pose regression)
- Planner ROI: `nbv_planner_agx_warehouse_aisle_safe.yaml` (east boundary x=-1.80, visited_cell=120)
- Octomap settle: 35s

## Diagnosis (`scripts/diagnose_fkie_nbv_empty.py`)

From `data/metrics/fkie_agx_warehouse_nbv/nbv_empty_diag_before.json`:

- Optical TF in boundary: **true**
- Octomap `data_len≈29095`, res=0.1
- Frontiers topic timeout (non-fatal for this run)

## Action result (verified)

```
result: complete_exploration=False request_base_pose=False goals=1 explored_boundary_id=0
GOAL[0] frame=world xyz=(-3.020, -5.294, 1.609)
VERIFY_FKIE_NONEMPTY_GOALS
```

## Verdict

| Claim | Status |
|-------|--------|
| Correct optical geometry | verified |
| Non-empty FKIE goals under aisle_safe | **verified** (`goals=1`) |
| Complete exploration closed-loop motion | not claimed (Mode A) |

Do **not** reintroduce identity/90° sensor pose hacks to “force” goals.
