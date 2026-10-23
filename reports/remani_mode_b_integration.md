# REMANI Mode B integration status

Date: 2026-10-23  
Commit tip before this report: `7c35686`

## Verified: `/ee_goal` contract (REMANI sim)

Built in `nbv_fkie`:

```bash
# deps: ros-noetic-ompl, libompl-dev, libnlopt-dev
cd /workspace/third_party/wbc/remani_planner
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Launched on **alternate** ROS master (avoid FKIE Gazebo conflict):

```bash
export ROS_MASTER_URI=http://127.0.0.1:11312 ROS_HOSTNAME=127.0.0.1
export ROS_PACKAGE_PATH=/workspace/third_party/agx/rangerboxcr10lidar_description:$ROS_PACKAGE_PATH
source devel/setup.bash
roscore -p 11312 &
roslaunch remani_planner remani_sim.launch robot_model:=ranger_cr10 target_type:=1 use_rviz:=false
```

Evidence:

- `rostopic info /ee_goal` → subscriber **`/remani_planner_node`**
- Published `PoseStamped` `frame_id=world` → log: **`[EE GOAL] committed EE_POSE`** then FSM `WAIT_TARGET → GEN_NEW_TRAJ`
- Logs: `reports/remani_sim_launch.log`

This proves NBV’s `nbv_to_remani_ee_goal.py` message type/frame matches REMANI FSM expectations.

## Not verified: warehouse Mode B physics

| Item | Status |
|------|--------|
| FKIE warehouse + AGX `AGX_MODE=dynamic` whole-body exec | **blocked** |
| REMANI `fake_mm` vs FKIE Gazebo world alignment | separate sims; not fused |
| D455 motion from REMANI traj in warehouse | not run |

Reasons: REMANI `remani_sim.launch` uses its own `fake_mm` + map_generator; co-running with FKIE Gazebo on one master risks TF/`/joint_states` fights. Full Mode B needs dynamic URDF + Gazebo js bridge + ESDF from warehouse — out of scope for a safe same-day close without risking Mode A regression.

## Mode A retained

Default AGX bringup remains **static** Mode A for perception/NBV.

## Next (Stage 5)

When warehouse Mode B exists: FKIE goal → `nbv_to_remani_ee_goal.py --publish` on the master where `remani_planner_node` subscribes → verify traj + camera move. Until then Stage 5 is **blocked** on Mode B warehouse fusion.
