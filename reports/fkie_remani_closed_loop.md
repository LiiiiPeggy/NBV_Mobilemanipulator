# FKIE → REMANI → perception closed loop

Date: 2026-10-23  
Status: **BLOCKED** (honest gate)

## Prerequisite

Stage 4 showed REMANI **sim** accepts `/ee_goal` (`[EE GOAL] committed`). Warehouse Mode B (dynamic AGX in FKIE Gazebo driven by REMANI traj) is **not** integrated.

## What was not claimed

- No FKIE Action goal executed by REMANI in the warehouse world.
- No physical D455 pose change from REMANI trajectory inside `nbv_fkie` Gazebo.
- No post-motion Grounded-SAM / memory update on a new FKIE viewpoint.

## Required before closing Stage 5

1. Single dynamic AGX Mode B stack sharing one ROS master with REMANI (or documented bridge).
2. One reachable FKIE camera goal → `/ee_goal` → traj → Gazebo motion → new RGB-D.
3. Perception+memory delta metrics recorded.

Until then, use Mode A multiview bake (`run_multiview_ycb_capture.sh`) for offline multi-view perception only — not autonomous closed-loop exploration.
