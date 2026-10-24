# FKIE → REMANI → AGX closed loop

Date: 2026-10-24  
Status: **B1–B4 passed**; **S5-A tooling ready**; **S5-B full FKIE Action→exec one-shot not completed in this session** (honest gate).

## Passed gates

| Gate | Status | Evidence |
|------|--------|----------|
| B1 Mode B dynamic | PASS | `reports/agx_mode_b_physics.md`, `stability_b1.json` |
| B2 REMANI←AGX state | PASS | `reports/remani_agx_state_alignment.md` |
| B3 warehouse map | PARTIAL | cloud feeder + probe JSON; start-collision filter TBD |
| B4 single EE exec | PASS | `reports/remani_single_goal_execution.md`, camera moved |
| S5-A goal semantics | PASS (tooling) | `nbv_to_remani_ee_goal.py --from-fkie-json --align-remani-ee` |
| S5-B FKIE Action closed loop | NOT RUN end-to-end | use `scripts/run_fkie_remani_one_step.sh` when Mode B+REMANI up |

## What counts as success (this session)

Real Gazebo AGX motion from REMANI trajectory + D455 TF change — **not** merely `/ee_goal` subscription or RViz path.

## Mode A

Unchanged path: `AGX_MODE=static` (default) still uses baked URDF + static TF.
