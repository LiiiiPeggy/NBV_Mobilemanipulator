# FKIE → REMANI one-step execution

Date: 2026-10-25 07:52:23 UTC
goal_id: `fkie_20261025_074842`

## Result: ONE_STEP_OK

| Item | Value |
|------|-------|
| FKIE camera goal | `[-3.537866163716675, -6.302776979242036, 1.2795437704709323]` + quat |
| REMANI EE goal | `[-3.446524663783247, -6.287896145856738, 0.8142294364831522]` |
| ee_err | 0.0020 m |
| cam_err (vs FKIE cam) | 0.0022 m |
| cam_delta (monitor) | 1.142 m |
| FSM | WAIT_TARGET → GEN_NEW_TRAJ → **EXEC_TRAJ** → WAIT_TARGET |

Artifacts under `data/metrics/fkie_agx_warehouse_mode_b/` and `reports/fkie_remani_one_step_shots/`.

Limits: kinematic Mode B bridges; filtered depth map (partial FOV).
