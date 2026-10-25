# C3: FKIE goal semantics → REMANI EE

Date: 2026-10-25 07:52:23 UTC

## Structured Action
`scripts/save_fkie_action_result.py` (no regex).

- n_goals: **1**
- camera_goal_xyz: `[-3.537866163716675, -6.302776979242036, 1.2795437704709323]`
- camera_goal_quat_xyzw: `[0.0, 0.0, -0.35836794954530027, 0.9335804264972017]`
- orientation_source: **fkie_action_quaternion**

Empty goals fail closed. Converter refuses silent default RPY.

## Conversion + align
- orientation_source: `goals[0].pose`
- ee_goal_xyz: `[-3.446524663783247, -6.287896145856738, 0.8142294364831522]`
- align: `{'applied': True, 'delta_xyz': [-0.0012253365866188304, -0.0020640623290217874, -0.424212129428682]}`

## EE vs TF Link6
Multi-sample mean Δz ≈ **-0.4240 m**, base_z ≈ **0.4230 m**, `matches_base_z=True`.
Offset tracks missing chassis height in REMANI FK — measured, not guessed.

## Publish discipline
Single `/ee_goal` from converter; executor `--monitor-only`.
