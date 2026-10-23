# Mode B + REMANI `/ee_goal` probe (P4)

Date: 2026-10-23

## Mode B bringup

- Entry: `AGX_MODE=dynamic SPAWN_PROFILE=aisle_safe ./scripts/fkie_agx_warehouse_nbv.sh`
- Probe helper: `scripts/run_mode_b_remani_probe.sh`
- This session kept the already-running **Mode A** `nbv_fkie` stack for FKIE/perception continuity; full Mode B physics restart was **not** completed here (would interrupt verified aisle_safe NBV).

## Single-goal bridge (verified on live FKIE)

FKIE goal used: `(-3.020, -5.294, 1.609)` from P1-R.

```
python3 scripts/nbv_to_remani_ee_goal.py \
  --cam-xyz -3.020 -5.294 1.609 --cam-rpy 0 0.35 0 \
  --camera-frame d455_depth_optical_frame --publish \
  --out data/metrics/nbv_remani/ee_goal_mode_b_probe.json
```

Result:

- `ee_goal_xyz ≈ (-2.980, -5.219, 1.551)`
- Published `/ee_goal` (`NBV_TO_REMANI_EE_GOAL_OK`)
- `rostopic info /ee_goal`: **no REMANI subscriber** (only transient `rostopic` clients)

JSON: `data/metrics/nbv_remani/ee_goal_mode_b_probe.json`

## Verdict

| Item | Status |
|------|--------|
| FKIE→`/ee_goal` conversion + publish | verified |
| REMANI planner execution of `/ee_goal` | **not verified** (no subscriber / wbc node not launched) |
| Mode A retained as default | yes |

Next: launch `third_party/wbc` REMANI sim accepting `/ee_goal`, then re-run probe under `AGX_MODE=dynamic`.
