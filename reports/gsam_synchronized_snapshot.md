# F2：同步 RGB-D 快照 + 同帧 Grounded-SAM

Date: 2026-10-25  
Commit base: `dde6668` + 本轮脚本

## 问题

旧流程 `capture_nbv_loop_snapshot.py` 存图后再跑 `perceive_rgbd_objects.py`，后者可能重新订阅话题 → RGB/Depth/TF 不同帧。

## 方案

1. `scripts/capture_synchronized_rgbd_snapshot.py`  
   - `ApproximateTimeSynchronizer`（slop=0.05）同步 RGB/Depth/CameraInfo  
   - 用 **RGB stamp** 查 optical TF（非 `Time(0)`）  
   - 写出 `rgb.png` / `depth.npy` / `camera_info.json` / `camera_pose.json` / `meta.json`（含 `snapshot_id`、`sync_dt_s`）

2. `scripts/perceive_snapshot_gsam.py`  
   - **只读快照目录**，绝不订阅 live 话题  
   - conda `nbv_perception` 跑 Grounding DINO + SAM2  
   - 三维定位使用同快照 depth + K + stamp TF  
   - 失败显式：`INPUT_MISSING` / `MODEL_FAIL` / `ok_empty` / `ok`

## 验证

```text
data/metrics/sync_snap_smoke_135136/
  meta.json          sync_dt_s=0.0  snapshot_id=smoke1
  detection.json     n_det=9 n_localized=9 status=ok
  masks/ perception.json gsam_vis.png
```

命令：

```bash
python3 scripts/capture_synchronized_rgbd_snapshot.py --out-dir ... --snapshot-id smoke1
# conda env:
.conda_envs/nbv_perception/bin/python scripts/perceive_snapshot_gsam.py --snapshot-dir ... --device cuda
```

## 验收

保存的 RGB、检测框、SAM mask、三维中心均可追溯到同一 `snapshot_id`。
