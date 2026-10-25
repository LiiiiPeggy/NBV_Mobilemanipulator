# NBV 到达后 GSAM 触发（P2）

更新：2026-10-25  
证据：`data/metrics/fkie_agx_warehouse_mode_b/gsam_arrival_trigger.json`

## 机制

脚本：`scripts/trigger_gsam_on_nbv_arrival.py`

门禁（全部满足才推理，**不**仅看 REMANI FSM）：

1. 位姿误差 ≤ `pos_tol` / `rot_tol_deg`（相机或 EE，可配置）  
2. TF 有限差分速度 ≤ `vel_tol`  
3. 连续稳定 ≥ `stable_s`  
4. RGB+Depth stamp 新于到达时刻  

然后：系统 Python + `cv_bridge` 存帧 → conda `nbv_perception` 离线 `perceive_saved_rgbd_gsam.py`（避开 conda↔`libgdal`/`cv_bridge` 冲突）。

## 实测

- Gate PASS：`pos_err≈0.0006 m`，`speed≈5e-4`，`stable_s≈0.8`，`fresh_ok`  
- GSAM：`returncode=0`，`elapsed≈6.3 s`，`ndet=1`，`PERCEIVE_SAVED_GSAM_OK`  
- 帧：`gsam_arrival_frames/rgb_arrival.png` + depth  

当前场景未加载 YCB 时检测数可能为 0–1；触发链路本身已验证。
