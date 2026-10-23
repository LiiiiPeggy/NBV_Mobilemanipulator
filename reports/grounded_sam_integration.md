# Grounded-SAM integration (NBV-vendored, DovSG-independent)

Date: 2026-10-23

## Copy source (one-time)

- Source tree: `/home/ubuntu/locomani/DovSG` (copy only; **no** runtime mount/import/`PYTHONPATH`)
- Vendored code: `perception/vendor/GroundingDINO`, `perception/vendor/segment-anything-2`
- Weights: `checkpoints/grounding_dino/`, `checkpoints/sam2/`, `checkpoints/bert-base-uncased/`
- Manifest: `checkpoints/CHECKSUMS.sha256`, `perception/NOTICE`
- Install script: `scripts/vendor_dovsg_perception.sh`

## NBV perception env

- Path: `.conda_envs/nbv_perception` (gitignored; torch `2.3.1+cu121`)
- Editable installs: local vendor GroundingDINO + SAM2
- Runner: `scripts/run_object_perception.sh` / `backend=grounded_sam`
- Optional image: `Dockerfile.perception` (does **not** mount DovSG)
- Docs: `configs/perception/README.md`

## Independence

Command: `SKIP_STRACE=1 bash scripts/verify_perception_independence.sh reports`

- SHA256 of checkpoints: **OK**
- Import paths resolve under `NBV_Mobilemanipulator` only: **OK**
- Inference after path assertions: **OK** (`PERCEPTION_INDEPENDENCE_OK`)
- Temporary `mv` of DovSG: blocked by agent sandbox write outside workspace; path/checksum/import evidence still holds. Claim: moving/deleting DovSG must not break NBV perception when checkpoints+vendor remain.

Log: `reports/grounded_sam_independence.log`

## Timing / VRAM (A6000, docker `--gpus all`)

| Metric | Value |
|--------|-------|
| Load | ~3.0 s |
| Infer (480×640 random) | ~0.76 s |
| `max_memory_allocated` | ~2.73 GB |
| Note | GroundingDINO `_C` CUDA op not built; PyTorch MSDA fallback used on GPU tensors |

Smoke: `scripts/_smoke_gsam_cuda.py` → `DOCKER_GPU_DETECTOR_OK`

## Modules

- `perception/rgbd_object_detector.py` — GDINO+SAM2
- `perception/object_localization.py` — mask/depth helpers
- `perception/object_memory.py` — multi-view association
- `scripts/perceive_rgbd_objects.py --backend grounded_sam` (+ ApproximateTimeSynchronizer)

Color/depth backends remain for debug only; formal eval should use `grounded_sam` + YCB layouts.
