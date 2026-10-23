# NBV perception env (Grounded-SAM)

Independent of DovSG at **runtime**. Weights/code are vendored under this repo
(`perception/vendor/`, `checkpoints/`). See `perception/NOTICE` and
`checkpoints/CHECKSUMS.sha256`.

## Create / refresh env

Preferred location (inside repo, gitignored):

```bash
# One-time: clone a CUDA torch env that already has deps (torch 2.3.1+cu121)
rsync -a /path/to/compatible_env/ .conda_envs/nbv_perception/
.conda_envs/nbv_perception/bin/python -m pip install -e perception/vendor/GroundingDINO --no-build-isolation --no-deps
.conda_envs/nbv_perception/bin/python -m pip install -e perception/vendor/segment-anything-2 --no-build-isolation --no-deps
```

Or build `Dockerfile.perception` (do **not** mount DovSG).

**Do not** activate `conda activate dovsg` for NBV perception.

## Run

```bash
export NBV_ROOT=$PWD NBV_PERCEPTION_ROOT=$PWD PYTHONPATH=$PWD
./scripts/run_object_perception.sh --out data/metrics/indoor_objects/perc_gsam.json --views 1
./scripts/verify_perception_independence.sh reports
```

FKIE Noetic (`nbv_fkie`) stays free of torch; subscribe RGB-D over host network.
