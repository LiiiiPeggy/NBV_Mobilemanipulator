"""Resolve checkpoint paths under NBV_Mobilemanipulator only."""
from __future__ import annotations
import os
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]


def nbv_root() -> Path:
    env = os.environ.get("NBV_PERCEPTION_ROOT") or os.environ.get("NBV_ROOT")
    return Path(env).resolve() if env else _ROOT


def grounding_dino_config() -> Path:
    return nbv_root() / "checkpoints/grounding_dino/GroundingDINO_SwinT_OGC.py"


def grounding_dino_checkpoint() -> Path:
    return nbv_root() / "checkpoints/grounding_dino/groundingdino_swint_ogc.pth"


def sam2_checkpoint() -> Path:
    return nbv_root() / "checkpoints/sam2/sam2_hiera_large.pt"


def sam2_model_cfg() -> str:
    # Hydra config name used by sam2.build_sam when package sam2_configs is installed.
    return "sam2_hiera_l.yaml"


def assert_local_checkpoints() -> None:
    for p in (grounding_dino_config(), grounding_dino_checkpoint(), sam2_checkpoint()):
        if not p.is_file():
            raise FileNotFoundError("Missing NBV-local checkpoint/config: %s" % p)
    # Hard fail if someone points at DovSG
    for p in (grounding_dino_checkpoint(), sam2_checkpoint()):
        resolved = str(p.resolve())
        if "/DovSG/" in resolved or resolved.startswith("/home/ubuntu/locomani/DovSG"):
            raise RuntimeError("Checkpoint path resolves under DovSG (forbidden): %s" % resolved)
