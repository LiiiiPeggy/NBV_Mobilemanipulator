#!/usr/bin/env python3
"""Grounding DINO + SAM2 detector using NBV-vendored packages only."""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np
import torch
import torchvision

from perception.paths import (
    assert_local_checkpoints,
    grounding_dino_checkpoint,
    grounding_dino_config,
    sam2_checkpoint,
    sam2_model_cfg,
)


@dataclass
class DetectionResult:
    xyxy: np.ndarray
    confidence: np.ndarray
    class_id: np.ndarray
    masks: Optional[np.ndarray]
    classes: List[str]


class GroundedSAMDetector:
    def __init__(
        self,
        box_threshold: float = 0.25,
        text_threshold: float = 0.25,
        nms_threshold: float = 0.5,
        device: Optional[str] = None,
    ):
        assert_local_checkpoints()
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold
        self.nms_threshold = nms_threshold
        if device:
            self.device = device
        else:
            try:
                self.device = "cuda" if torch.cuda.is_available() else "cpu"
            except Exception:
                self.device = "cpu"

        from perception.paths import nbv_root

        root = str(nbv_root())
        bert_path = os.path.join(root, "checkpoints/bert-base-uncased")
        if os.path.isdir(bert_path):
            os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
            os.environ.setdefault("HF_HOME", os.path.join(root, "checkpoints"))
            # GroundingDINO config references bert-base-uncased; force local dir via cwd-relative path.
            os.environ.setdefault("BERT_PATH", bert_path)

        from groundingdino.util.inference import Model as GDModel
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        try:
            self.grounding_dino_model = GDModel(
                model_config_path=str(grounding_dino_config()),
                model_checkpoint_path=str(grounding_dino_checkpoint()),
                device=self.device,
            )
            sam2_model = build_sam2(sam2_model_cfg(), str(sam2_checkpoint()), device=self.device)
        except Exception as e:
            if self.device != "cpu":
                self.device = "cpu"
                self.grounding_dino_model = GDModel(
                    model_config_path=str(grounding_dino_config()),
                    model_checkpoint_path=str(grounding_dino_checkpoint()),
                    device=self.device,
                )
                sam2_model = build_sam2(sam2_model_cfg(), str(sam2_checkpoint()), device=self.device)
            else:
                raise
        self.sam2_predictor = SAM2ImagePredictor(sam2_model)

    def run(self, image_rgb: np.ndarray, classes: List[str]) -> DetectionResult:
        detections = self.grounding_dino_model.predict_with_classes(
            image=cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR),
            classes=classes,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
        )
        masks = None
        if detections.class_id is not None and len(detections.class_id) > 0:
            nms_idx = (
                torchvision.ops.nms(
                    torch.from_numpy(detections.xyxy),
                    torch.from_numpy(detections.confidence),
                    self.nms_threshold,
                )
                .numpy()
                .tolist()
            )
            detections.xyxy = detections.xyxy[nms_idx]
            detections.confidence = detections.confidence[nms_idx]
            detections.class_id = detections.class_id[nms_idx]
            valid_idx = [i for i, val in enumerate(detections.class_id) if val is not None and val != -1]
            detections.xyxy = detections.xyxy[valid_idx]
            detections.confidence = detections.confidence[valid_idx]
            detections.class_id = detections.class_id[valid_idx]
            if len(detections.class_id) > 0:
                masks = self._sam2_masks(image_rgb, detections.xyxy)
        return DetectionResult(
            xyxy=detections.xyxy if detections.xyxy is not None else np.zeros((0, 4)),
            confidence=detections.confidence if detections.confidence is not None else np.zeros((0,)),
            class_id=detections.class_id if detections.class_id is not None else np.zeros((0,), dtype=int),
            masks=masks,
            classes=list(classes),
        )

    def _sam2_masks(self, image_rgb: np.ndarray, xyxy: np.ndarray) -> np.ndarray:
        self.sam2_predictor.set_image(image_rgb)
        result = []
        for box in xyxy:
            masks, scores, _ = self.sam2_predictor.predict(box=box, multimask_output=True)
            result.append(masks[int(np.argmax(scores))].astype(bool))
        return np.asarray(result)
