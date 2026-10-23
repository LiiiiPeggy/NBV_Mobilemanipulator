#!/usr/bin/env python3
"""Mask + depth localization into a target frame via TF."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class LocalizedObject:
    label: str
    confidence: float
    xyz: Tuple[float, float, float]
    pixel_count: int
    bbox_xyxy: Tuple[float, float, float, float]


def depth_to_points(
    depth: np.ndarray,
    mask: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_scale: float = 1.0,
    min_depth: float = 0.05,
    max_depth: float = 8.0,
) -> np.ndarray:
    """Back-project masked depth pixels to camera-frame 3D points."""
    ys, xs = np.where(mask.astype(bool))
    if ys.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    z = depth[ys, xs].astype(np.float64) * float(depth_scale)
    valid = np.isfinite(z) & (z > min_depth) & (z < max_depth)
    xs, ys, z = xs[valid], ys[valid], z[valid]
    if z.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    x = (xs.astype(np.float64) - cx) * z / fx
    y = (ys.astype(np.float64) - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply 4x4 transform to Nx3 points."""
    if points.size == 0:
        return points
    homog = np.concatenate([points, np.ones((points.shape[0], 1))], axis=1)
    out = (T @ homog.T).T[:, :3]
    return out


def localize_detections(
    detections,
    depth: np.ndarray,
    K: Sequence[float],
    T_target_cam: np.ndarray,
    depth_scale: float = 1.0,
    min_pixels: int = 30,
) -> list:
    """Localize each detection mask into target frame (median XYZ)."""
    fx, fy, cx, cy = float(K[0]), float(K[4]) if len(K) >= 5 else float(K[1]), float(K[2]), float(K[5]) if len(K) > 5 else float(K[3])
    # Accept either 3x3 flattened or (fx,fy,cx,cy)
    if len(K) == 4:
        fx, fy, cx, cy = map(float, K)
    elif len(K) >= 9:
        fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])

    results = []
    masks = detections.masks
    if masks is None or len(detections.class_id) == 0:
        return results
    for i, cid in enumerate(detections.class_id):
        label = detections.classes[int(cid)] if 0 <= int(cid) < len(detections.classes) else f"cls_{cid}"
        conf = float(detections.confidence[i]) if i < len(detections.confidence) else 0.0
        mask = masks[i]
        pts_cam = depth_to_points(depth, mask, fx, fy, cx, cy, depth_scale=depth_scale)
        if pts_cam.shape[0] < min_pixels:
            continue
        pts = transform_points(pts_cam, T_target_cam)
        xyz = tuple(float(x) for x in np.median(pts, axis=0))
        box = detections.xyxy[i]
        results.append(
            LocalizedObject(
                label=label,
                confidence=conf,
                xyz=xyz,
                pixel_count=int(pts_cam.shape[0]),
                bbox_xyxy=tuple(float(v) for v in box),
            )
        )
    return results


def estimate_sync_dt(stamp_a, stamp_b) -> Optional[float]:
    """Absolute time delta in seconds between two ROS Time-like stamps."""
    try:
        ta = stamp_a.to_sec() if hasattr(stamp_a, "to_sec") else float(stamp_a)
        tb = stamp_b.to_sec() if hasattr(stamp_b, "to_sec") else float(stamp_b)
        return abs(ta - tb)
    except Exception:
        return None
