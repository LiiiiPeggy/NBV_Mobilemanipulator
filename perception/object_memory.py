#!/usr/bin/env python3
"""Multi-view object memory with greedy spatial association (online IDs only)."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Any
import json
import time
from pathlib import Path

import numpy as np


@dataclass
class MemoryObject:
    object_id: str
    class_id: str
    class_name: str
    class_evidence: List[Dict[str, Any]] = field(default_factory=list)
    position_world: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    position_uncertainty: float = 0.25
    observation_count: int = 1
    last_observed_time: float = 0.0
    last_camera_pose: Optional[List[float]] = None
    observation_quality: float = 0.0
    confirmation_status: str = "tentative"  # tentative | confirmed | conflict
    association_conflict: bool = False
    view_ids: List[int] = field(default_factory=list)
    # backward-compat aliases used by older callers
    center_world: Optional[List[float]] = None
    confidence: float = 0.0
    n_observations: int = 1

    def __post_init__(self):
        if self.center_world is None:
            self.center_world = list(self.position_world)
        else:
            self.position_world = list(self.center_world)
        self.n_observations = int(self.observation_count)
        if self.confidence and not self.observation_quality:
            self.observation_quality = float(self.confidence)


class ObjectMemory:
    def __init__(self, assoc_thresh_m: float = 0.25, confirm_min_obs: int = 2):
        self.assoc_thresh_m = float(assoc_thresh_m)
        self.confirm_min_obs = int(confirm_min_obs)
        self.objects: Dict[str, MemoryObject] = {}
        self._next_id = 0

    def _new_id(self, class_name: str) -> str:
        oid = "mem_%s_%03d" % (class_name, self._next_id)
        self._next_id += 1
        return oid

    def upsert(
        self,
        class_name: str,
        center_world,
        confidence: float = 0.0,
        view_id: int = 0,
        camera_pose=None,
        stamp: Optional[float] = None,
        bbox_diag_m: Optional[float] = None,
    ) -> str:
        c = np.asarray(center_world, dtype=np.float64).reshape(3)
        stamp = float(stamp if stamp is not None else time.time())
        best_id, best_d = None, 1e9
        conflict = False
        near = []
        for oid, obj in self.objects.items():
            d = float(np.linalg.norm(c - np.asarray(obj.position_world)))
            if d <= self.assoc_thresh_m * 1.5:
                near.append((d, oid, obj.class_name))
            if obj.class_name != class_name:
                continue
            if d < best_d:
                best_d, best_id = d, oid
        # Ambiguous: multiple different classes within thresh → mark conflict, do not merge
        same_pos_diff_class = [x for x in near if x[0] <= self.assoc_thresh_m and x[2] != class_name]
        if same_pos_diff_class and (best_id is None or best_d > self.assoc_thresh_m):
            conflict = True

        if best_id is not None and best_d <= self.assoc_thresh_m and not conflict:
            obj = self.objects[best_id]
            n = obj.observation_count
            prev = np.asarray(obj.position_world, dtype=np.float64)
            fused = (prev * n + c) / float(n + 1)
            obj.position_world = fused.tolist()
            obj.center_world = obj.position_world
            obj.confidence = max(obj.confidence, float(confidence))
            obj.observation_quality = max(obj.observation_quality, float(confidence))
            obj.observation_count = n + 1
            obj.n_observations = obj.observation_count
            obj.last_observed_time = stamp
            if camera_pose is not None:
                obj.last_camera_pose = list(camera_pose)
            obj.class_evidence.append({"view_id": int(view_id), "class": class_name, "conf": float(confidence)})
            if view_id not in obj.view_ids:
                obj.view_ids.append(int(view_id))
            # uncertainty shrinks slowly with observations
            obj.position_uncertainty = float(max(0.05, obj.position_uncertainty / np.sqrt(obj.observation_count)))
            if obj.observation_count >= self.confirm_min_obs:
                obj.confirmation_status = "confirmed"
            return best_id

        oid = self._new_id(class_name)
        unc = float(bbox_diag_m) if bbox_diag_m else self.assoc_thresh_m
        self.objects[oid] = MemoryObject(
            object_id=oid,
            class_id=class_name,
            class_name=class_name,
            class_evidence=[{"view_id": int(view_id), "class": class_name, "conf": float(confidence)}],
            position_world=c.tolist(),
            position_uncertainty=unc,
            observation_count=1,
            last_observed_time=stamp,
            last_camera_pose=list(camera_pose) if camera_pose is not None else None,
            observation_quality=float(confidence),
            confirmation_status="conflict" if conflict else "tentative",
            association_conflict=bool(conflict),
            view_ids=[int(view_id)],
            center_world=c.tolist(),
            confidence=float(confidence),
            n_observations=1,
        )
        return oid

    def ingest_perception_json(self, perc: dict) -> None:
        for view in perc.get("views", []):
            vid = int(view.get("view_id", 0))
            cam = view.get("camera_pose_xyz")
            for o in view.get("objects", []):
                self.upsert(
                    o["class_name"],
                    o["center_world"],
                    confidence=float(o.get("confidence", 0.0)),
                    view_id=vid,
                    camera_pose=cam,
                    stamp=view.get("stamp"),
                )

    def confirmed_objects(self) -> List[MemoryObject]:
        return [o for o in self.objects.values() if o.confirmation_status == "confirmed"]

    def to_dict(self) -> dict:
        return {
            "assoc_thresh_m": self.assoc_thresh_m,
            "confirm_min_obs": self.confirm_min_obs,
            "n_objects": len(self.objects),
            "n_confirmed": len(self.confirmed_objects()),
            "objects": [asdict(o) for o in self.objects.values()],
        }

    def save(self, path: str) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path: str) -> "ObjectMemory":
        data = json.loads(Path(path).read_text())
        mem = cls(
            assoc_thresh_m=float(data.get("assoc_thresh_m", 0.25)),
            confirm_min_obs=int(data.get("confirm_min_obs", 2)),
        )
        for o in data.get("objects", []):
            # tolerate older schema
            if "class_id" not in o:
                o["class_id"] = o.get("class_name", "obj")
            if "position_world" not in o and "center_world" in o:
                o["position_world"] = o["center_world"]
            if "observation_count" not in o and "n_observations" in o:
                o["observation_count"] = o["n_observations"]
            mem.objects[o["object_id"]] = MemoryObject(**{k: v for k, v in o.items() if k in MemoryObject.__dataclass_fields__})
            try:
                mem._next_id = max(mem._next_id, int(o["object_id"].rsplit("_", 1)[-1]) + 1)
            except Exception:
                pass
        return mem
