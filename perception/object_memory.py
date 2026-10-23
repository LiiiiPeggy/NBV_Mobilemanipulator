#!/usr/bin/env python3
"""Simple multi-view object memory with greedy spatial association."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple
import json
from pathlib import Path

import numpy as np


@dataclass
class MemoryObject:
    object_id: str
    class_name: str
    center_world: List[float]
    confidence: float
    n_observations: int = 1
    view_ids: List[int] = field(default_factory=list)


class ObjectMemory:
    def __init__(self, assoc_thresh_m: float = 0.25):
        self.assoc_thresh_m = float(assoc_thresh_m)
        self.objects: Dict[str, MemoryObject] = {}
        self._next_id = 0

    def _new_id(self, class_name: str) -> str:
        oid = "mem_%s_%03d" % (class_name, self._next_id)
        self._next_id += 1
        return oid

    def upsert(self, class_name: str, center_world, confidence: float = 0.0, view_id: int = 0) -> str:
        c = np.asarray(center_world, dtype=np.float64).reshape(3)
        best_id, best_d = None, 1e9
        for oid, obj in self.objects.items():
            if obj.class_name != class_name:
                continue
            d = float(np.linalg.norm(c - np.asarray(obj.center_world)))
            if d < best_d:
                best_d, best_id = d, oid
        if best_id is not None and best_d <= self.assoc_thresh_m:
            obj = self.objects[best_id]
            n = obj.n_observations
            prev = np.asarray(obj.center_world, dtype=np.float64)
            fused = (prev * n + c) / float(n + 1)
            obj.center_world = fused.tolist()
            obj.confidence = max(obj.confidence, float(confidence))
            obj.n_observations = n + 1
            if view_id not in obj.view_ids:
                obj.view_ids.append(int(view_id))
            return best_id
        oid = self._new_id(class_name)
        self.objects[oid] = MemoryObject(
            object_id=oid,
            class_name=class_name,
            center_world=c.tolist(),
            confidence=float(confidence),
            n_observations=1,
            view_ids=[int(view_id)],
        )
        return oid

    def ingest_perception_json(self, perc: dict) -> None:
        for view in perc.get("views", []):
            vid = int(view.get("view_id", 0))
            for o in view.get("objects", []):
                self.upsert(
                    o["class_name"],
                    o["center_world"],
                    confidence=float(o.get("confidence", 0.0)),
                    view_id=vid,
                )

    def to_dict(self) -> dict:
        return {
            "assoc_thresh_m": self.assoc_thresh_m,
            "n_objects": len(self.objects),
            "objects": [asdict(o) for o in self.objects.values()],
        }

    def save(self, path: str) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path: str) -> "ObjectMemory":
        data = json.loads(Path(path).read_text())
        mem = cls(assoc_thresh_m=float(data.get("assoc_thresh_m", 0.25)))
        for o in data.get("objects", []):
            mem.objects[o["object_id"]] = MemoryObject(**o)
            mem._next_id = max(mem._next_id, int(o["object_id"].rsplit("_", 1)[-1]) + 1)
        return mem
