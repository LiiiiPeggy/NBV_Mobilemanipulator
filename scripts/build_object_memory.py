#!/usr/bin/env python3
"""Build ObjectMemory JSON from a perceive_rgbd_objects output."""
from __future__ import print_function
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from perception.object_memory import ObjectMemory


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pred", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--assoc-thresh", type=float, default=0.25)
    args = ap.parse_args()
    perc = json.loads(Path(args.pred).read_text())
    mem = ObjectMemory(assoc_thresh_m=args.assoc_thresh)
    mem.ingest_perception_json(perc)
    mem.save(args.out)
    print("n_objects", mem.to_dict()["n_objects"])
    print("BUILD_OBJECT_MEMORY_OK")


if __name__ == "__main__":
    main()
