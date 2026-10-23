#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path("/workspace/third_party/gazebo_objects/gazebo_ycb/models")
for m in ["mustard_bottle", "banana", "cracker_box", "tomato_soup_can", "apple", "bowl"]:
    dae = ROOT / m / "meshes" / "textured.dae"
    if not dae.is_file():
        print(m, "MISSING_DAE")
        continue
    t = dae.read_text(errors="ignore")
    u = re.search(r'meter="([^"]+)"', t)
    arrays = re.findall(r"<float_array[^>]*>([^<]+)</float_array>", t)
    vals = []
    for a in arrays[:3]:
        vals.extend(float(x) for x in a.split()[:5000])
    if vals:
        print(
            m,
            "unit",
            u.group(1) if u else None,
            "minmax",
            round(min(vals), 4),
            round(max(vals), 4),
            "bytes",
            dae.stat().st_size,
        )
    else:
        print(m, "no_float_array", "bytes", dae.stat().st_size)
