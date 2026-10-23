#!/usr/bin/env python3
"""Search CR10 bake pose so D455 optical (+Z) looks near a world target (Mode A FK).

Uses analytical FK from source URDF (no per-candidate URDF bake) — fast grid search.
"""
from __future__ import print_function
import argparse
import math
import re
from pathlib import Path

import numpy as np
import yaml


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def axis_angle(axis, q):
    ax = np.asarray(axis, dtype=float)
    ax = ax / (np.linalg.norm(ax) + 1e-12)
    x, y, z = ax
    c, s = math.cos(q), math.sin(q)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def T_origin(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def parse_joint(urdf_text, name):
    m = re.search(
        r'<joint[^>]*name="%s"[^>]*>(.*?)</joint>' % re.escape(name),
        urdf_text,
        re.S,
    )
    if not m:
        raise KeyError(name)
    block = m.group(1)
    om = re.search(r'<origin[^>]*xyz="([^"]+)"[^>]*rpy="([^"]+)"', block)
    if om:
        xyz = [float(x) for x in om.group(1).split()]
        rpy = [float(x) for x in om.group(2).split()]
    else:
        om = re.search(r'<origin[^>]*rpy="([^"]+)"[^>]*xyz="([^"]+)"', block)
        if not om:
            xyz, rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        else:
            rpy = [float(x) for x in om.group(1).split()]
            xyz = [float(x) for x in om.group(2).split()]
    am = re.search(r'<axis[^>]*xyz="([^"]+)"', block)
    axis = [float(x) for x in am.group(1).split()] if am else [0.0, 0.0, 1.0]
    return xyz, rpy, axis


def load_chain(urdf_path):
    text = Path(urdf_path).read_text()
    # base_link -> cr10_base_link -> j1..j6 -> gripper -> d455 optical
    names = [
        "cr10_mount_joint",
        "cr10_joint1",
        "cr10_joint2",
        "cr10_joint3",
        "cr10_joint4",
        "cr10_joint5",
        "cr10_joint6",
        "cr10_Link6_gripper_base_fixed",
        "d455_joint",
        "d455_link_joint",
        "d455_depth_joint",
        "d455_depth_optical_joint",
    ]
    specs = []
    movable = {
        "cr10_joint1",
        "cr10_joint2",
        "cr10_joint3",
        "cr10_joint4",
        "cr10_joint5",
        "cr10_joint6",
    }
    for n in names:
        xyz, rpy, axis = parse_joint(text, n)
        specs.append({"name": n, "xyz": xyz, "rpy": rpy, "axis": axis, "movable": n in movable})
    return specs


def fk_optical(specs, qdict, Tb):
    T = Tb.copy()
    for s in specs:
        T = T @ T_origin(s["xyz"], s["rpy"])
        if s["movable"]:
            q = float(qdict.get(s["name"], 0.0))
            R = np.eye(4)
            R[:3, :3] = axis_angle(s["axis"], q)
            T = T @ R
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", nargs=3, type=float, default=[-2.65, -6.80, 0.85])
    ap.add_argument("--base", nargs=4, type=float, default=[-2.10, -5.89, 0.42, 2.833])
    ap.add_argument(
        "--src-urdf",
        default="/workspace/third_party/agx/rangerboxcr10lidar_description/urdf/rangercr10lidar.urdf",
    )
    ap.add_argument("--out-yaml", default="/workspace/configs/fkie/agx_adapt/config/arm_observe_aisle_table.yaml")
    ap.add_argument("--max-ang-deg", type=float, default=25.0)
    args = ap.parse_args()

    src = Path(args.src_urdf)
    if not src.is_file():
        # host path fallback
        alt = Path(__file__).resolve().parents[1] / "third_party/agx/rangerboxcr10lidar_description/urdf/rangercr10lidar.urdf"
        src = alt
    specs = load_chain(src)

    bx, by, bz, byaw = args.base
    Tb = np.eye(4)
    Tb[:3, :3] = rpy_matrix(0, 0, byaw)
    Tb[:3, 3] = [bx, by, bz]
    target = np.array(args.target, dtype=float)

    # Prefer joint1 that yaws arm toward table in base frame.
    dx, dy = target[0] - bx, target[1] - by
    world_bearing = math.atan2(dy, dx)
    # base_link +X is forward; joint1 rotates about Z of cr10_base (aligned with base).
    # Desired world bearing relative to base yaw ≈ joint1 contribution (approx).
    j1_hint = world_bearing - byaw
    # wrap to joint1 range roughly [-3.92, 0.94]
    while j1_hint > math.pi:
        j1_hint -= 2 * math.pi
    while j1_hint < -math.pi:
        j1_hint += 2 * math.pi

    best = None
    n = 0
    j1s = np.unique(
        np.clip(
            np.concatenate(
                [
                    np.linspace(j1_hint - 1.0, j1_hint + 1.0, 21),
                    np.linspace(-1.8, 0.6, 13),
                ]
            ),
            -3.5,
            0.9,
        )
    )
    for j1 in j1s:
        for j2 in np.linspace(-1.57, -0.35, 14):
            for j3 in np.linspace(0.4, 1.7, 14):
                for j5 in np.linspace(0.2, 1.7, 12):
                    n += 1
                    pose = {
                        "cr10_joint1": float(j1),
                        "cr10_joint2": float(j2),
                        "cr10_joint3": float(j3),
                        "cr10_joint4": 0.0,
                        "cr10_joint5": float(j5),
                        "cr10_joint6": 0.0,
                        "gripper_finger1_joint": 0.0,
                    }
                    Tw = fk_optical(specs, pose, Tb)
                    cam = Tw[:3, 3]
                    fwd = Tw[:3, 2]  # optical +Z
                    v = target - cam
                    dist = float(np.linalg.norm(v))
                    if dist < 0.4 or dist > 3.5:
                        continue
                    v = v / dist
                    c = float(np.clip(np.dot(v, fwd), -1.0, 1.0))
                    ang = math.degrees(math.acos(c))
                    # Prefer looking slightly down onto table, not through floor.
                    down = float(fwd[2])  # optical Z world; negative = looking down
                    score = ang + 0.2 * abs(dist - 1.5) + (0.0 if down < -0.05 else 8.0)
                    if best is None or score < best[0]:
                        best = (score, ang, dist, pose, cam.copy(), fwd.copy())

    if best is None:
        raise SystemExit("no candidate")
    score, ang, dist, pose, cam, fwd = best
    out = Path(args.out_yaml)
    if str(out).startswith("/workspace/") and not Path("/workspace").exists():
        out = Path(__file__).resolve().parents[1] / str(out).replace("/workspace/", "")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        "# Auto-searched to look at clear-aisle table ROI (no shelf OBB).\n"
        "# target=%s ang=%.1fdeg dist=%.2fm cam=%s fwd=%s\n"
        % (list(args.target), ang, dist, cam.tolist(), fwd.tolist())
        + yaml.safe_dump(pose)
    )
    print("tried", n)
    print("BEST ang=%.1f dist=%.2f score=%.2f" % (ang, dist, score))
    print("cam", cam.tolist())
    print("fwd", fwd.tolist())
    print("j1_hint", j1_hint)
    print("Wrote", out)
    if ang > args.max_ang_deg:
        print("WARN: ang>%.1f — may need finer search or different table" % args.max_ang_deg)
    print(yaml.safe_dump(pose))


if __name__ == "__main__":
    main()
