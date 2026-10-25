#!/usr/bin/env python3
"""Geometry self-filter for AGX Ranger+CR10+AG95+D455 depth clouds.

Uses link TFs at the cloud stamp and conservative capsule/box proxies derived from
the URDF kinematic tree (not a blanket XY radius cull). Approximation error is
documented in APPROX_NOTE / per-link radii.
"""
from __future__ import print_function

import math

import numpy as np

# Conservative proxies in each link frame. Radii/half-extents intentionally a bit
# larger than mesh (≈2–4 cm) so REMANI collision spheres (thickness≈0.06–0.10)
# do not see residual robot body returns. Nearby shelves/tables outside these
# volumes are preserved.
APPROX_NOTE = (
    "Mesh→capsule/box: base deck box ±4 cm; wheel spheres +3 cm; CR10 links "
    "capsules ≈ REMANI sample envelopes + manipulator_thickness(0.06)+inflate; "
    "gripper/camera boxes +3 cm. Not a full convex hull."
)

# (link_name, kind, params)
# kind=box: half_xyz + origin_xyz in link frame
# kind=sphere: radius + origin_xyz
# kind=capsule: radius, p0, p1 in link frame
LINK_PROXIES = [
    # Ranger chassis — match REMANI mm_param_ranger_cr10 L=1.10 W=0.90
    ("base_link", "box", {"half": (0.55, 0.45, 0.25), "origin": (0.0, 0.0, 0.12)}),
    ("box_link", "box", {"half": (0.40, 0.35, 0.28), "origin": (0.0, 0.0, 0.22)}),
    ("fr_wheel_link", "sphere", {"radius": 0.18, "origin": (0.0, 0.0, 0.0)}),
    ("fl_wheel_link", "sphere", {"radius": 0.18, "origin": (0.0, 0.0, 0.0)}),
    ("rl_wheel_link", "sphere", {"radius": 0.18, "origin": (0.0, 0.0, 0.0)}),
    ("rr_wheel_link", "sphere", {"radius": 0.18, "origin": (0.0, 0.0, 0.0)}),
    # CR10
    ("cr10_base_link", "box", {"half": (0.12, 0.12, 0.12), "origin": (0.0, 0.0, 0.08)}),
    ("cr10_Link1", "sphere", {"radius": 0.14, "origin": (0.0, 0.0, 0.0)}),
    ("cr10_Link2", "capsule", {"radius": 0.12, "p0": (0.0, 0.0, 0.0), "p1": (-0.60, 0.0, 0.0)}),
    ("cr10_Link3", "capsule", {"radius": 0.11, "p0": (0.0, 0.0, 0.0), "p1": (-0.55, 0.0, 0.19)}),
    ("cr10_Link4", "sphere", {"radius": 0.12, "origin": (0.0, 0.0, 0.0)}),
    ("cr10_Link5", "sphere", {"radius": 0.12, "origin": (0.0, 0.0, 0.0)}),
    ("cr10_Link6", "sphere", {"radius": 0.12, "origin": (0.0, 0.0, 0.05)}),
    # Gripper + camera mount
    ("gripper_base_link", "box", {"half": (0.08, 0.08, 0.08), "origin": (0.0, 0.0, 0.05)}),
    ("gripper_finger1_finger_tip_link", "box", {"half": (0.03, 0.02, 0.04), "origin": (0.0, 0.0, 0.0)}),
    ("gripper_finger2_finger_tip_link", "box", {"half": (0.03, 0.02, 0.04), "origin": (0.0, 0.0, 0.0)}),
    ("d455_link", "box", {"half": (0.06, 0.08, 0.04), "origin": (0.0, 0.0, 0.0)}),
    ("lidar_link0", "sphere", {"radius": 0.10, "origin": (0.0, 0.0, 0.0)}),
]

SELF_FILTER_LINKS = [p[0] for p in LINK_PROXIES]


def _quat_to_R(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def transform_from_tf(tf_msg):
    t = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    T = np.eye(4)
    T[0:3, 0:3] = _quat_to_R(r.x, r.y, r.z, r.w)
    T[0:3, 3] = [t.x, t.y, t.z]
    return T


def lookup_link_transforms(tf_buffer, stamp, world_frame="world", timeout=0.15):
    """Return {link: 4x4 T_world_link} at stamp. Raises if any required TF missing."""
    import rospy

    out = {}
    stamp = rospy.Time(stamp.secs, stamp.nsecs) if hasattr(stamp, "secs") else stamp
    for link, _, _ in LINK_PROXIES:
        tfm = tf_buffer.lookup_transform(world_frame, link, stamp, rospy.Duration(timeout))
        out[link] = transform_from_tf(tfm)
    return out


def _point_in_box(pw, T_w_l, half, origin, inflate=0.0):
    R = T_w_l[0:3, 0:3]
    t = T_w_l[0:3, 3]
    pl = R.T.dot(pw - t) - np.asarray(origin, dtype=float)
    h = np.asarray(half, dtype=float) + inflate
    return bool(np.all(np.abs(pl) <= h))


def _point_in_sphere(pw, T_w_l, radius, origin, inflate=0.0):
    R = T_w_l[0:3, 0:3]
    t = T_w_l[0:3, 3]
    c = t + R.dot(np.asarray(origin, dtype=float))
    return float(np.linalg.norm(pw - c)) <= (radius + inflate)


def _point_in_capsule(pw, T_w_l, radius, p0, p1, inflate=0.0):
    R = T_w_l[0:3, 0:3]
    t = T_w_l[0:3, 3]
    a = t + R.dot(np.asarray(p0, dtype=float))
    b = t + R.dot(np.asarray(p1, dtype=float))
    ab = b - a
    den = float(np.dot(ab, ab))
    if den < 1e-12:
        return float(np.linalg.norm(pw - a)) <= (radius + inflate)
    u = max(0.0, min(1.0, float(np.dot(pw - a, ab) / den)))
    closest = a + u * ab
    return float(np.linalg.norm(pw - closest)) <= (radius + inflate)


def point_is_robot(pw, link_T, inflate=0.02):
    """True if world point pw is inside any link proxy."""
    pw = np.asarray(pw, dtype=float)
    for link, kind, params in LINK_PROXIES:
        T = link_T.get(link)
        if T is None:
            continue
        if kind == "box":
            if _point_in_box(pw, T, params["half"], params["origin"], inflate):
                return True
        elif kind == "sphere":
            if _point_in_sphere(pw, T, params["radius"], params["origin"], inflate):
                return True
        elif kind == "capsule":
            if _point_in_capsule(pw, T, params["radius"], params["p0"], params["p1"], inflate):
                return True
    return False


def filter_points(points_xyz, link_T, inflate=0.02):
    """Return (kept, removed_count). points_xyz: Nx3 or list of (x,y,z)."""
    if points_xyz is None:
        return [], 0
    arr = np.asarray(points_xyz, dtype=float)
    if arr.size == 0:
        return [], 0
    if arr.ndim == 1:
        arr = arr.reshape(1, 3)
    kept = []
    removed = 0
    for p in arr:
        if point_is_robot(p, link_T, inflate=inflate):
            removed += 1
        else:
            kept.append((float(p[0]), float(p[1]), float(p[2])))
    return kept, removed


def synthetic_self_and_env_cloud(base_xy=(-2.1, -5.89), table_xy=(-3.9, -7.5)):
    """Unit-test helper: robot-body points + nearby table points (no ROS)."""
    bx, by = base_xy
    robot = []
    for dx in np.linspace(-0.3, 0.3, 7):
        for dy in np.linspace(-0.2, 0.2, 5):
            for z in (0.2, 0.4, 0.6):
                robot.append((bx + dx, by + dy, z))
    # arm-ish points above base
    for z in np.linspace(0.5, 1.2, 8):
        robot.append((bx + 0.1, by, z))
    env = []
    tx, ty = table_xy
    for dx in np.linspace(-0.3, 0.3, 7):
        for dy in np.linspace(-0.2, 0.2, 5):
            env.append((tx + dx, ty + dy, 0.74))
    # shelf face near but outside chassis
    for y in np.linspace(-7.0, -6.0, 11):
        env.append((-5.0, y, 1.0))
    return robot, env


def identity_base_link_T(base_xy=(-2.1, -5.89), z=0.42):
    """Fake link TFs for offline unit tests (robot at aisle_safe, arm folded approx)."""
    out = {}
    Tbase = np.eye(4)
    Tbase[0:3, 3] = [base_xy[0], base_xy[1], z]
    for link, _, _ in LINK_PROXIES:
        T = Tbase.copy()
        if link.startswith("cr10_Link"):
            # raise along z for crude arm stack
            idx = int(link[-1]) if link[-1].isdigit() else 0
            T[2, 3] = z + 0.2 + 0.12 * idx
        elif link == "d455_link":
            T[2, 3] = z + 1.0
            T[0, 3] = base_xy[0] + 0.3
        elif "wheel" in link:
            T[2, 3] = z - 0.25
        out[link] = T
    return out


def offline_self_filter_smoke():
    link_T = identity_base_link_T()
    robot, env = synthetic_self_and_env_cloud()
    kept_r, rem_r = filter_points(robot, link_T, inflate=0.02)
    kept_e, rem_e = filter_points(env, link_T, inflate=0.02)
    return {
        "approx_note": APPROX_NOTE,
        "robot_in": len(robot),
        "robot_removed": rem_r,
        "robot_kept": len(kept_r),
        "env_in": len(env),
        "env_removed": rem_e,
        "env_kept": len(kept_e),
        "ok": rem_r >= int(0.7 * len(robot)) and rem_e <= int(0.05 * len(env)),
    }


if __name__ == "__main__":
    import json

    print(json.dumps(offline_self_filter_smoke(), indent=2))
