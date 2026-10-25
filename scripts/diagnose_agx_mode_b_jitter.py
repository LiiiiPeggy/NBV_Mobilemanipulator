#!/usr/bin/env python3
"""Quantify Mode B Gazebo/RViz jitter (P0).

Records Gazebo model/link poses, joints, TF, /joint_states rate & duplicate ratio,
and optional RTF. Writes JSON (+ optional CSV row).

Usage:
  python3 diagnose_agx_mode_b_jitter.py --tag before_idle --duration 20 --out ...
"""
from __future__ import print_function

import argparse
import csv
import json
import math
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
from gazebo_msgs.msg import LinkStates, ModelStates
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


ARM = [
    "cr10_joint1",
    "cr10_joint2",
    "cr10_joint3",
    "cr10_joint4",
    "cr10_joint5",
    "cr10_joint6",
]


def stats(arr):
    a = np.asarray(arr, dtype=float)
    if a.size == 0:
        return {"n": 0}
    return {
        "n": int(a.size),
        "mean": float(np.mean(a)),
        "std": float(np.std(a)),
        "ptp": float(np.ptp(a)),
        "min": float(np.min(a)),
        "max": float(np.max(a)),
    }


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rpy_from_q(q):
    # roll pitch yaw
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    yaw = yaw_from_q(q)
    return roll, pitch, yaw


class Collector(object):
    def __init__(self, model):
        self.model = model
        self.js_count = 0
        self.js_dup = 0
        self._last_js_key = None
        self.js_times = []
        self.base_xyz = []
        self.base_rpy = []
        self.arm = defaultdict(list)
        self.p3d_xyz = []
        self.cam_xyz = []
        self.cam_rpy = []
        self.link6_xyz = []
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        rospy.Subscriber("/joint_states", JointState, self._on_js, queue_size=200)
        rospy.Subscriber("/ground_truth/state", Odometry, self._on_p3d, queue_size=50)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_models, queue_size=20)
        self._get = None
        try:
            rospy.wait_for_service("/gazebo/get_model_state", timeout=5)
            self._get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        except Exception:
            pass

    def _on_js(self, msg):
        self.js_count += 1
        self.js_times.append(rospy.Time.now().to_sec())
        key = tuple(round(p, 6) for p in msg.position[: min(12, len(msg.position))])
        if key == self._last_js_key:
            self.js_dup += 1
        self._last_js_key = key
        name_to_p = dict(zip(msg.name, msg.position))
        for j in ARM:
            if j in name_to_p:
                self.arm[j].append(float(name_to_p[j]))

    def _on_p3d(self, msg):
        p = msg.pose.pose.position
        self.p3d_xyz.append([p.x, p.y, p.z])

    def _on_models(self, msg):
        if self.model not in msg.name:
            return
        i = msg.name.index(self.model)
        p = msg.pose[i].position
        o = msg.pose[i].orientation
        self.base_xyz.append([p.x, p.y, p.z])
        self.base_rpy.append(list(rpy_from_q(o)))

    def sample_tf(self):
        try:
            t = self.buf.lookup_transform(
                "world", "d455_depth_optical_frame", rospy.Time(0), rospy.Duration(0.05)
            )
            p = t.transform.translation
            o = t.transform.rotation
            self.cam_xyz.append([p.x, p.y, p.z])
            self.cam_rpy.append(list(rpy_from_q(o)))
        except Exception:
            pass
        try:
            t = self.buf.lookup_transform("world", "cr10_Link6", rospy.Time(0), rospy.Duration(0.05))
            p = t.transform.translation
            self.link6_xyz.append([p.x, p.y, p.z])
        except Exception:
            pass

    def gz_sample(self):
        if not self._get:
            return
        try:
            st = self._get(self.model, "world")
            if st.success:
                p = st.pose.position
                o = st.pose.orientation
                # also stored via model_states; keep explicit
                _ = (p, o)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--model", default="agx_ranger_cr10")
    ap.add_argument(
        "--out",
        default="data/metrics/fkie_agx_warehouse_mode_b/jitter_diag.json",
    )
    ap.add_argument(
        "--csv",
        default="reports/agx_mode_b_jitter_before_after.csv",
    )
    # Pre-declared idle thresholds (do not loosen post-hoc)
    ap.add_argument("--thr-base-xy-std", dest="thr_base_xy_std", type=float, default=0.002)
    ap.add_argument("--thr-base-z-std", type=float, default=0.003)
    ap.add_argument("--thr-yaw-std-deg", type=float, default=0.5)
    ap.add_argument("--thr-arm-std-rad", type=float, default=0.005)
    ap.add_argument("--thr-cam-xy-std", type=float, default=0.005)
    ap.add_argument("--thr-js-hz-max", type=float, default=200.0)
    args = ap.parse_args()

    rospy.init_node("diagnose_agx_mode_b_jitter", anonymous=True)
    col = Collector(args.model)
    rospy.sleep(0.5)
    t0 = time.time()
    while time.time() - t0 < args.duration and not rospy.is_shutdown():
        col.sample_tf()
        col.gz_sample()
        rospy.sleep(0.05)

    dur = max(1e-6, time.time() - t0)
    js_hz = col.js_count / dur
    js_dup_ratio = col.js_dup / float(col.js_count) if col.js_count else 0.0

    base_xyz = np.asarray(col.base_xyz) if col.base_xyz else np.zeros((0, 3))
    base_rpy = np.asarray(col.base_rpy) if col.base_rpy else np.zeros((0, 3))
    p3d = np.asarray(col.p3d_xyz) if col.p3d_xyz else np.zeros((0, 3))
    cam = np.asarray(col.cam_xyz) if col.cam_xyz else np.zeros((0, 3))

    arm_stats = {j: stats(col.arm[j]) for j in ARM}
    out = {
        "tag": args.tag,
        "duration_s": dur,
        "model": args.model,
        "joint_states": {
            "count": col.js_count,
            "hz": js_hz,
            "dup_ratio": js_dup_ratio,
        },
        "base_gazebo": {
            "x": stats(base_xyz[:, 0]) if base_xyz.size else stats([]),
            "y": stats(base_xyz[:, 1]) if base_xyz.size else stats([]),
            "z": stats(base_xyz[:, 2]) if base_xyz.size else stats([]),
            "roll": stats(base_rpy[:, 0]) if base_rpy.size else stats([]),
            "pitch": stats(base_rpy[:, 1]) if base_rpy.size else stats([]),
            "yaw": stats(base_rpy[:, 2]) if base_rpy.size else stats([]),
        },
        "p3d": {
            "x": stats(p3d[:, 0]) if p3d.size else stats([]),
            "y": stats(p3d[:, 1]) if p3d.size else stats([]),
            "z": stats(p3d[:, 2]) if p3d.size else stats([]),
        },
        "camera_tf": {
            "x": stats(cam[:, 0]) if cam.size else stats([]),
            "y": stats(cam[:, 1]) if cam.size else stats([]),
            "z": stats(cam[:, 2]) if cam.size else stats([]),
        },
        "arm_joints": arm_stats,
        "thresholds": {
            "base_xy_std": args.thr_base_xy_std,
            "base_z_std": args.thr_base_z_std,
            "yaw_std_deg": args.thr_yaw_std_deg,
            "arm_std_rad": args.thr_arm_std_rad,
            "cam_xy_std": args.thr_cam_xy_std,
            "js_hz_max": args.thr_js_hz_max,
        },
    }

    def sget(d, k):
        return float(d.get(k, {}).get("std", 1e9) or 1e9)

    yaw_std_deg = math.degrees(sget(out["base_gazebo"], "yaw"))
    arm_ok = all(
        (arm_stats[j].get("std", 1e9) or 1e9) <= args.thr_arm_std_rad for j in ARM if arm_stats[j].get("n", 0) > 0
    )
    gates = {
        "js_hz_ok": js_hz <= args.thr_js_hz_max,
        "base_xy_ok": max(sget(out["base_gazebo"], "x"), sget(out["base_gazebo"], "y"))
        <= args.thr_base_xy_std,
        "base_z_ok": sget(out["base_gazebo"], "z") <= args.thr_base_z_std,
        "yaw_ok": yaw_std_deg <= args.thr_yaw_std_deg,
        "arm_ok": arm_ok,
        "cam_xy_ok": max(sget(out["camera_tf"], "x"), sget(out["camera_tf"], "y"))
        <= args.thr_cam_xy_std,
    }
    out["gates"] = gates
    out["ok_idle"] = all(gates.values())

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))

    csv_path = Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    row = {
        "tag": args.tag,
        "js_hz": js_hz,
        "js_dup_ratio": js_dup_ratio,
        "base_x_std": sget(out["base_gazebo"], "x"),
        "base_y_std": sget(out["base_gazebo"], "y"),
        "base_z_std": sget(out["base_gazebo"], "z"),
        "yaw_std_deg": yaw_std_deg,
        "cam_x_std": sget(out["camera_tf"], "x"),
        "cam_y_std": sget(out["camera_tf"], "y"),
        "arm_j2_std": arm_stats["cr10_joint2"].get("std"),
        "ok_idle": out["ok_idle"],
    }
    write_header = not csv_path.is_file()
    with csv_path.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if write_header:
            w.writeheader()
        w.writerow(row)

    print(json.dumps(out, indent=2))
    print("JITTER_IDLE_OK" if out["ok_idle"] else "JITTER_IDLE_FAIL")
    if not out["ok_idle"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
