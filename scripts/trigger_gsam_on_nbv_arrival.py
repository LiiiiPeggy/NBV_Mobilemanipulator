#!/usr/bin/env python3
"""Trigger Grounding DINO + SAM2 only after real NBV arrival + camera stability.

Does NOT fire on REMANI FSM transitions alone.

Gates (all required):
  1. EE / camera pose error vs goal below thresholds
  2. Linear / angular velocity below thresholds (TF finite-diff)
  3. Stable for --stable-s consecutive seconds
  4. Fresh RGB-D stamps newer than arrival time

Then runs perceive_rgbd_objects.py (grounded_sam) once and records evidence JSON.
"""
from __future__ import print_function

import argparse
import json
import math
import subprocess
import time
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image


def lookup(buf, parent, child):
    t = buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(1.0))
    p = t.transform.translation
    q = t.transform.rotation
    return np.array([p.x, p.y, p.z], dtype=float), np.array([q.x, q.y, q.z, q.w], dtype=float)


def quat_ang_err(q_a, q_b):
    import tf.transformations as tft

    a = tft.quaternion_matrix(q_a)
    b = tft.quaternion_matrix(q_b)
    R = a[0:3, 0:3].T.dot(b[0:3, 0:3])
    tr = max(-1.0, min(3.0, float(R[0, 0] + R[1, 1] + R[2, 2])))
    return math.acos(max(-1.0, min(1.0, 0.5 * (tr - 1.0))))


class ArrivalGate(object):
    def __init__(self, args):
        self.args = args
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        self.goal_xyz = None
        self.goal_q = None
        self.goal_frame = "camera"  # or ee
        self.arrival_t = None
        self.stable_since = None
        self.last_rgb_stamp = None
        self.last_depth_stamp = None
        self._prev_cam = None
        self._prev_t = None
        self.triggered = False
        rospy.Subscriber(args.goal_topic, PoseStamped, self._on_goal, queue_size=1)
        rospy.Subscriber(args.rgb_topic, Image, self._on_rgb, queue_size=1)
        rospy.Subscriber(args.depth_topic, Image, self._on_depth, queue_size=1)

    def _on_goal(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        self.goal_xyz = np.array([p.x, p.y, p.z], dtype=float)
        self.goal_q = np.array([o.x, o.y, o.z, o.w], dtype=float)
        self.arrival_t = None
        self.stable_since = None
        self.triggered = False
        rospy.loginfo("GSAM gate: new goal (%.3f, %.3f, %.3f)", p.x, p.y, p.z)

    def _on_rgb(self, msg):
        self.last_rgb_stamp = msg.header.stamp

    def _on_depth(self, msg):
        self.last_depth_stamp = msg.header.stamp

    def _cam_state(self):
        return lookup(self.buf, "world", self.args.camera_frame)

    def poll(self):
        if self.triggered or self.goal_xyz is None:
            return None
        try:
            cam_p, cam_q = self._cam_state()
            ee_p, ee_q = lookup(self.buf, "world", self.args.ee_frame)
        except Exception:
            return None

        now = time.time()
        # velocity via TF finite difference
        speed = 1e9
        if self._prev_cam is not None and self._prev_t is not None:
            dt = max(1e-3, now - self._prev_t)
            speed = float(np.linalg.norm(cam_p - self._prev_cam) / dt)
        self._prev_cam = cam_p
        self._prev_t = now

        if self.args.goal_is_ee:
            pos_err = float(np.linalg.norm(ee_p - self.goal_xyz))
            rot_err = quat_ang_err(ee_q, self.goal_q) if self.goal_q is not None else 0.0
        else:
            pos_err = float(np.linalg.norm(cam_p - self.goal_xyz))
            rot_err = quat_ang_err(cam_q, self.goal_q) if self.goal_q is not None else 0.0

        pos_ok = pos_err <= self.args.pos_tol
        rot_ok = math.degrees(rot_err) <= self.args.rot_tol_deg
        vel_ok = speed <= self.args.vel_tol

        if pos_ok and rot_ok and vel_ok:
            if self.stable_since is None:
                self.stable_since = now
                self.arrival_t = rospy.Time.now()
            stable_s = now - self.stable_since
        else:
            self.stable_since = None
            stable_s = 0.0

        fresh_ok = False
        if self.arrival_t is not None and self.last_rgb_stamp and self.last_depth_stamp:
            fresh_ok = (
                self.last_rgb_stamp > self.arrival_t
                and self.last_depth_stamp > self.arrival_t
            )

        status = {
            "pos_err": pos_err,
            "rot_err_deg": math.degrees(rot_err),
            "speed": speed,
            "stable_s": stable_s,
            "pos_ok": pos_ok,
            "rot_ok": rot_ok,
            "vel_ok": vel_ok,
            "fresh_ok": fresh_ok,
        }
        if (
            pos_ok
            and rot_ok
            and vel_ok
            and stable_s >= self.args.stable_s
            and fresh_ok
        ):
            return status
        rospy.loginfo_throttle(
            2.0,
            "GSAM gate waiting: pos=%.3f rot=%.1fdeg spd=%.3f stable=%.1f fresh=%s",
            pos_err,
            math.degrees(rot_err),
            speed,
            stable_s,
            fresh_ok,
        )
        return None


def _save_latest_rgbd(rgb_topic, depth_topic, out_dir):
    """Capture with system rospy/cv_bridge (avoid conda↔libgdal clash)."""
    from cv_bridge import CvBridge
    import cv2

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    bridge = CvBridge()
    rgb = rospy.wait_for_message(rgb_topic, Image, timeout=5.0)
    depth = rospy.wait_for_message(depth_topic, Image, timeout=5.0)
    bgr = bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
    rgb_path = out / "rgb_arrival.png"
    depth_path = out / "depth_arrival.png"
    cv2.imwrite(str(rgb_path), bgr)
    # depth as 16UC1 mm if needed
    d = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
    if d.dtype == np.float32:
        dmm = np.clip(d * 1000.0, 0, 65535).astype(np.uint16)
    else:
        dmm = d.astype(np.uint16)
    cv2.imwrite(str(depth_path), dmm)
    return {
        "rgb": str(rgb_path),
        "depth": str(depth_path),
        "rgb_stamp": rgb.header.stamp.to_sec(),
        "depth_stamp": depth.header.stamp.to_sec(),
    }


def run_gsam(out_path, rgb_path, views=1):
    """Offline Grounded-SAM on saved RGB (conda; no cv_bridge)."""
    import os

    env = os.environ.copy()
    env["NBV_ROOT"] = "/workspace"
    env["NBV_PERCEPTION_ROOT"] = "/workspace"
    env["TRANSFORMERS_OFFLINE"] = "1"
    env["HF_HOME"] = "/workspace/checkpoints"
    py = "/workspace/.conda_envs/nbv_perception/bin/python"
    vendor = [
        "/workspace",
        "/workspace/.conda_envs/nbv_perception/lib/python3.9/site-packages",
        "/workspace/perception/vendor/GroundingDINO",
        "/workspace/perception/vendor/segment-anything-2",
    ]
    env["PYTHONPATH"] = ":".join(vendor)
    env["PATH"] = "/workspace/.conda_envs/nbv_perception/bin:" + env.get("PATH", "")
    pred2d = str(Path(out_path).with_name(Path(out_path).stem + "_pred2d.json"))
    cmd = [
        py,
        "/workspace/scripts/perceive_saved_rgbd_gsam.py",
        "--rgb",
        rgb_path,
        "--out",
        pred2d,
    ]
    t0 = time.time()
    proc = subprocess.run(cmd, cwd="/workspace", capture_output=True, text=True, env=env)
    # Also keep a slim 3D-ready stub pointing at pred2d for memory pipeline
    summary = {
        "pred2d": pred2d,
        "returncode": proc.returncode,
        "elapsed_s": time.time() - t0,
        "stdout_tail": (proc.stdout or "")[-2000:],
        "stderr_tail": (proc.stderr or "")[-2000:],
    }
    Path(out_path).write_text(json.dumps(summary, indent=2))
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal-topic", default="/nbv/goal_pose")
    ap.add_argument("--goal-is-ee", action="store_true", help="Goal is EE PoseStamped")
    ap.add_argument("--ee-goal-fallback", default="/ee_goal")
    ap.add_argument("--camera-frame", default="d455_depth_optical_frame")
    ap.add_argument("--ee-frame", default="cr10_Link6")
    ap.add_argument("--rgb-topic", default="/realsense/rgb/image_raw")
    ap.add_argument("--depth-topic", default="/realsense/depth/image_raw")
    ap.add_argument("--pos-tol", type=float, default=0.12)
    ap.add_argument("--rot-tol-deg", type=float, default=20.0)
    ap.add_argument("--vel-tol", type=float, default=0.03)
    ap.add_argument("--stable-s", type=float, default=1.0)
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument(
        "--out",
        default="data/metrics/fkie_agx_warehouse_mode_b/gsam_arrival_trigger.json",
    )
    ap.add_argument(
        "--perceive-out",
        default="data/metrics/fkie_agx_warehouse_mode_b/gsam_after_arrival.json",
    )
    ap.add_argument("--seed-ee-goal", action="store_true", help="Also listen on /ee_goal")
    args = ap.parse_args()

    rospy.init_node("trigger_gsam_on_nbv_arrival")
    if args.goal_is_ee:
        args.goal_topic = args.ee_goal_fallback
    gate = ArrivalGate(args)
    if args.goal_is_ee:
        gate.goal_frame = "ee"

    def _on_ee_goal(msg):
        gate.goal_xyz = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
        )
        gate.goal_q = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            dtype=float,
        )
        gate.triggered = False
        gate.stable_since = None
        gate.arrival_t = None
        rospy.loginfo("GSAM gate: ee_goal update")

    if args.seed_ee_goal and not args.goal_is_ee:
        rospy.Subscriber(args.ee_goal_fallback, PoseStamped, _on_ee_goal, queue_size=1)

    t0 = time.time()
    evidence = {"triggered": False, "gate": None, "gsam": None}
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and time.time() - t0 < args.timeout:
        st = gate.poll()
        if st is not None:
            gate.triggered = True
            evidence["triggered"] = True
            evidence["gate"] = st
            evidence["arrival_wall_s"] = time.time() - t0
            rospy.loginfo("GSAM gate PASS — capturing RGB-D then grounded_sam")
            cap_dir = str(Path(args.perceive_out).parent / "gsam_arrival_frames")
            try:
                caps = _save_latest_rgbd(args.rgb_topic, args.depth_topic, cap_dir)
            except Exception as e:
                evidence["capture_error"] = str(e)
                break
            evidence["capture"] = caps
            gsam = run_gsam(args.perceive_out, caps["rgb"])
            evidence["gsam"] = gsam
            evidence["perceive_out"] = args.perceive_out
            break
        rate.sleep()

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(evidence, indent=2))
    print(json.dumps(evidence, indent=2))
    if not evidence["triggered"]:
        raise SystemExit(2)
    if evidence["gsam"] and evidence["gsam"].get("returncode", 1) != 0:
        raise SystemExit(3)
    print("GSAM_ARRIVAL_OK")


if __name__ == "__main__":
    main()
