#!/usr/bin/env python3
"""Ensure AGX Mode B base stays at aisle_safe spawn before FKIE planning.

REMANI mm_controller may keep publishing a stale car_cmd after a prior
runaway plan; the Mode B car bridge will track it and drag the chassis out of
the exploration polygon → FKIE returns complete_exploration with no goals.

This helper:
  1) Checks Gazebo base XY vs aisle spawn (or camera inside polygon).
  2) If drifted: SetModelState snap + publish aisle car_cmd so the bridge relatches.
  3) Optionally restores arm joint hold pose via set_model_configuration.

Does not invent NBV goals. Safe to call between exploration steps.
"""
from __future__ import print_function

import argparse
import json
import math
import time
from pathlib import Path

import rospy
import yaml
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, SetModelState
from geometry_msgs.msg import PoseStamped, Twist


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def inside_poly(x, y, xmin, xmax, ymin, ymax):
    return xmin <= x <= xmax and ymin <= y <= ymax


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="agx_ranger_cr10")
    ap.add_argument("--x", type=float, default=-2.10)
    ap.add_argument("--y", type=float, default=-5.89)
    ap.add_argument("--z", type=float, default=0.42)
    ap.add_argument("--yaw", type=float, default=2.833)
    ap.add_argument("--xy-tol", type=float, default=0.35)
    ap.add_argument(
        "--poly",
        default="[[-8.3603,-2.6852],[-1.80,-2.6852],[-1.80,-8.6852],[-8.3603,-8.6852]]",
        help="axis-aligned bbox from boundary polygon (min/max of verts)",
    )
    ap.add_argument("--arm-pose", default="")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--hold-sec", type=float, default=2.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    verts = json.loads(args.poly)
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)

    rospy.init_node("ensure_agx_aisle_hold", anonymous=True)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=20)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=20)
    get_s = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_s = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    pub = rospy.Publisher("/mm_controller_node/car_cmd", Twist, queue_size=1)

    st = get_s(args.model, "world")
    if not st.success:
        raise SystemExit("GAZEBO_MODEL_FAIL")
    bx, by, bz = st.pose.position.x, st.pose.position.y, st.pose.position.z
    byaw = yaw_from_quat(st.pose.orientation)
    drift = math.hypot(bx - args.x, by - args.y)

    cam_xy = None
    cam_inside = None
    try:
        cam = rospy.wait_for_message("/robot/camera_pose", PoseStamped, timeout=3.0)
        cam_xy = [cam.pose.position.x, cam.pose.position.y, cam.pose.position.z]
        cam_inside = inside_poly(cam_xy[0], cam_xy[1], xmin, xmax, ymin, ymax)
    except Exception as e:
        cam_inside = None
        cam_err = str(e)
    else:
        cam_err = None

    # Do NOT snap merely because base left the spawn XY — legitimate G_BASE_POSE
    # moves away from spawn. Only recover when camera left the exploration polygon
    # (or --force at run start after a runaway).
    need = bool(args.force) or (cam_inside is False)
    snapped = False
    if need:
        qx, qy, qz, qw = quat_from_yaw(args.yaw)
        ms = ModelState()
        ms.model_name = args.model
        ms.reference_frame = "world"
        ms.pose.position.x = args.x
        ms.pose.position.y = args.y
        ms.pose.position.z = args.z
        ms.pose.orientation.x = qx
        ms.pose.orientation.y = qy
        ms.pose.orientation.z = qz
        ms.pose.orientation.w = qw
        for _ in range(12):
            set_s(ms)
            rospy.sleep(0.03)
        if args.arm_pose:
            pose = yaml.safe_load(Path(args.arm_pose).read_text())
            rospy.wait_for_service("/gazebo/set_model_configuration", timeout=10)
            cfg = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
            names = list(pose.keys())
            cfg(args.model, "", names, [float(pose[n]) for n in names])
        snapped = True

    cmd = Twist()
    cmd.linear.x, cmd.linear.y, cmd.linear.z = args.x, args.y, args.yaw
    t0 = time.time()
    while time.time() - t0 < args.hold_sec and not rospy.is_shutdown():
        pub.publish(cmd)
        rospy.sleep(0.05)

    st2 = get_s(args.model, "world")
    out = {
        "need_snap": need,
        "snapped": snapped,
        "drift_before_m": drift,
        "before_xy": [bx, by, bz],
        "before_yaw": byaw,
        "after_xy": [st2.pose.position.x, st2.pose.position.y, st2.pose.position.z],
        "target_xy_yaw": [args.x, args.y, args.yaw],
        "cam_before": cam_xy,
        "cam_inside_before": cam_inside,
        "cam_err": cam_err,
    }
    try:
        cam2 = rospy.wait_for_message("/robot/camera_pose", PoseStamped, timeout=3.0)
        out["cam_after"] = [
            cam2.pose.position.x,
            cam2.pose.position.y,
            cam2.pose.position.z,
        ]
        out["cam_inside_after"] = inside_poly(
            cam2.pose.position.x, cam2.pose.position.y, xmin, xmax, ymin, ymax
        )
    except Exception as e:
        out["cam_after_err"] = str(e)

    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    ok = bool(out.get("cam_inside_after", True)) and math.hypot(
        st2.pose.position.x - args.x, st2.pose.position.y - args.y
    ) <= args.xy_tol
    print("AISLE_HOLD_OK" if ok else "AISLE_HOLD_FAIL")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
