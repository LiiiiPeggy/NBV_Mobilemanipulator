#!/usr/bin/env python3
"""Mode B B1 check: upright model, joint hold, TF vs Gazebo, small joint move."""
from __future__ import print_function
import argparse
import json
import math
import time
from pathlib import Path

import rospy
import tf2_ros
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import JointState


ARM = [
    "cr10_joint1",
    "cr10_joint2",
    "cr10_joint3",
    "cr10_joint4",
    "cr10_joint5",
    "cr10_joint6",
]


def rpy_from_quat(q):
    sinr = 2 * (q.w * q.x + q.y * q.z)
    cosr = 1 - 2 * (q.x * q.x + q.y * q.y)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    roll = math.atan2(sinr, cosr)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="agx_ranger_cr10")
    ap.add_argument("--hold-s", type=float, default=3.0)
    ap.add_argument("--move-joint", default="cr10_joint5")
    ap.add_argument("--move-delta", type=float, default=0.15)
    ap.add_argument("--out", default="data/metrics/fkie_agx_warehouse_mode_b/stability_b1.json")
    ap.add_argument("--skip-move", action="store_true")
    args = ap.parse_args()

    rospy.init_node("check_agx_dynamic_stability", anonymous=True)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=30)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    cmd_pub = rospy.Publisher(
        "/mm_controller_node/joint_cmd", JointTrajectoryControllerState, queue_size=1
    )
    rospy.sleep(0.5)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    js0 = rospy.wait_for_message("/joint_states", JointState, timeout=20)
    js_map0 = dict(zip(js0.name, js0.position))
    st0 = get_state(args.model, "world")
    r0, p0, y0 = rpy_from_quat(st0.pose.orientation)
    nan0 = any(math.isnan(v) for v in js0.velocity) if js0.velocity else False
    time.sleep(args.hold_s)
    js1 = rospy.wait_for_message("/joint_states", JointState, timeout=10)
    js_map1 = dict(zip(js1.name, js1.position))
    st1 = get_state(args.model, "world")
    r1, p1, y1 = rpy_from_quat(st1.pose.orientation)

    arm_drift = {
        j: abs(float(js_map1.get(j, 0)) - float(js_map0.get(j, 0))) for j in ARM if j in js_map0
    }
    base_drift = math.hypot(
        st1.pose.position.x - st0.pose.position.x, st1.pose.position.y - st0.pose.position.y
    )

    tfm = buf.lookup_transform("world", "base_link", rospy.Time(0), rospy.Duration(2.0))
    tf_err = math.hypot(
        tfm.transform.translation.x - st1.pose.position.x,
        tfm.transform.translation.y - st1.pose.position.y,
    )
    cam_tf = None
    try:
        c = buf.lookup_transform("world", "d455_depth_optical_frame", rospy.Time(0), rospy.Duration(2.0))
        cam_tf = [
            c.transform.translation.x,
            c.transform.translation.y,
            c.transform.translation.z,
        ]
    except Exception as e:
        cam_tf = ["err", str(e)]

    move = {"skipped": True}
    if not args.skip_move and args.move_joint in js_map1:
        q_before = float(js_map1[args.move_joint])
        q_cmd = q_before + float(args.move_delta)

        def publish_arm(positions):
            msg = JointTrajectoryControllerState()
            msg.header.stamp = rospy.Time.now()
            msg.desired.positions = [float(positions[j]) for j in ARM]
            cmd_pub.publish(msg)

        pos = {j: float(js_map1.get(j, 0.0)) for j in ARM}
        pos[args.move_joint] = q_cmd
        t_end = time.time() + 2.0
        while time.time() < t_end and not rospy.is_shutdown():
            publish_arm(pos)
            rospy.sleep(0.05)
        js2 = rospy.wait_for_message("/joint_states", JointState, timeout=5)
        js_map2 = dict(zip(js2.name, js2.position))
        q_after = float(js_map2.get(args.move_joint, q_before))
        pos[args.move_joint] = q_before
        t_end = time.time() + 1.5
        while time.time() < t_end and not rospy.is_shutdown():
            publish_arm(pos)
            rospy.sleep(0.05)
        move = {
            "skipped": False,
            "joint": args.move_joint,
            "q_before": q_before,
            "q_cmd": q_cmd,
            "q_after": q_after,
            "err": abs(q_after - q_cmd),
            "ok": abs(q_after - q_cmd) < 0.08,
        }

    upright = abs(r1) < 0.25 and abs(p1) < 0.25
    hold_ok = (max(arm_drift.values()) < 0.15) if arm_drift else False
    tf_ok = tf_err < 0.08
    pose_ok = abs(st1.pose.position.x) + abs(st1.pose.position.y) > 0.5  # not stuck at origin
    ok = bool(upright and hold_ok and tf_ok and pose_ok and (not nan0) and move.get("ok", True))

    out = {
        "ok": ok,
        "upright": upright,
        "pose_ok": pose_ok,
        "nan_velocity_at_start": nan0,
        "roll_pitch": [r1, p1],
        "base_xy": [st1.pose.position.x, st1.pose.position.y],
        "base_z": st1.pose.position.z,
        "base_drift_m": base_drift,
        "arm_drift_rad": arm_drift,
        "max_arm_drift": max(arm_drift.values()) if arm_drift else None,
        "tf_vs_gazebo_xy_err_m": tf_err,
        "camera_optical_xyz": cam_tf,
        "move_test": move,
        "control_mode": "kinematic_set_model_configuration",
        "note": "Mode B B1: dynamic AGX + kinematic joint hold/move (not effort motors)",
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print("AGX_MODE_B_STABILITY_OK" if ok else "AGX_MODE_B_STABILITY_FAIL")
    if not ok:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
