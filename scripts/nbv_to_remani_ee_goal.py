#!/usr/bin/env python3
"""Convert FKIE camera NBV goal to REMANI /ee_goal (PoseStamped, world EE).

Uses live TF: T_world_ee = T_world_camera * inv(T_ee_camera)
where T_ee_camera is looked up as transform ee_frame <- camera_frame.

Default ee_frame=cr10_Link6 (CR10 flange). Does NOT teleport the robot.
Publishes /ee_goal only with --publish; otherwise dry-run JSON.
"""
from __future__ import print_function
import argparse
import json
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft


def pose_to_matrix(p, q):
    T = tft.quaternion_matrix([q[0], q[1], q[2], q[3]])
    T[0:3, 3] = p
    return T


def matrix_to_pose(T):
    q = tft.quaternion_from_matrix(T)
    p = T[0:3, 3]
    return p, q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam-xyz", nargs=3, type=float, required=True)
    ap.add_argument("--cam-rpy", nargs=3, type=float, default=[0, 0.35, 0])
    ap.add_argument("--camera-frame", default="d455_color_optical_frame")
    ap.add_argument("--ee-frame", default="cr10_Link6")
    ap.add_argument("--world-frame", default="world")
    ap.add_argument("--publish", action="store_true")
    ap.add_argument("--out", default="data/metrics/nbv_remani/ee_goal.json")
    args = ap.parse_args()

    rospy.init_node("nbv_to_remani_ee_goal")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    # Camera goal orientation from RPY (world)
    q_cam = tft.quaternion_from_euler(*args.cam_rpy)
    T_w_c_goal = pose_to_matrix(args.cam_xyz, q_cam)

    # Extrinsic: transform of camera in EE frame = lookup ee <- camera at current state
    # For static Mode A this equals fixed URDF extrinsic.
    try:
        tfm = buf.lookup_transform(args.ee_frame, args.camera_frame, rospy.Time(0), rospy.Duration(2.0))
    except Exception as e:
        raise SystemExit("TF lookup %s <- %s failed: %s" % (args.ee_frame, args.camera_frame, e))
    t = tfm.transform.translation
    r = tfm.transform.rotation
    T_ee_c = pose_to_matrix([t.x, t.y, t.z], [r.x, r.y, r.z, r.w])
    T_c_ee = np.linalg.inv(T_ee_c)
    T_w_ee = T_w_c_goal.dot(T_c_ee)
    p, q = matrix_to_pose(T_w_ee)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = args.world_frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(p[0]), float(p[1]), float(p[2])
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = [
        float(q[0]),
        float(q[1]),
        float(q[2]),
        float(q[3]),
    ]

    out = {
        "camera_goal_xyz": list(args.cam_xyz),
        "camera_goal_rpy": list(args.cam_rpy),
        "ee_frame": args.ee_frame,
        "camera_frame": args.camera_frame,
        "ee_goal_xyz": [float(p[0]), float(p[1]), float(p[2])],
        "ee_goal_quat_xyzw": [float(q[0]), float(q[1]), float(q[2]), float(q[3])],
        "published": bool(args.publish),
        "note": "Planning execution requires REMANI node accepting /ee_goal; Mode A cannot execute motion.",
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))

    if args.publish:
        pub = rospy.Publisher("/ee_goal", PoseStamped, queue_size=1, latch=True)
        rospy.sleep(0.5)
        pub.publish(msg)
        print("Published /ee_goal")
    print("NBV_TO_REMANI_EE_GOAL_OK")


if __name__ == "__main__":
    main()
