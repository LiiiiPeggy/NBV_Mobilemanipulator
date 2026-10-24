#!/usr/bin/env python3
"""Convert FKIE camera NBV goal to REMANI /ee_goal (PoseStamped, world EE).

T_world_ee = T_world_camera * inv(T_ee_camera)

Mode B note: REMANI internal EE FK differs from URDF cr10_Link6 by ~base_z.
Use --align-remani-ee to shift the goal into /ee_current_pose frame before publish.
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


def load_fkie_goal(path):
    data = json.loads(Path(path).read_text())
    # Accept several evidence JSON shapes from verify_fkie_action / probes.
    if "goals" in data and data["goals"]:
        g = data["goals"][0]
        if "pose" in g:
            p = g["pose"]["position"]
            o = g["pose"]["orientation"]
            return [p["x"], p["y"], p["z"]], [o["x"], o["y"], o["z"], o["w"]], "goals[0].pose"
        if "xyz" in g:
            xyz = g["xyz"]
            rpy = g.get("rpy")
            if rpy:
                return xyz, list(tft.quaternion_from_euler(*rpy)), "goals[0].xyz+rpy"
            return xyz, None, "goals[0].xyz"
    if "goal_pose_3d" in data:
        p = data["goal_pose_3d"]["position"]
        o = data["goal_pose_3d"]["orientation"]
        return [p["x"], p["y"], p["z"]], [o["x"], o["y"], o["z"], o["w"]], "goal_pose_3d"
    if "camera_goal_xyz" in data:
        xyz = data["camera_goal_xyz"]
        if "camera_goal_quat_xyzw" in data:
            return xyz, data["camera_goal_quat_xyzw"], "camera_goal_*"
        rpy = data.get("camera_goal_rpy", [0, 0.35, 0])
        return xyz, list(tft.quaternion_from_euler(*rpy)), "camera_goal_xyz+rpy"
    raise SystemExit("Unrecognized FKIE goal JSON: %s" % path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam-xyz", nargs=3, type=float, default=None)
    ap.add_argument("--cam-rpy", nargs=3, type=float, default=None)
    ap.add_argument("--cam-quat", nargs=4, type=float, default=None, help="xyzw")
    ap.add_argument("--from-fkie-json", default="", help="Load camera goal from FKIE evidence JSON")
    ap.add_argument("--camera-frame", default="d455_depth_optical_frame")
    ap.add_argument("--ee-frame", default="cr10_Link6")
    ap.add_argument("--world-frame", default="world")
    ap.add_argument(
        "--align-remani-ee",
        action="store_true",
        help="Subtract (TF_ee - /ee_current_pose) so goal matches REMANI FK frame",
    )
    ap.add_argument("--publish", action="store_true")
    ap.add_argument("--out", default="data/metrics/nbv_remani/ee_goal.json")
    args = ap.parse_args()

    rospy.init_node("nbv_to_remani_ee_goal")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    orientation_source = "default_rpy"
    if args.from_fkie_json:
        cam_xyz, cam_q, orientation_source = load_fkie_goal(args.from_fkie_json)
        if cam_q is None:
            if args.cam_rpy is None:
                raise SystemExit("FKIE JSON has position only; pass --cam-rpy or --cam-quat (adaptation)")
            cam_q = list(tft.quaternion_from_euler(*args.cam_rpy))
            orientation_source = "fkie_xyz+cli_rpy_adaptation"
    else:
        if args.cam_xyz is None:
            raise SystemExit("Need --cam-xyz or --from-fkie-json")
        cam_xyz = list(args.cam_xyz)
        if args.cam_quat is not None:
            cam_q = list(args.cam_quat)
            orientation_source = "cli_quat"
        else:
            rpy = args.cam_rpy if args.cam_rpy is not None else [0.0, 0.35, 0.0]
            cam_q = list(tft.quaternion_from_euler(*rpy))
            orientation_source = "cli_rpy" if args.cam_rpy is not None else "default_rpy_0_0.35_0"

    T_w_c_goal = pose_to_matrix(cam_xyz, cam_q)

    try:
        tfm = buf.lookup_transform(args.ee_frame, args.camera_frame, rospy.Time(0), rospy.Duration(2.0))
    except Exception as e:
        raise SystemExit("TF lookup %s <- %s failed: %s" % (args.ee_frame, args.camera_frame, e))
    t = tfm.transform.translation
    r = tfm.transform.rotation
    T_ee_c = pose_to_matrix([t.x, t.y, t.z], [r.x, r.y, r.z, r.w])
    T_w_ee = T_w_c_goal.dot(np.linalg.inv(T_ee_c))
    p, q = matrix_to_pose(T_w_ee)

    align = {"applied": False}
    if args.align_remani_ee:
        try:
            rem = rospy.wait_for_message("/ee_current_pose", PoseStamped, timeout=3.0)
            tf_ee = buf.lookup_transform(args.world_frame, args.ee_frame, rospy.Time(0), rospy.Duration(2.0))
            dx = rem.pose.position.x - tf_ee.transform.translation.x
            dy = rem.pose.position.y - tf_ee.transform.translation.y
            dz = rem.pose.position.z - tf_ee.transform.translation.z
            p = [p[0] + dx, p[1] + dy, p[2] + dz]
            align = {"applied": True, "delta_xyz": [dx, dy, dz]}
        except Exception as e:
            align = {"applied": False, "error": str(e)}

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
        "camera_goal_xyz": cam_xyz,
        "orientation_source": orientation_source,
        "ee_frame": args.ee_frame,
        "camera_frame": args.camera_frame,
        "ee_goal_xyz": [float(p[0]), float(p[1]), float(p[2])],
        "ee_goal_quat_xyzw": [float(q[0]), float(q[1]), float(q[2]), float(q[3])],
        "align_remani_ee": align,
        "published": bool(args.publish),
        "note": "S5-A: prefer FKIE orientation; --align-remani-ee for Mode B FK offset",
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
