#!/usr/bin/env python3
"""Send one EE PoseStamped to /ee_goal and record planning/execution evidence.

Stages reported separately:
  goal_published, traj_received, car_cmd_seen, joint_cmd_seen, ee_moved, camera_moved
"""
from __future__ import print_function
import argparse
import json
import math
import time
from pathlib import Path

import rospy
import tf2_ros
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PolynomialTraj
import tf.transformations as tft


def lookup(buf, parent, child):
    t = buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(2.0))
    p = t.transform.translation
    q = t.transform.rotation
    return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ee-xyz", nargs=3, type=float, default=None)
    ap.add_argument("--ee-rpy", nargs=3, type=float, default=None)
    ap.add_argument("--delta-xyz", nargs=3, type=float, default=[0.0, 0.0, 0.08])
    ap.add_argument("--ee-frame", default="cr10_Link6")
    ap.add_argument("--camera-frame", default="d455_depth_optical_frame")
    ap.add_argument("--timeout", type=float, default=45.0)
    ap.add_argument("--out", default="data/metrics/fkie_agx_warehouse_mode_b/single_ee_exec.json")
    args = ap.parse_args()

    rospy.init_node("execute_single_ee_goal")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    cam0, _ = lookup(buf, "world", args.camera_frame)
    # Prefer REMANI's own FK (/ee_current_pose): URDF Link6 can differ by ~base_z.
    ee0 = None
    qee0 = None
    try:
        cur = rospy.wait_for_message("/ee_current_pose", PoseStamped, timeout=3.0)
        ee0 = [cur.pose.position.x, cur.pose.position.y, cur.pose.position.z]
        qee0 = [
            cur.pose.orientation.x,
            cur.pose.orientation.y,
            cur.pose.orientation.z,
            cur.pose.orientation.w,
        ]
        ee_source = "ee_current_pose"
    except Exception:
        ee0, qee0 = lookup(buf, "world", args.ee_frame)
        ee_source = "tf:" + args.ee_frame
    tf_ee0, _ = lookup(buf, "world", args.ee_frame)

    if args.ee_xyz is not None:
        goal_xyz = list(args.ee_xyz)
    else:
        goal_xyz = [ee0[i] + args.delta_xyz[i] for i in range(3)]

    if args.ee_rpy is not None:
        q_goal = tft.quaternion_from_euler(*args.ee_rpy)
    else:
        q_goal = qee0

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal_xyz
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q_goal

    state = {
        "goal_published": False,
        "traj_received": False,
        "traj_pieces": 0,
        "car_cmd_seen": False,
        "joint_cmd_seen": False,
        "ee_moved": False,
        "camera_moved": False,
        "ee_err_m": None,
        "cam_delta_m": None,
    }

    def on_traj(m):
        state["traj_received"] = True
        state["traj_pieces"] = len(getattr(m, "pieces", [])) or len(getattr(m, "order", [])) or 1

    def on_car(m):
        state["car_cmd_seen"] = True

    def on_joint(m):
        if m.desired.positions:
            state["joint_cmd_seen"] = True

    rospy.Subscriber("/planning/trajectory", PolynomialTraj, on_traj, queue_size=10)
    rospy.Subscriber("/mm_controller_node/car_cmd", Twist, on_car, queue_size=10)
    rospy.Subscriber("/mm_controller_node/joint_cmd", JointTrajectoryControllerState, on_joint, queue_size=10)

    pub = rospy.Publisher("/ee_goal", PoseStamped, queue_size=1, latch=True)
    rospy.sleep(0.5)
    pub.publish(msg)
    state["goal_published"] = True
    print("Published /ee_goal", goal_xyz)

    t0 = time.time()
    while time.time() - t0 < args.timeout and not rospy.is_shutdown():
        try:
            ee1, _ = lookup(buf, "world", args.ee_frame)
            cam1, _ = lookup(buf, "world", args.camera_frame)
            ee_err = math.sqrt(sum((ee1[i] - goal_xyz[i]) ** 2 for i in range(3)))
            cam_d = math.sqrt(sum((cam1[i] - cam0[i]) ** 2 for i in range(3)))
            state["ee_err_m"] = ee_err
            state["cam_delta_m"] = cam_d
            if cam_d > 0.03:
                state["camera_moved"] = True
            if ee_err < 0.12 and cam_d > 0.02:
                state["ee_moved"] = True
                break
            if state["traj_received"] and state["joint_cmd_seen"] and cam_d > 0.05:
                state["ee_moved"] = True
        except Exception:
            pass
        rospy.sleep(0.2)

    ee_f, _ = lookup(buf, "world", args.ee_frame)
    cam_f, _ = lookup(buf, "world", args.camera_frame)
    out = {
        "ee_source": ee_source,
        "ee0_remani_or_tf": ee0,
        "ee0_tf_link6": tf_ee0,
        "cam0": cam0,
        "goal_xyz": goal_xyz,
        "ee_final_tf": ee_f,
        "cam_final": cam_f,
        "stages": state,
        "ok": bool(
            state["goal_published"]
            and state["traj_received"]
            and (state["joint_cmd_seen"] or state["car_cmd_seen"])
            and state["camera_moved"]
        ),
        "note": "Goals should be in REMANI EE frame (/ee_current_pose); TF Link6 may differ ~base_z",
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print("SINGLE_EE_EXEC_OK" if out["ok"] else "SINGLE_EE_EXEC_FAIL")
    if not out["ok"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
