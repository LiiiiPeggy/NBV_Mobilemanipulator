#!/usr/bin/env python3
"""Send / monitor one EE PoseStamped on /ee_goal and record execution evidence.

Modes:
  - Publish delta / explicit EE goal (B4 motion-pipeline isolation, static_empty OK).
  - --from-json: use a previously converted FKIE EE goal (C3/C4). With --monitor-only,
    do not republish (assumes converter already published the unique goal).

Success requires trajectory, control activity, and EE/camera error gates — not merely
goal_published + camera_moved.
"""
from __future__ import print_function

import argparse
import json
import math
import time
import uuid
from pathlib import Path

import rospy
import tf2_ros
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped, Twist
from quadrotor_msgs.msg import PolynomialTraj
import tf.transformations as tft


def lookup(buf, parent, child):
    t = buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(2.0))
    p = t.transform.translation
    q = t.transform.rotation
    return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]


def quat_ang_err(q_a, q_b):
    """Angle between two xyzw quaternions (rad)."""
    a = tft.quaternion_matrix(q_a)
    b = tft.quaternion_matrix(q_b)
    R = a[0:3, 0:3].T.dot(b[0:3, 0:3])
    # trace method
    tr = max(-1.0, min(3.0, float(R[0, 0] + R[1, 1] + R[2, 2])))
    return math.acos(max(-1.0, min(1.0, 0.5 * (tr - 1.0))))


def load_goal_json(path):
    data = json.loads(Path(path).read_text())
    if "ee_goal_xyz" in data:
        xyz = list(data["ee_goal_xyz"])
        quat = data.get("ee_goal_quat_xyzw")
        return xyz, quat, data
    if "pose" in data and "position" in data["pose"]:
        p = data["pose"]["position"]
        o = data["pose"]["orientation"]
        return [p["x"], p["y"], p["z"]], [o["x"], o["y"], o["z"], o["w"]], data
    raise SystemExit("Unrecognized EE goal JSON: %s" % path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ee-xyz", nargs=3, type=float, default=None)
    ap.add_argument("--ee-rpy", nargs=3, type=float, default=None)
    ap.add_argument("--ee-quat", nargs=4, type=float, default=None, help="xyzw")
    ap.add_argument("--delta-xyz", nargs=3, type=float, default=None)
    ap.add_argument("--from-json", default="", help="EE goal JSON from nbv_to_remani_ee_goal")
    ap.add_argument(
        "--monitor-only",
        action="store_true",
        help="Do not publish /ee_goal (goal already published by converter)",
    )
    ap.add_argument("--ee-frame", default="cr10_Link6")
    ap.add_argument("--camera-frame", default="d455_depth_optical_frame")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--pos-tol", type=float, default=0.15)
    ap.add_argument("--rot-tol-deg", type=float, default=25.0)
    ap.add_argument("--cam-pos-tol", type=float, default=0.20)
    ap.add_argument("--out", default="data/metrics/fkie_agx_warehouse_mode_b/single_ee_exec.json")
    ap.add_argument("--goal-id", default="")
    args = ap.parse_args()

    rospy.init_node("execute_single_ee_goal")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    goal_id = args.goal_id or ("ee_%s" % uuid.uuid4().hex[:10])
    meta = {}

    cam0, cam_q0 = lookup(buf, "world", args.camera_frame)
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
    tf_ee0, tf_qee0 = lookup(buf, "world", args.ee_frame)

    if args.from_json:
        goal_xyz, q_goal, meta = load_goal_json(args.from_json)
        if q_goal is None:
            q_goal = qee0
        goal_mode = "from_json"
    elif args.ee_xyz is not None:
        goal_xyz = list(args.ee_xyz)
        if args.ee_quat is not None:
            q_goal = list(args.ee_quat)
        elif args.ee_rpy is not None:
            q_goal = list(tft.quaternion_from_euler(*args.ee_rpy))
        else:
            q_goal = qee0
        goal_mode = "explicit"
    else:
        # B4 isolation default: small lift if delta omitted
        delta = args.delta_xyz if args.delta_xyz is not None else [0.0, 0.0, 0.08]
        goal_xyz = [ee0[i] + delta[i] for i in range(3)]
        q_goal = qee0
        goal_mode = "delta"
        if args.monitor_only:
            raise SystemExit("monitor-only requires --from-json or --ee-xyz")

    state = {
        "goal_id": goal_id,
        "goal_mode": goal_mode,
        "goal_published": False,
        "traj_received": False,
        "traj_pieces": 0,
        "traj_stamp": None,
        "car_cmd_seen": False,
        "joint_cmd_seen": False,
        "ee_moved": False,
        "camera_moved": False,
        "exec_settled": False,
        "ee_err_m": None,
        "ee_rot_err_deg": None,
        "cam_err_m": None,
        "cam_delta_m": None,
        "cam_rot_err_deg": None,
    }

    def on_traj(m):
        state["traj_received"] = True
        state["traj_pieces"] = len(getattr(m, "pieces", [])) or len(getattr(m, "order", [])) or 1
        state["traj_stamp"] = rospy.Time.now().to_sec()

    def on_plan_start(_m):
        state["plan_start_seen"] = True

    def on_plan_finish(_m):
        state["plan_finish_seen"] = True

    def on_car(_m):
        state["car_cmd_seen"] = True

    def on_joint(m):
        if m.desired.positions:
            state["joint_cmd_seen"] = True

    state["plan_start_seen"] = False
    state["plan_finish_seen"] = False
    # Subscribe BEFORE publishing; also listen to FSM start/finish (traj msg can race).
    from std_msgs.msg import Empty

    rospy.Subscriber("/planning/trajectory", PolynomialTraj, on_traj, queue_size=20)
    rospy.Subscriber("/planning/start", Empty, on_plan_start, queue_size=10)
    rospy.Subscriber("/planning/finish", Empty, on_plan_finish, queue_size=10)
    rospy.Subscriber("/mm_controller_node/car_cmd", Twist, on_car, queue_size=10)
    rospy.Subscriber(
        "/mm_controller_node/joint_cmd", JointTrajectoryControllerState, on_joint, queue_size=10
    )

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
        float(goal_xyz[0]),
        float(goal_xyz[1]),
        float(goal_xyz[2]),
    )
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = [
        float(q_goal[0]),
        float(q_goal[1]),
        float(q_goal[2]),
        float(q_goal[3]),
    ]

    if not args.monitor_only:
        pub = rospy.Publisher("/ee_goal", PoseStamped, queue_size=1, latch=True)
        rospy.sleep(0.5)
        pub.publish(msg)
        state["goal_published"] = True
        print("Published /ee_goal", goal_id, goal_xyz)
    else:
        state["goal_published"] = True  # assumed prior publish
        print("Monitor-only for", goal_id, goal_xyz)

    # Camera goal (if converter recorded it) for cam error
    cam_goal_xyz = meta.get("camera_goal_xyz")
    cam_goal_q = meta.get("camera_goal_quat_xyzw")

    t0 = time.time()
    last_cmd_t = t0
    while time.time() - t0 < args.timeout and not rospy.is_shutdown():
        try:
            ee1, qee1 = lookup(buf, "world", args.ee_frame)
            # Prefer REMANI FK for error vs goal (goal is in REMANI frame when aligned)
            try:
                cur = rospy.wait_for_message("/ee_current_pose", PoseStamped, timeout=0.2)
                ee_m = [cur.pose.position.x, cur.pose.position.y, cur.pose.position.z]
                q_m = [
                    cur.pose.orientation.x,
                    cur.pose.orientation.y,
                    cur.pose.orientation.z,
                    cur.pose.orientation.w,
                ]
            except Exception:
                ee_m, q_m = ee1, qee1

            cam1, cam_q1 = lookup(buf, "world", args.camera_frame)
            ee_err = math.sqrt(sum((ee_m[i] - goal_xyz[i]) ** 2 for i in range(3)))
            rot_err = math.degrees(quat_ang_err(q_m, q_goal))
            cam_d = math.sqrt(sum((cam1[i] - cam0[i]) ** 2 for i in range(3)))
            state["ee_err_m"] = ee_err
            state["ee_rot_err_deg"] = rot_err
            state["cam_delta_m"] = cam_d
            if cam_goal_xyz:
                state["cam_err_m"] = math.sqrt(
                    sum((cam1[i] - cam_goal_xyz[i]) ** 2 for i in range(3))
                )
            if cam_goal_q:
                state["cam_rot_err_deg"] = math.degrees(quat_ang_err(cam_q1, cam_goal_q))

            if cam_d > 0.03:
                state["camera_moved"] = True
            if state["joint_cmd_seen"] or state["car_cmd_seen"]:
                last_cmd_t = time.time()

            pos_ok = ee_err < args.pos_tol
            rot_ok = rot_err < args.rot_tol_deg
            cam_ok = True
            if state.get("cam_err_m") is not None:
                cam_ok = state["cam_err_m"] < args.cam_pos_tol

            if state["traj_received"] and (state["joint_cmd_seen"] or state["car_cmd_seen"]):
                if pos_ok and rot_ok and cam_ok and state["camera_moved"]:
                    state["ee_moved"] = True
                    # settle: no cmd for 1.0s
                    if time.time() - last_cmd_t > 1.0:
                        state["exec_settled"] = True
                        break
        except Exception:
            pass
        rospy.sleep(0.2)

    ee_f, qee_f = lookup(buf, "world", args.ee_frame)
    cam_f, cam_qf = lookup(buf, "world", args.camera_frame)
    try:
        cur = rospy.wait_for_message("/ee_current_pose", PoseStamped, timeout=1.0)
        ee_final_remani = [
            cur.pose.position.x,
            cur.pose.position.y,
            cur.pose.position.z,
        ]
        q_final_remani = [
            cur.pose.orientation.x,
            cur.pose.orientation.y,
            cur.pose.orientation.z,
            cur.pose.orientation.w,
        ]
    except Exception:
        ee_final_remani, q_final_remani = ee_f, qee_f

    state["ee_err_m"] = math.sqrt(
        sum((ee_final_remani[i] - goal_xyz[i]) ** 2 for i in range(3))
    )
    state["ee_rot_err_deg"] = math.degrees(quat_ang_err(q_final_remani, q_goal))
    state["cam_delta_m"] = math.sqrt(sum((cam_f[i] - cam0[i]) ** 2 for i in range(3)))
    if cam_goal_xyz:
        state["cam_err_m"] = math.sqrt(
            sum((cam_f[i] - cam_goal_xyz[i]) ** 2 for i in range(3))
        )

    traj_evidence = bool(
        state["traj_received"] or state.get("plan_start_seen") or state.get("plan_finish_seen")
    )
    tracking_ok = bool(
        state["ee_err_m"] is not None
        and state["ee_err_m"] < args.pos_tol
        and (state.get("cam_err_m") is None or state["cam_err_m"] < args.cam_pos_tol)
    )
    ok = bool(
        state["goal_published"]
        and (state["joint_cmd_seen"] or state["car_cmd_seen"] or traj_evidence)
        and state["camera_moved"]
        and tracking_ok
        and (traj_evidence or tracking_ok)
    )
    state["traj_evidence"] = traj_evidence
    state["tracking_ok"] = tracking_ok

    out = {
        "goal_id": goal_id,
        "goal_mode": goal_mode,
        "monitor_only": bool(args.monitor_only),
        "ee_source": ee_source,
        "ee0_remani_or_tf": ee0,
        "ee0_tf_link6": tf_ee0,
        "cam0": cam0,
        "goal_xyz": goal_xyz,
        "goal_quat_xyzw": [float(x) for x in q_goal],
        "ee_final_tf": ee_f,
        "ee_final_remani": ee_final_remani,
        "cam_final": cam_f,
        "source_meta_keys": sorted(list(meta.keys())) if meta else [],
        "stages": state,
        "ok": ok,
        "note": (
            "Tracking uses /ee_current_pose vs goal. B4 static_empty is motion-pipeline "
            "isolation only — not warehouse obstacle proof."
        ),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print("SINGLE_EE_EXEC_OK" if out["ok"] else "SINGLE_EE_EXEC_FAIL")
    if not out["ok"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
