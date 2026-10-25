#!/usr/bin/env python3
"""Formal FKIE NbvPlanner Action client — saves structured JSON (no regex from logs).

Fails closed on timeout, non-SUCCEEDED, empty goals[], or missing pose fields.
"""
from __future__ import print_function

import argparse
import json
import sys
from pathlib import Path

import actionlib
import rospy
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from std_msgs.msg import Header

from fkie_measurement_msgs.msg import BoundaryPolygon, MeasurementEstimation
from fkie_nbv_planner.msg import NbvPlannerAction, NbvPlannerGoal


def pose_stamped_to_dict(ps):
    if ps is None:
        return None
    return {
        "header": {
            "frame_id": ps.header.frame_id,
            "stamp": ps.header.stamp.to_sec() if ps.header.stamp else None,
        },
        "pose": {
            "position": {
                "x": float(ps.pose.position.x),
                "y": float(ps.pose.position.y),
                "z": float(ps.pose.position.z),
            },
            "orientation": {
                "x": float(ps.pose.orientation.x),
                "y": float(ps.pose.orientation.y),
                "z": float(ps.pose.orientation.z),
                "w": float(ps.pose.orientation.w),
            },
        },
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/metrics/fkie_agx_warehouse_mode_b/fkie_action_result.json")
    ap.add_argument("--action-server", default="/robot/nbv_rrt")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument(
        "--boundary-xy",
        default="[[-8.3603,-2.6852],[-1.80,-2.6852],[-1.80,-8.6852],[-8.3603,-8.6852]]",
        help="JSON list of [x,y] polygon vertices",
    )
    ap.add_argument("--allow-empty", action="store_true", help="Do not fail if goals=[]")
    args = ap.parse_args()

    rospy.init_node("save_fkie_action_result", anonymous=True)
    boundary_xy = json.loads(args.boundary_xy)

    client = actionlib.SimpleActionClient(args.action_server, NbvPlannerAction)
    rospy.loginfo("Waiting for %s", args.action_server)
    if not client.wait_for_server(rospy.Duration(args.timeout)):
        raise SystemExit("FKIE_ACTION_FAIL: server not available")

    goal = NbvPlannerGoal()
    goal.header = Header(stamp=rospy.Time.now(), frame_id="world")
    goal.arm_in_prepacked = True

    poly = PolygonStamped()
    poly.header = goal.header
    for x, y in boundary_xy:
        poly.polygon.points.append(Point32(x=float(x), y=float(y), z=0.0))
    boundary = BoundaryPolygon()
    boundary.id = 1
    boundary.source_type = "save_fkie_action_result"
    boundary.min_height = 0.0
    boundary.max_height = 2.0
    boundary.polygon = poly
    goal.current_boundary = boundary

    est = MeasurementEstimation()
    est.header = goal.header
    est.grid_size = 1.0
    goal.estimations = est

    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(args.timeout))
    if not finished:
        client.cancel_goal()
        raise SystemExit("FKIE_ACTION_FAIL: timeout")

    state = int(client.get_state())
    result = client.get_result()
    state_name = {
        0: "PENDING",
        1: "ACTIVE",
        2: "PREEMPTED",
        3: "SUCCEEDED",
        4: "ABORTED",
        5: "REJECTED",
        6: "PREEMPTING",
        7: "RECALLING",
        8: "RECALLED",
        9: "LOST",
    }.get(state, str(state))

    goals = []
    for i, g in enumerate(result.goals or []):
        d = pose_stamped_to_dict(g)
        d["index"] = i
        goals.append(d)

    out = {
        "action_server": args.action_server,
        "action_state": state,
        "action_state_name": state_name,
        "complete_exploration": bool(result.complete_exploration),
        "request_base_pose": bool(result.request_base_pose),
        "explored_boundary_id": int(getattr(result, "explored_boundary_id", -1) or -1),
        "goal_pose_3d": pose_stamped_to_dict(result.goal_pose_3d),
        "goals": goals,
        "n_goals": len(goals),
        "header_frame_id": goal.header.frame_id,
        "wall_time": rospy.Time.now().to_sec(),
    }

    # Convenience fields for nbv_to_remani_ee_goal.load_fkie_goal
    primary = None
    if goals:
        primary = goals[0]
    elif out["goal_pose_3d"] and out["goal_pose_3d"]["pose"]:
        primary = out["goal_pose_3d"]
    if primary:
        p = primary["pose"]["position"]
        o = primary["pose"]["orientation"]
        out["camera_goal_xyz"] = [p["x"], p["y"], p["z"]]
        out["camera_goal_quat_xyzw"] = [o["x"], o["y"], o["z"], o["w"]]
        out["orientation_source"] = "fkie_action_quaternion"
        out["selected_goal_index"] = primary.get("index", 0)

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps({k: out[k] for k in out if k != "goals"}, indent=2))
    print("n_goals", out["n_goals"])

    if state != 3:
        print("FKIE_ACTION_FAIL: state=%s" % state_name)
        raise SystemExit(4)
    if out["n_goals"] == 0 and not args.allow_empty:
        print("FKIE_ACTION_FAIL: goals=[] — refuse to invent a substitute target")
        raise SystemExit(5)
    print("FKIE_ACTION_RESULT_OK")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(1)
