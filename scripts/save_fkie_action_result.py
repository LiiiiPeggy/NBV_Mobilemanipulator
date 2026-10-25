#!/usr/bin/env python3
"""Formal FKIE NbvPlanner Action client — structured JSON + empty-goal diagnosis.

Valid action sources (fail-closed if none):
  A) result.goals[] non-empty → action_type=camera_nbv (arm/camera poses)
  B) request_base_pose && valid goal_pose_3d → action_type=base_pose
     (FKIE wants base relocation toward frontier; NOT an EE/camera optical goal)
Never invent a substitute EE. No fixed camera-height hard reject — REMANI/base
executors decide reachability.
"""
from __future__ import print_function

import argparse
import json
import math
import sys
from pathlib import Path

import actionlib
import rospy
import tf2_ros
from geometry_msgs.msg import Point32, PolygonStamped
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from fkie_measurement_msgs.msg import BoundaryPolygon, MeasurementEstimation
from fkie_nbv_planner.msg import NbvPlannerAction, NbvPlannerGoal

try:
    from octomap_msgs.msg import Octomap
except Exception:
    Octomap = None


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


def pose_norm(pose_dict):
    if not pose_dict or not pose_dict.get("pose"):
        return 0.0
    p = pose_dict["pose"]["position"]
    o = pose_dict["pose"]["orientation"]
    return math.sqrt(p["x"] ** 2 + p["y"] ** 2 + p["z"] ** 2) + abs(o["w"]) + abs(o["x"]) + abs(o["y"]) + abs(o["z"])


def pose_struct_valid(pose_dict):
    """Basic legality: finite xyz + finite non-degenerate quaternion. No height band."""
    if not pose_dict or not pose_dict.get("pose"):
        return False, "missing_pose"
    p = pose_dict["pose"]["position"]
    o = pose_dict["pose"]["orientation"]
    vals = [p["x"], p["y"], p["z"], o["x"], o["y"], o["z"], o["w"]]
    if not all(math.isfinite(float(v)) for v in vals):
        return False, "non_finite"
    qn = math.sqrt(o["x"] ** 2 + o["y"] ** 2 + o["z"] ** 2 + o["w"] ** 2)
    if qn < 0.5:
        return False, "degenerate_quat_norm=%.3f" % qn
    return True, "ok"


def yaw_from_quat_xyzw(o):
    x, y, z, w = float(o["x"]), float(o["y"]), float(o["z"]), float(o["w"])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def marker_stats(topic, timeout=1.5):
    try:
        ma = rospy.wait_for_message(topic, MarkerArray, timeout=timeout)
    except Exception as e:
        return {"ok": False, "error": str(e)}
    n_pts = sum(len(m.points or []) for m in ma.markers)
    n_pose = sum(1 for m in ma.markers if abs(m.pose.position.x) + abs(m.pose.position.y) > 1e-6)
    return {
        "ok": True,
        "n_markers": len(ma.markers),
        "n_points": n_pts,
        "n_posed": n_pose,
        "stamp": ma.markers[0].header.stamp.to_sec() if ma.markers else None,
    }


def lookup_xyz(buf, parent, child):
    t = buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(2.0))
    p = t.transform.translation
    q = t.transform.rotation
    return {
        "xyz": [float(p.x), float(p.y), float(p.z)],
        "quat_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
        "stamp": t.header.stamp.to_sec() if t.header.stamp else None,
    }


def classify_empty(result_dict, ctx):
    """Return (code, reason) for goals=[] cases. See task F1 §3.2."""
    n_goals = int(result_dict.get("n_goals") or 0)
    if n_goals > 0:
        return "HAS_GOALS", "goals[] non-empty"
    complete = bool(result_dict.get("complete_exploration"))
    req_base = bool(result_dict.get("request_base_pose"))
    gp = result_dict.get("goal_pose_3d")
    gp_ok = pose_norm(gp) > 0.05
    fr = ctx.get("frontiers") or {}
    cached = ctx.get("cached_nodes") or {}
    rrt = ctx.get("rrt") or {}
    best = ctx.get("best_branch") or {}

    if req_base and gp_ok:
        feas, why = pose_struct_valid(gp)
        if feas:
            return (
                "G_BASE_POSE",
                "goals[] empty but request_base_pose with valid goal_pose_3d "
                "(FKIE base relocation toward frontier — not an EE target)",
            )
        return (
            "H_ERROR",
            "request_base_pose goal_pose_3d structurally invalid (%s)" % why,
        )
    if complete and not gp_ok and not (fr.get("n_posed") or fr.get("n_markers")):
        return "A_COMPLETE", "complete_exploration=true and no usable frontier/goal_pose_3d"
    if complete and (cached.get("n_posed") or cached.get("n_markers")):
        return (
            "D_FILTERED",
            "complete_exploration with cached nodes still present — likely "
            "frontiers inside arm_goal_range and/or RRT branch gain-filtered",
        )
    if not (cached.get("n_posed") or cached.get("n_markers") or fr.get("n_posed")):
        if (rrt.get("n_markers") or 0) > 0:
            return "C_NO_BRANCH", "RRT markers exist but no frontiers/cached goals"
        return "B_NO_FRONTIER", "no cached/frontier markers; unknown space may remain"
    if (best.get("n_markers") or 0) == 0 and (rrt.get("n_markers") or 0) > 0:
        return "C_NO_BRANCH", "RRT grew but best_branch empty after gain/arm filter"
    if not ctx.get("camera_pose_ok"):
        return "E_TF", "camera TF missing or stale at diagnosis time"
    if not ctx.get("octomap_ok"):
        return "E_MAP", "octomap unavailable at diagnosis time"
    if req_base and not gp_ok:
        return "H_ERROR", "request_base_pose set but goal_pose_3d trivial/invalid"
    return "UNKNOWN_EMPTY", "goals=[] without a clearer diagnostic match"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/metrics/fkie_agx_warehouse_mode_b/fkie_action_result.json")
    ap.add_argument("--action-server", default="/robot/nbv_rrt")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument(
        "--boundary-xy",
        default="[[-8.3603,-2.6852],[-1.80,-2.6852],[-1.80,-8.6852],[-8.3603,-8.6852]]",
        help="JSON list of [x,y] polygon vertices (aisle_safe default)",
    )
    ap.add_argument("--min-height", type=float, default=0.0)
    ap.add_argument("--max-height", type=float, default=2.0)
    ap.add_argument(
        "--arm-in-prepacked",
        type=int,
        default=1,
        help="FKIE Action field (kept for API completeness; planner may ignore)",
    )
    ap.add_argument(
        "--allow-empty",
        action="store_true",
        help="Exit 0 even with no usable NBV (still write diagnosis)",
    )
    ap.add_argument("--accept-base-pose", action="store_true", default=True)
    ap.add_argument("--no-accept-base-pose", action="store_true")
    args = ap.parse_args()
    accept_base = bool(args.accept_base_pose) and not bool(args.no_accept_base_pose)

    rospy.init_node("save_fkie_action_result", anonymous=True)
    boundary_xy = json.loads(args.boundary_xy)

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(0.3)

    ctx = {
        "camera_pose_ok": False,
        "octomap_ok": False,
        "frontiers": marker_stats("/robot/viz_curr_frontiers"),
        "cached_nodes": marker_stats("/robot/viz_cached_nodes"),
        "rrt": marker_stats("/robot/viz_rrt"),
        "best_branch": marker_stats("/robot/viz_best_branch"),
    }
    try:
        ctx["camera_tf"] = lookup_xyz(buf, "world", "d455_depth_optical_frame")
        ctx["base_tf"] = lookup_xyz(buf, "world", "base_link")
        ctx["camera_pose_ok"] = True
    except Exception as e:
        ctx["camera_tf_error"] = str(e)
    if Octomap is not None:
        try:
            om = rospy.wait_for_message("/robot/octomap_full", Octomap, timeout=3.0)
            ctx["octomap"] = {
                "frame": om.header.frame_id,
                "stamp": om.header.stamp.to_sec() if om.header.stamp else None,
                "resolution": float(om.resolution),
                "data_len": len(om.data) if om.data is not None else 0,
            }
            ctx["octomap_ok"] = True
        except Exception as e:
            ctx["octomap_error"] = str(e)

    client = actionlib.SimpleActionClient(args.action_server, NbvPlannerAction)
    rospy.loginfo("Waiting for %s", args.action_server)
    if not client.wait_for_server(rospy.Duration(args.timeout)):
        raise SystemExit("FKIE_ACTION_FAIL: server not available")

    goal = NbvPlannerGoal()
    goal.header = Header(stamp=rospy.Time.now(), frame_id="world")
    goal.arm_in_prepacked = bool(args.arm_in_prepacked)

    poly = PolygonStamped()
    poly.header = goal.header
    for x, y in boundary_xy:
        poly.polygon.points.append(Point32(x=float(x), y=float(y), z=0.0))
    boundary = BoundaryPolygon()
    boundary.id = 1
    boundary.source_type = "save_fkie_action_result"
    boundary.min_height = float(args.min_height)
    boundary.max_height = float(args.max_height)
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

    # Refresh viz after planning
    ctx["frontiers_after"] = marker_stats("/robot/viz_curr_frontiers")
    ctx["cached_nodes_after"] = marker_stats("/robot/viz_cached_nodes")
    ctx["rrt_after"] = marker_stats("/robot/viz_rrt")
    ctx["best_branch_after"] = marker_stats("/robot/viz_best_branch")
    ctx["frontiers"] = ctx["frontiers_after"]
    ctx["cached_nodes"] = ctx["cached_nodes_after"]
    ctx["rrt"] = ctx["rrt_after"]
    ctx["best_branch"] = ctx["best_branch_after"]

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
        "boundary_xy": boundary_xy,
        "boundary_height": [float(args.min_height), float(args.max_height)],
        "arm_in_prepacked": bool(args.arm_in_prepacked),
        "wall_time": rospy.Time.now().to_sec(),
        "diagnosis_context": ctx,
    }

    code, reason = classify_empty(out, ctx)
    out["empty_class"] = code
    out["empty_reason"] = reason

    action_type = None
    source = None
    feasibility = {"ok": False, "reason": "none"}
    candidate_indices = []

    # Prefer arm/camera goals[] — structural validity only (REMANI decides reachability).
    valid_goals = []
    for g in goals:
        ok, why = pose_struct_valid(g)
        g["struct_ok"] = ok
        g["struct_reason"] = why
        if ok:
            valid_goals.append(g)
            candidate_indices.append(int(g.get("index", len(candidate_indices))))

    if valid_goals:
        action_type = "camera_nbv"
        source = "goals[%d]" % int(valid_goals[0].get("index", 0))
        feasibility = {"ok": True, "reason": "struct_ok", "n_candidates": len(valid_goals)}
        primary = valid_goals[0]
        p = primary["pose"]["position"]
        o = primary["pose"]["orientation"]
        out["camera_goal_xyz"] = [p["x"], p["y"], p["z"]]
        out["camera_goal_quat_xyzw"] = [o["x"], o["y"], o["z"], o["w"]]
        out["orientation_source"] = "fkie_action_quaternion"
        out["selected_goal_index"] = int(primary.get("index", 0))
        out["camera_candidates"] = [
            {
                "index": int(g.get("index", i)),
                "xyz": [
                    g["pose"]["position"]["x"],
                    g["pose"]["position"]["y"],
                    g["pose"]["position"]["z"],
                ],
                "quat_xyzw": [
                    g["pose"]["orientation"]["x"],
                    g["pose"]["orientation"]["y"],
                    g["pose"]["orientation"]["z"],
                    g["pose"]["orientation"]["w"],
                ],
            }
            for i, g in enumerate(valid_goals)
        ]
    elif accept_base and bool(result.request_base_pose) and pose_norm(out["goal_pose_3d"]) > 0.05:
        ok, why = pose_struct_valid(out["goal_pose_3d"])
        feasibility = {"ok": ok, "reason": why, "kind": "base_pose"}
        if ok:
            action_type = "base_pose"
            source = "goal_pose_3d_request_base_pose"
            gp = out["goal_pose_3d"]["pose"]
            # Base relocation uses XY + yaw; keep FKIE z only as metadata (not EE).
            out["base_goal_xy"] = [gp["position"]["x"], gp["position"]["y"]]
            out["base_goal_yaw"] = yaw_from_quat_xyzw(gp["orientation"])
            out["base_goal_z_meta"] = gp["position"]["z"]
            out["base_goal_quat_xyzw"] = [
                gp["orientation"]["x"],
                gp["orientation"]["y"],
                gp["orientation"]["z"],
                gp["orientation"]["w"],
            ]
            # Do NOT populate camera_goal_* for base requests — prevents EE misuse.
        else:
            out["empty_class"] = "H_ERROR"
            out["empty_reason"] = "request_base_pose goal_pose_3d invalid (%s)" % why

    usable = action_type is not None
    out["action_type"] = action_type
    out["candidate_indices"] = candidate_indices
    out["feasibility"] = feasibility
    out["usable_nbv"] = bool(usable)
    out["nbv_source"] = source

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    summary = {
        k: out[k]
        for k in (
            "action_state_name",
            "complete_exploration",
            "request_base_pose",
            "n_goals",
            "usable_nbv",
            "nbv_source",
            "action_type",
            "empty_class",
            "empty_reason",
            "camera_goal_xyz",
            "camera_goal_quat_xyzw",
            "base_goal_xy",
            "base_goal_yaw",
            "candidate_indices",
        )
        if k in out
    }
    print(json.dumps(summary, indent=2))
    print("n_goals", out["n_goals"], "usable_nbv", out["usable_nbv"], "class", out["empty_class"])

    if state != 3:
        print("FKIE_ACTION_FAIL: state=%s" % state_name)
        raise SystemExit(4)
    if not usable and not args.allow_empty:
        print("FKIE_ACTION_FAIL: no usable NBV (%s) — refuse to invent a substitute" % code)
        raise SystemExit(5)
    if usable:
        print("FKIE_ACTION_RESULT_OK")
    else:
        print("FKIE_ACTION_EMPTY_DIAGNOSED")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(1)
