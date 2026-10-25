#!/usr/bin/env python3
"""Execute a Mode B base XY+yaw goal via Gazebo set_model_state (kinematic plant).

FKIE request_base_pose / goal_pose_3d → base relocation (NOT EE/camera optical).
Uses the same SetModelState plant as agx_mode_b_car_cmd_bridge so REMANI idle
car_cmd hold cannot steal the target. Also publishes car_cmd for observability.
"""
from __future__ import print_function

import argparse
import json
import math
import time
from pathlib import Path

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Twist


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def load_goal(path):
    d = json.loads(Path(path).read_text())
    if d.get("action_type") == "camera_nbv":
        raise SystemExit("REFUSE: camera_nbv JSON is not a base goal")
    if "base_goal_xy" in d:
        xy = list(d["base_goal_xy"])
        yaw = float(d.get("base_goal_yaw", 0.0))
        return xy[0], xy[1], yaw, d
    if "goal_pose_3d" in d and d["goal_pose_3d"]:
        p = d["goal_pose_3d"]["pose"]["position"]
        o = d["goal_pose_3d"]["pose"]["orientation"]
        yaw = math.atan2(
            2.0 * (o["w"] * o["z"] + o["x"] * o["y"]),
            1.0 - 2.0 * (o["y"] * o["y"] + o["z"] * o["z"]),
        )
        return float(p["x"]), float(p["y"]), yaw, d
    raise SystemExit("Unrecognized base goal JSON: %s" % path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from-fkie-json", required=True)
    ap.add_argument("--model", default="agx_ranger_cr10")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--xy-tol", type=float, default=0.12)
    ap.add_argument("--yaw-tol-deg", type=float, default=12.0)
    ap.add_argument("--alpha", type=float, default=0.25, help="lerp factor toward goal")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    gx, gy, gyaw, meta = load_goal(args.from_fkie_json)
    if not all(math.isfinite(v) for v in (gx, gy, gyaw)):
        raise SystemExit("INVALID_BASE_GOAL: non-finite")

    rospy.init_node("execute_base_pose_goal", anonymous=True)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=20)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=20)
    get_s = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_s = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    pub = rospy.Publisher("/mm_controller_node/car_cmd", Twist, queue_size=1)

    st0 = get_s(args.model, "world")
    if not st0.success:
        raise SystemExit("GAZEBO_MODEL_FAIL")
    x0, y0, z0 = st0.pose.position.x, st0.pose.position.y, st0.pose.position.z
    yaw0 = yaw_from_quat(st0.pose.orientation)
    dist0 = math.hypot(gx - x0, gy - y0)
    if dist0 > 8.0:
        raise SystemExit("BASE_GOAL_TOO_FAR: dist=%.2f m — refuse" % dist0)

    cmd = Twist()
    cmd.linear.x, cmd.linear.y, cmd.linear.z = gx, gy, gyaw
    yaw_tol = math.radians(args.yaw_tol_deg)
    alpha = float(args.alpha)
    dt = 1.0 / max(1.0, float(args.rate))

    x, y, yaw = x0, y0, yaw0
    t0 = time.time()
    final = None
    ok = False
    while time.time() - t0 < args.timeout and not rospy.is_shutdown():
        # exponential approach in world XY + yaw (Mode B kinematic)
        x = x + alpha * (gx - x)
        y = y + alpha * (gy - y)
        yaw = yaw + alpha * ang_diff(gyaw, yaw)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        ms = ModelState()
        ms.model_name = args.model
        ms.reference_frame = "world"
        ms.pose.position.x = x
        ms.pose.position.y = y
        ms.pose.position.z = z0
        ms.pose.orientation.x = qx
        ms.pose.orientation.y = qy
        ms.pose.orientation.z = qz
        ms.pose.orientation.w = qw
        set_s(ms)
        pub.publish(cmd)

        st = get_s(args.model, "world")
        xr, yr = st.pose.position.x, st.pose.position.y
        yawr = yaw_from_quat(st.pose.orientation)
        exy = math.hypot(gx - xr, gy - yr)
        eyaw = abs(ang_diff(gyaw, yawr))
        final = {
            "xyz": [xr, yr, st.pose.position.z],
            "yaw": yawr,
            "xy_err": exy,
            "yaw_err_rad": eyaw,
        }
        if exy <= args.xy_tol and eyaw <= yaw_tol:
            # snap exact
            ms.pose.position.x, ms.pose.position.y = gx, gy
            qx, qy, qz, qw = quat_from_yaw(gyaw)
            ms.pose.orientation.x, ms.pose.orientation.y = qx, qy
            ms.pose.orientation.z, ms.pose.orientation.w = qz, qw
            for _ in range(8):
                set_s(ms)
                pub.publish(cmd)
                rospy.sleep(0.02)
            ok = True
            break
        rospy.sleep(dt)

    out = {
        "ok": ok,
        "action_type": "base_pose",
        "goal_xy": [gx, gy],
        "goal_yaw": gyaw,
        "start_xy": [x0, y0],
        "start_yaw": yaw0,
        "path_length_xy": dist0,
        "elapsed_s": time.time() - t0,
        "final": final,
        "plant": "gazebo_set_model_state_lerp",
        "source_meta_keys": sorted(meta.keys()),
    }
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print("BASE_POSE_EXEC_OK" if ok else "BASE_POSE_EXEC_FAIL")
    raise SystemExit(0 if ok else 11)


if __name__ == "__main__":
    main()
