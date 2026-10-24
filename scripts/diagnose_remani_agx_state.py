#!/usr/bin/env python3
"""Compare Gazebo AGX state vs REMANI expected topics (/mm/car/odom, /mm/mani/joint_state, TF)."""
from __future__ import print_function
import json
import math
from pathlib import Path

import rospy
import tf2_ros
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


ARM = ["cr10_joint1", "cr10_joint2", "cr10_joint3", "cr10_joint4", "cr10_joint5", "cr10_joint6"]


def yaw_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rospy.init_node("diagnose_remani_agx_state", anonymous=True)
    out_path = rospy.get_param("~out", "data/metrics/fkie_agx_warehouse_mode_b/remani_state_align.json")
    model = rospy.get_param("~model", "agx_ranger_cr10")
    rospy.wait_for_service("/gazebo/get_model_state", timeout=20)
    gz = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)(model, "world")
    js = rospy.wait_for_message("/joint_states", JointState, timeout=10)
    js_map = dict(zip(js.name, js.position))

    mm_odom = None
    mm_js = None
    try:
        mm_odom = rospy.wait_for_message("/mm/car/odom", Odometry, timeout=5)
    except Exception as e:
        mm_odom = str(e)
    try:
        mm_js = rospy.wait_for_message("/mm/mani/joint_state", JointState, timeout=5)
    except Exception as e:
        mm_js = str(e)

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(0.5)
    try:
        tfm = buf.lookup_transform("world", "base_link", rospy.Time(0), rospy.Duration(2.0))
        tf_xy = [tfm.transform.translation.x, tfm.transform.translation.y]
        tf_yaw = yaw_q(tfm.transform.rotation)
    except Exception as e:
        tf_xy, tf_yaw = ["err", str(e)], None

    gz_xy = [gz.pose.position.x, gz.pose.position.y]
    gz_yaw = yaw_q(gz.pose.orientation)
    align = {"ok": False}
    if isinstance(mm_odom, Odometry):
        # fake_mm encoding: linear = x,y,yaw
        mx, my, myaw = mm_odom.twist.twist.linear.x, mm_odom.twist.twist.linear.y, mm_odom.twist.twist.linear.z
        xy_err = math.hypot(mx - gz_xy[0], my - gz_xy[1])
        yaw_err = abs(math.atan2(math.sin(myaw - gz_yaw), math.cos(myaw - gz_yaw)))
        joint_err = {}
        if isinstance(mm_js, JointState) and mm_js.position:
            for i, name in enumerate(ARM):
                if i < len(mm_js.position):
                    joint_err[name] = abs(float(mm_js.position[i]) - float(js_map.get(name, 0.0)))
        max_j = max(joint_err.values()) if joint_err else 999.0
        ok = xy_err < 0.05 and yaw_err < 0.1 and max_j < 0.1
        align = {
            "ok": ok,
            "xy_err_m": xy_err,
            "yaw_err_rad": yaw_err,
            "max_joint_err_rad": max_j,
            "joint_err": joint_err,
            "mm_odom_linear_xyz": [mx, my, myaw],
        }

    out = {
        "gazebo_xy_yaw": gz_xy + [gz_yaw],
        "tf_xy_yaw": (tf_xy + [tf_yaw]) if tf_yaw is not None else tf_xy,
        "arm_from_joint_states": {k: float(js_map.get(k, 0)) for k in ARM},
        "mm_car_odom_present": isinstance(mm_odom, Odometry),
        "mm_mani_js_present": isinstance(mm_js, JointState),
        "alignment": align,
    }
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    Path(out_path).write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print("REMANI_AGX_STATE_OK" if align.get("ok") else "REMANI_AGX_STATE_FAIL")
    if not align.get("ok"):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
