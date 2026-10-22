#!/usr/bin/env python3
"""Compare T_world_camera from TF vs Gazebo (Mode A aware).

Mode A <static> + fixed-joint bake collapses arm links (d455_link/cr10_Link*) into
base_link. For those, validate:
  1) world←base TF vs /gazebo/get_model_state
  2) a surviving EE link (gripper tip) TF vs Gazebo link_states
  3) T_world_optical_from_TF vs model_pose * T_base_optical (URDF/TF)
Optional calib JSON is reported only as historical debug, not as authority.
"""
from __future__ import print_function
import argparse
import json
import math
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import GetModelState


def pose_err(T_a, T_b):
    d = T_a[:3, 3] - T_b[:3, 3]
    R = T_a[:3, :3].T.dot(T_b[:3, :3])
    tr = np.clip((np.trace(R) - 1) * 0.5, -1, 1)
    ang = math.acos(tr)
    return float(np.linalg.norm(d)), float(ang)


def tf_to_T(buf, parent, child):
    tfm = buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(2.0))
    t, r = tfm.transform.translation, tfm.transform.rotation
    T = tft.quaternion_matrix([r.x, r.y, r.z, r.w])
    T[0:3, 3] = [t.x, t.y, t.z]
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--optical-frame", default="d455_depth_optical_frame")
    ap.add_argument("--probe-link", default="gripper_finger1_finger_tip_link")
    ap.add_argument("--model", default="agx_ranger_cr10")
    ap.add_argument("--calib-json", default="")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    rospy.init_node("diagnose_d455_tf")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    T_tf_opt = tf_to_T(buf, "world", args.optical_frame)
    T_tf_base = tf_to_T(buf, "world", "base_link")
    T_base_opt = tf_to_T(buf, "base_link", args.optical_frame)

    gm = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
    ms = gm(args.model, "world")
    gp, go = ms.pose.position, ms.pose.orientation
    T_gz_model = tft.quaternion_matrix([go.x, go.y, go.z, go.w])
    T_gz_model[0:3, 3] = [gp.x, gp.y, gp.z]

    # Compose expected optical from Gazebo model × URDF/TF base→optical
    T_gz_opt = T_gz_model.dot(T_base_opt)

    link_states = rospy.wait_for_message("/gazebo/link_states", LinkStates, timeout=5.0)
    gz_links = {}
    for name, pose in zip(link_states.name, link_states.pose):
        if name.startswith(args.model + "::"):
            short = name.split("::", 1)[1]
            gz_links[short] = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ]

    collapsed = args.optical_frame not in gz_links and "d455_link" not in gz_links
    probe = {"name": args.probe_link, "tf": None, "gazebo": gz_links.get(args.probe_link), "pos_err_m": None}
    if args.probe_link:
        try:
            T_probe = tf_to_T(buf, "world", args.probe_link)
            probe["tf"] = T_probe[0:3, 3].tolist()
            if probe["gazebo"] is not None:
                probe["pos_err_m"] = float(np.linalg.norm(np.array(probe["tf"]) - np.array(probe["gazebo"])))
        except Exception as e:
            probe["error"] = str(e)

    base_err, base_ang = pose_err(T_tf_base, T_gz_model)
    opt_err, opt_ang = pose_err(T_tf_opt, T_gz_opt)

    out = {
        "optical_frame": args.optical_frame,
        "mode_a_links_collapsed": collapsed,
        "gazebo_agx_link_names": sorted(gz_links.keys()),
        "tf_optical_xyz": T_tf_opt[0:3, 3].tolist(),
        "tf_optical_fwd": T_tf_opt[0:3, 0:3].dot([0, 0, 1]).tolist(),
        "gazebo_model_xyz": T_gz_model[0:3, 3].tolist(),
        "gazebo_compose_optical_xyz": T_gz_opt[0:3, 3].tolist(),
        "gazebo_compose_optical_fwd": T_gz_opt[0:3, 0:3].dot([0, 0, 1]).tolist(),
        "base_tf_vs_gazebo_pos_err_m": base_err,
        "base_tf_vs_gazebo_ang_err_deg": base_ang * 180.0 / math.pi,
        "optical_tf_vs_gazebo_compose_pos_err_m": opt_err,
        "optical_tf_vs_gazebo_compose_ang_err_deg": opt_ang * 180.0 / math.pi,
        "probe_link": probe,
        "match_ok": (
            base_err < 0.02
            and opt_err < 0.02
            and opt_ang < 0.05
            and (probe.get("pos_err_m") is None or probe["pos_err_m"] < 0.02)
        ),
        "note": "Authority: TF optical after Mode A bake == Gazebo model × base→optical. Calib JSON is legacy only.",
    }
    if args.calib_json and Path(args.calib_json).is_file():
        T_cal = np.array(json.loads(Path(args.calib_json).read_text())["T"], dtype=float)
        pe, ae = pose_err(T_tf_opt, T_cal)
        out["calib_vs_tf_pos_err_m"] = pe
        out["calib_vs_tf_ang_err_deg"] = ae * 180.0 / math.pi
        out["calib_deprecated"] = True

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(
        json.dumps(
            {
                k: out[k]
                for k in (
                    "match_ok",
                    "mode_a_links_collapsed",
                    "optical_tf_vs_gazebo_compose_pos_err_m",
                    "optical_tf_vs_gazebo_compose_ang_err_deg",
                    "probe_link",
                    "tf_optical_xyz",
                )
            },
            indent=2,
        )
    )
    print("DIAGNOSE_D455_TF_OK" if out["match_ok"] else "DIAGNOSE_D455_TF_MISMATCH")


if __name__ == "__main__":
    main()
