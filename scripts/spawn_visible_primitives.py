#!/usr/bin/env python3
"""Spawn brightly colored primitive objects in Mode A D455 free-space FOV.

Used when YCB meshes are hard to see against warehouse shelves. Primitives still
have collision and are localized via RGB-D. Does not read GT into perception.
"""
from __future__ import print_function
import argparse
import json
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from cv_bridge import CvBridge
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetWorldProperties
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import Image, CameraInfo


COLORS = [
    ("bottle", (1.0, 0.55, 0.0), 0.06),
    ("bottle", (0.1, 0.35, 0.95), 0.06),
    ("banana", (1.0, 0.9, 0.1), 0.05),
    ("apple", (0.95, 0.05, 0.05), 0.05),
    ("box", (0.75, 0.55, 0.2), 0.07),
    ("box", (0.55, 0.4, 0.15), 0.07),
    ("bowl", (0.85, 0.85, 0.9), 0.07),
    ("can", (0.9, 0.25, 0.1), 0.05),
    ("cone", (1.0, 0.45, 0.0), 0.08),
    ("box", (0.65, 0.5, 0.25), 0.07),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument(
        "--out-layout",
        default="/workspace/data/scenes/indoor_objects/primitive_debug/layouts/seed42.json",
    )
    ap.add_argument("--n", type=int, default=10)
    args = ap.parse_args()

    rospy.init_node("spawn_visible_primitives")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)
    t = buf.lookup_transform("world", "d455_depth_optical_frame", rospy.Time(0), rospy.Duration(2))
    p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
    T = tft.quaternion_matrix(q)
    T[:3, 3] = p
    K = np.array(rospy.wait_for_message("/realsense/depth/camera_info", CameraInfo, timeout=5).K).reshape(3, 3)
    depth = CvBridge().imgmsg_to_cv2(
        rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=10), "passthrough"
    ).astype(np.float32)
    if np.nanmedian(depth[np.isfinite(depth)]) > 50:
        depth *= 0.001

    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    wp = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)()
    for n in list(wp.model_names):
        if n.startswith("mo_") or n.startswith("vis_") or n == "probe_sphere":
            try:
                delete(n)
            except Exception:
                pass
    rospy.sleep(0.4)

    slots = []
    for gu in range(50, 520, 35):
        for gv in range(50, 420, 35):
            win = depth[gv - 3 : gv + 4, gu - 3 : gu + 4]
            fin = win[np.isfinite(win) & (win > 0.7) & (win < 2.6)]
            if fin.size < 12:
                continue
            z = float(np.median(fin))
            if float(np.percentile(fin, 80) - np.percentile(fin, 20)) > 0.35:
                continue
            x = (gu - K[0, 2]) * z / K[0, 0]
            y = (gv - K[1, 2]) * z / K[1, 1]
            pw = T.dot([x, y, z, 1.0])[:3]
            d = pw - p
            d = d / np.linalg.norm(d)
            place = pw - 0.25 * d
            if place[2] < 0.2 or place[2] > 2.2:
                continue
            slots.append((gu, gv, z, place))

    chosen = []
    for s in sorted(slots, key=lambda x: x[2]):
        if all(np.linalg.norm(s[3] - c[3]) > 0.18 for c in chosen):
            chosen.append(s)
        if len(chosen) >= args.n:
            break
    print("slots", len(chosen))

    objs = []
    for i, ((cls, col, rad), slot) in enumerate(zip(COLORS, chosen)):
        gu, gv, z, place = slot
        name = "vis_obj_%02d" % i
        sdf = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="%s">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><sphere><radius>%f</radius></sphere></geometry>
        <material>
          <ambient>%f %f %f 1</ambient>
          <diffuse>%f %f %f 1</diffuse>
        </material>
      </visual>
      <collision name="c">
        <geometry><sphere><radius>%f</radius></sphere></geometry>
      </collision>
    </link>
  </model>
</sdf>""" % (
            name,
            rad,
            col[0],
            col[1],
            col[2],
            col[0],
            col[1],
            col[2],
            rad,
        )
        pose = Pose()
        pose.position = Point(float(place[0]), float(place[1]), float(place[2]))
        pose.orientation.w = 1.0
        resp = spawn(name, sdf, "", pose, "world")
        print("spawn", name, resp.success, [round(float(x), 3) for x in place], cls)
        objs.append(
            {
                "instance_id": "obj_%02d" % i,
                "gazebo_name": name,
                "model": "primitive_sphere",
                "class_name": cls,
                "difficulty": "easy" if i < 4 else ("medium" if i < 7 else "hard"),
                "spawn": {"x": float(place[0]), "y": float(place[1]), "z": float(place[2]), "yaw": 0.0},
                "source": "primitive",
                "ok": bool(resp.success),
                "msg": resp.status_message,
            }
        )

    layout = {
        "scene_id": "indoor_objects_seed%d" % args.seed,
        "seed": args.seed,
        "notes": "Visible primitives in Mode A Gazebo camera FOV (TF matches Gazebo joints).",
        "frame_id": "world",
        "furniture": [],
        "objects": objs,
        "occluders": [],
    }
    out = Path(args.out_layout)
    if not out.is_absolute():
        root = Path(rospy.get_param("/nbv_root", "/workspace"))
        out = root / out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(layout, indent=2))
    out.with_suffix(".spawned.json").write_text(
        json.dumps({"layout": str(out), "spawned": objs}, indent=2)
    )
    print("SPAWN_VISIBLE_PRIMITIVES_OK", len(objs), out)


if __name__ == "__main__":
    main()
