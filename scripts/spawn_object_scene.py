#!/usr/bin/env python3
"""Spawn furniture/objects from a layout JSON into a running Gazebo Classic."""
from __future__ import print_function
import argparse
import json
import os
from pathlib import Path

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tft


def quat_yaw(yaw):
    q = tft.quaternion_from_euler(0, 0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def model_paths():
    root = Path(os.environ.get("NBV_ROOT", "/workspace"))
    return {
        "ycb": root / "third_party/gazebo_objects/gazebo_ycb/models",
        "local": root / "configs/fkie/agx_adapt/models",
    }


def resolve_sdf(item, paths):
    model = item["model"]
    src = item.get("source", "ycb")
    if src == "ycb":
        p = paths["ycb"] / model / "model.sdf"
        model_dir = paths["ycb"] / model
    else:
        p = paths["local"] / model / "model.sdf"
        model_dir = paths["local"] / model
    if not p.is_file():
        raise FileNotFoundError(str(p))
    return p.read_text(), model, model_dir


def prepare_sdf(xml, model_dir, force_static=True):
    """Rewrite model:// URIs to absolute file:// and optionally force static."""
    # model://mustard_bottle/meshes/x -> file:///.../mustard_bottle/meshes/x
    # Also handle relative mesh paths under this model dir.
    abs_model = model_dir.resolve().as_posix()
    parent = model_dir.parent.resolve().as_posix()
    model_name = model_dir.name
    xml = xml.replace("model://%s/" % model_name, "file://%s/" % abs_model)
    # generic model://foo -> file://<parent>/foo
    import re

    def repl(m):
        uri = m.group(1)
        if uri.startswith("/"):
            return "file://%s" % uri
        return "file://%s/%s" % (parent, uri)

    xml = re.sub(r"model://([^\s\"'<]+)", repl, xml)
    if force_static and "<static>" not in xml:
        xml = xml.replace("<model ", "<model ", 1)
        # insert static after opening model tag
        xml = re.sub(
            r"(<model[^>]*>)",
            r"\1\n    <static>true</static>",
            xml,
            count=1,
        )
    return xml


def spawn_one(spawn_srv, delete_srv, name, xml, pose_dict, get_state):
    # delete if exists
    try:
        st = get_state(name, "world")
        if st.success:
            delete_srv(name)
            rospy.sleep(0.2)
    except Exception:
        pass
    pose = Pose()
    pose.position = Point(x=pose_dict["x"], y=pose_dict["y"], z=pose_dict["z"])
    pose.orientation = quat_yaw(float(pose_dict.get("yaw", 0.0)))
    resp = spawn_srv(name, xml, "", pose, "world")
    return resp.success, resp.status_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--prefix", default="mo_")
    args = ap.parse_args()

    layout = json.loads(Path(args.layout).read_text())
    paths = model_paths()
    rospy.init_node("spawn_object_scene")
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=60)
    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    items = list(layout.get("furniture", [])) + list(layout.get("objects", [])) + list(
        layout.get("occluders", [])
    )
    results = []
    for item in items:
        name = args.prefix + item["instance_id"]
        xml, _, model_dir = resolve_sdf(item, paths)
        xml = prepare_sdf(xml, model_dir, force_static=True)
        ok, msg = spawn_one(spawn_srv, delete_srv, name, xml, item["spawn"], get_state)
        print("spawn", name, ok, msg)
        results.append({"gazebo_name": name, "ok": bool(ok), "msg": msg, **item})

    out = Path(args.layout).with_suffix(".spawned.json")
    Path(out).write_text(json.dumps({"layout": args.layout, "spawned": results}, indent=2))
    print("Wrote", out)
    n_ok = sum(1 for r in results if r["ok"])
    print("SPAWN_OBJECT_SCENE_OK" if n_ok == len(results) else "SPAWN_OBJECT_SCENE_PARTIAL", n_ok, "/", len(results))


if __name__ == "__main__":
    main()
