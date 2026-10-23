#!/usr/bin/env python3
"""Manual visibility probe: table + mustard + cracker near aisle_safe."""
from __future__ import print_function
import json
import re
import sys
from pathlib import Path

import rospy
from gazebo_msgs.srv import SpawnModel, GetModelState, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tft


def prep(path):
    model_dir = path.parent
    xml = path.read_text()
    abs_model = model_dir.resolve().as_posix()
    parent = model_dir.parent.resolve().as_posix()
    name = model_dir.name
    xml = xml.replace("model://%s/" % name, "file://%s/" % abs_model)

    def repl(m):
        uri = m.group(1)
        if uri.startswith("/"):
            return "file://%s" % uri
        return "file://%s/%s" % (parent, uri)

    xml = re.sub(r"model://([^\s\"'<]+)", repl, xml)
    if "<static>" not in xml:
        xml = re.sub(r"(<model[^>]*>)", r"\1\n    <static>true</static>", xml, count=1)
    return xml


def main():
    rospy.init_node("manual_spawn_probe", anonymous=True)
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=30)
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    items = [
        (
            "probe_simple_table",
            Path("/workspace/configs/fkie/agx_adapt/models/simple_table/model.sdf"),
            -2.55,
            -6.55,
            0.0,
        ),
        (
            "probe_mustard_bottle",
            Path("/workspace/third_party/gazebo_objects/gazebo_ycb/models/mustard_bottle/model.sdf"),
            -2.40,
            -6.40,
            0.74,
        ),
        (
            "probe_cracker_box",
            Path("/workspace/third_party/gazebo_objects/gazebo_ycb/models/cracker_box/model.sdf"),
            -2.70,
            -6.45,
            0.74,
        ),
    ]
    out = []
    for name, sdf, x, y, z in items:
        try:
            st = get(name, "world")
            if st.success:
                delete(name)
                rospy.sleep(0.2)
        except Exception:
            pass
        xml = prep(sdf)
        missing = []
        for m in re.finditer(r"file://([^\s\"'<]+)", xml):
            p = Path(m.group(1))
            if not p.is_file():
                missing.append(str(p))
        q = tft.quaternion_from_euler(0, 0, 0.2)
        pose = Pose(position=Point(x, y, z), orientation=Quaternion(*q))
        resp = spawn(name, xml, "", pose, "world")
        rospy.sleep(0.3)
        st = get(name, "world")
        actual = None
        if st.success:
            actual = [float(st.pose.position.x), float(st.pose.position.y), float(st.pose.position.z)]
        rec = {
            "name": name,
            "spawn_ok": bool(resp.success),
            "msg": resp.status_message,
            "missing_assets": missing,
            "exists": bool(st.success),
            "actual": actual,
        }
        out.append(rec)
        print(json.dumps(rec))
    Path("/workspace/data/metrics/indoor_objects/ycb").mkdir(parents=True, exist_ok=True)
    Path("/workspace/data/metrics/indoor_objects/ycb/manual_probe.json").write_text(json.dumps(out, indent=2))
    n = sum(1 for r in out if r["exists"])
    print("MANUAL_PROBE_DONE", n, "/", len(out))
    return 0 if n == len(out) else 1


if __name__ == "__main__":
    sys.exit(main())
