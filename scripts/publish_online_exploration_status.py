#!/usr/bin/env python3
"""Publish online NBV exploration HUD + confirmed-object markers for RViz."""
from __future__ import print_function

import argparse
import json
import time
from pathlib import Path

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--status-json", required=True)
    ap.add_argument("--memory-json", default="")
    ap.add_argument("--rate", type=float, default=2.0)
    ap.add_argument("--once", action="store_true")
    args = ap.parse_args()

    rospy.init_node("publish_online_exploration_status", anonymous=True)
    pub_txt = rospy.Publisher("/nbv/online/status_text", String, queue_size=1, latch=True)
    pub_mk = rospy.Publisher("/nbv/online/confirmed_objects", MarkerArray, queue_size=1, latch=True)

    def publish_once():
        st = json.loads(Path(args.status_json).read_text())
        n_conf = int(st.get("confirmed_objects", 0))
        lines = [
            "NBV Step: %s/%s" % (st.get("step", "?"), st.get("max_steps", "?")),
            "FKIE: %s" % st.get("fkie", "n/a"),
            "REMANI: %s" % st.get("remani", "n/a"),
            "Camera: %s" % st.get("camera", "n/a"),
            "GSAM: %s" % st.get("gsam", "n/a"),
            "Confirmed Objects: %d" % n_conf,
            "Phase: %s" % st.get("phase", "n/a"),
        ]
        msg = String(data="\n".join(lines))
        pub_txt.publish(msg)

        ma = MarkerArray()
        mem_path = args.memory_json or st.get("memory_path")
        if mem_path and Path(mem_path).is_file():
            mem = json.loads(Path(mem_path).read_text())
            for i, o in enumerate(mem.get("objects", [])):
                if o.get("confirmation_status") != "confirmed":
                    continue
                m = Marker()
                m.header.frame_id = "world"
                m.header.stamp = rospy.Time.now()
                m.ns = "confirmed_objects"
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                c = o.get("position_world") or o.get("center_world") or [0, 0, 0]
                m.pose.position.x, m.pose.position.y, m.pose.position.z = c
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.08
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 0.9
                m.text = o.get("class_name", "")
                ma.markers.append(m)
        pub_mk.publish(ma)
        print(msg.data.replace("\n", " | "))

    if args.once:
        publish_once()
        return

    r = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        try:
            publish_once()
        except Exception as e:
            rospy.logwarn_throttle(5.0, "status publish: %s", e)
        r.sleep()


if __name__ == "__main__":
    main()
