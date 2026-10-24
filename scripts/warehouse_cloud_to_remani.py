#!/usr/bin/env python3
"""Publish warehouse obstacle PointCloud2 on /map_generator/global_cloud for REMANI GridMap.

Primary: downsample /realsense/depth/points2 into world frame (sensor online map).
Fallback: if --synthetic-shelves, also emit coarse shelf/table boxes for early map fill
(validation aid; marked in JSON metrics — not a hidden GT planning oracle when disabled).
"""
from __future__ import print_function
import argparse
import struct

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


def make_cloud(points, stamp, frame="world"):
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    return pc2.create_cloud(rospy.Header(stamp=stamp, frame_id=frame), fields, points)


def box_points(xmin, xmax, ymin, ymax, zmin, zmax, step=0.15):
    pts = []
    x = xmin
    while x <= xmax + 1e-9:
        y = ymin
        while y <= ymax + 1e-9:
            z = zmin
            while z <= zmax + 1e-9:
                # surface-ish sampling: keep near faces
                on_face = (
                    abs(x - xmin) < step
                    or abs(x - xmax) < step
                    or abs(y - ymin) < step
                    or abs(y - ymax) < step
                    or abs(z - zmin) < step
                    or abs(z - zmax) < step
                )
                if on_face:
                    pts.append((x, y, z))
                z += step
            y += step
        x += step
    return pts


class Feeder(object):
    def __init__(self, synthetic, skip):
        self.synthetic = synthetic
        self.skip = max(1, int(skip))
        self.pub = rospy.Publisher("/map_generator/global_cloud", PointCloud2, queue_size=1, latch=True)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        self._syn = []
        if synthetic:
            # Coarse warehouse obstacles near aisle_safe (approx; for early ESDF fill only)
            self._syn += box_points(-8.5, -6.5, -9.0, -2.5, 0.0, 2.2, 0.2)
            self._syn += box_points(-5.5, -3.5, -9.0, -2.5, 0.0, 2.2, 0.2)
            self._syn += box_points(-4.2, -3.6, -7.8, -7.2, 0.0, 0.74, 0.1)  # table
        rospy.Subscriber("/realsense/depth/points2", PointCloud2, self._on_pc, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0), self._tick)
        self.last_pts = list(self._syn)

    def _on_pc(self, msg):
        pts = []
        try:
            tfm = self.buf.lookup_transform("world", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.2))
            import tf.transformations as tft
            import numpy as np

            q = tfm.transform.rotation
            t = tfm.transform.translation
            T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            T[0:3, 3] = [t.x, t.y, t.z]
            for i, p in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
                if i % self.skip != 0:
                    continue
                v = np.array([p[0], p[1], p[2], 1.0])
                w = T.dot(v)
                if 0.05 < w[2] < 3.5:
                    pts.append((float(w[0]), float(w[1]), float(w[2])))
                if len(pts) > 80000:
                    break
        except Exception:
            return
        self.last_pts = list(self._syn) + pts

    def _tick(self, _evt):
        if not self.last_pts:
            return
        cloud = make_cloud(self.last_pts, rospy.Time.now(), "world")
        self.pub.publish(cloud)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic-shelves", action="store_true")
    ap.add_argument("--skip", type=int, default=8)
    args, _ = ap.parse_known_args()
    rospy.init_node("warehouse_cloud_to_remani")
    Feeder(args.synthetic_shelves, args.skip)
    rospy.loginfo(
        "warehouse→REMANI cloud feeder (synthetic=%s skip=%d)", args.synthetic_shelves, args.skip
    )
    rospy.spin()


if __name__ == "__main__":
    main()
