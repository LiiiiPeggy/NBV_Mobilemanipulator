#!/usr/bin/env python3
"""Publish filtered warehouse obstacle PointCloud2 on /map_generator/global_cloud.

Pipeline:
  D455 /realsense/depth/points2
    → TF at msg.header.stamp (skip frame if unavailable; never silently use latest)
    → geometric robot self-filter (link capsules/boxes; NOT a 0.9 m radius cull)
    → voxel downsample + local accumulation
    → /map_generator/global_cloud (world)

Also publishes debug:
  /warehouse_map/cloud_raw_world
  /warehouse_map/cloud_filtered
  /warehouse_map/self_filter_stats (std_msgs/String JSON)

GridMap (simulated + use_load_map + use_global_map) consumes the first global
cloud once; keep publishing a latched accumulated cloud so late subscribers still
get a filtered map. Prefer starting REMANI after feeder reports ready.
"""
from __future__ import print_function

import argparse
import json
import math
import sys
from collections import OrderedDict

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft

sys.path.insert(0, "/workspace/scripts")
try:
    from robot_self_filter import APPROX_NOTE, filter_points, lookup_link_transforms
except ImportError:
    from scripts.robot_self_filter import (  # type: ignore
        APPROX_NOTE,
        filter_points,
        lookup_link_transforms,
    )


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


class VoxelAccum(object):
    """Finite local voxel set (ordered for stable publish)."""

    def __init__(self, voxel=0.05, max_voxels=250000):
        self.voxel = float(voxel)
        self.max_voxels = int(max_voxels)
        self._vox = OrderedDict()

    def _key(self, x, y, z):
        v = self.voxel
        return (int(math.floor(x / v)), int(math.floor(y / v)), int(math.floor(z / v)))

    def add_many(self, pts):
        v = self.voxel
        for p in pts:
            k = self._key(p[0], p[1], p[2])
            if k in self._vox:
                continue
            self._vox[k] = (k[0] * v + 0.5 * v, k[1] * v + 0.5 * v, k[2] * v + 0.5 * v)
            if len(self._vox) > self.max_voxels:
                # drop oldest
                self._vox.popitem(last=False)

    def points(self):
        return list(self._vox.values())

    def __len__(self):
        return len(self._vox)


class Feeder(object):
    def __init__(self, args):
        self.skip = max(1, int(args.skip))
        self.inflate = float(args.self_inflate)
        self.z_min = float(args.z_min)
        self.z_max = float(args.z_max)
        self.synthetic = bool(args.synthetic_shelves)
        self.accum = VoxelAccum(voxel=args.voxel, max_voxels=args.max_voxels)
        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        tf2_ros.TransformListener(self.buf)

        self.pub = rospy.Publisher(
            "/map_generator/global_cloud", PointCloud2, queue_size=1, latch=True
        )
        self.pub_raw = rospy.Publisher(
            "/warehouse_map/cloud_raw_world", PointCloud2, queue_size=1, latch=False
        )
        self.pub_filt = rospy.Publisher(
            "/warehouse_map/cloud_filtered", PointCloud2, queue_size=1, latch=True
        )
        self.pub_stats = rospy.Publisher(
            "/warehouse_map/self_filter_stats", String, queue_size=1, latch=True
        )

        self._syn = []
        if self.synthetic:
            # Validation aid only — NOT for claiming online map completeness.
            self._syn += box_points(-8.5, -6.5, -9.0, -2.5, 0.0, 2.2, 0.2)
            self._syn += box_points(-5.5, -3.5, -9.0, -2.5, 0.0, 2.2, 0.2)
            self._syn += box_points(-4.2, -3.6, -7.8, -7.2, 0.0, 0.74, 0.1)

        self.stats = {
            "approx_note": APPROX_NOTE,
            "frames_ok": 0,
            "frames_skip_tf": 0,
            "frames_skip_empty": 0,
            "last_raw": 0,
            "last_kept": 0,
            "last_removed": 0,
            "accum_n": 0,
            "last_stamp": None,
            "last_skip_reason": None,
            "use_sim_time": None,
            "ready": False,
        }
        self._last_filt = list(self._syn)
        if self._syn:
            self.accum.add_many(self._syn)

        rospy.Subscriber("/realsense/depth/points2", PointCloud2, self._on_pc, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.5), self._tick)
        rospy.loginfo(
            "warehouse→REMANI feeder: self-filter+stamp-TF voxel=%.3f synthetic=%s",
            args.voxel,
            self.synthetic,
        )

    def _tf_matrix(self, tfm):
        q = tfm.transform.rotation
        t = tfm.transform.translation
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = [t.x, t.y, t.z]
        return T

    def _on_pc(self, msg):
        stamp = msg.header.stamp
        frame = msg.header.frame_id
        if stamp.to_sec() <= 0.0:
            self.stats["frames_skip_tf"] += 1
            self.stats["last_skip_reason"] = "cloud_stamp_zero"
            return

        try:
            tfm = self.buf.lookup_transform(
                "world", frame, stamp, rospy.Duration(0.2)
            )
        except Exception as e:
            self.stats["frames_skip_tf"] += 1
            self.stats["last_skip_reason"] = "cam_tf_miss:" + str(e)
            rospy.logwarn_throttle(2.0, "skip cloud: camera TF at stamp failed: %s", e)
            return

        try:
            link_T = lookup_link_transforms(self.buf, stamp, world_frame="world", timeout=0.15)
        except Exception as e:
            self.stats["frames_skip_tf"] += 1
            self.stats["last_skip_reason"] = "link_tf_miss:" + str(e)
            rospy.logwarn_throttle(2.0, "skip cloud: link TF at stamp failed: %s", e)
            return

        T = self._tf_matrix(tfm)
        raw = []
        for i, p in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if i % self.skip != 0:
                continue
            v = np.array([p[0], p[1], p[2], 1.0])
            w = T.dot(v)
            if self.z_min < w[2] < self.z_max:
                raw.append((float(w[0]), float(w[1]), float(w[2])))
            if len(raw) > 120000:
                break

        if not raw:
            self.stats["frames_skip_empty"] += 1
            self.stats["last_skip_reason"] = "empty_after_z"
            return

        kept, removed = filter_points(raw, link_T, inflate=self.inflate)
        self.stats["frames_ok"] += 1
        self.stats["last_raw"] = len(raw)
        self.stats["last_kept"] = len(kept)
        self.stats["last_removed"] = int(removed)
        self.stats["last_stamp"] = stamp.to_sec()
        self.stats["last_skip_reason"] = None
        self.stats["use_sim_time"] = bool(rospy.get_param("/use_sim_time", False))

        self.pub_raw.publish(make_cloud(raw, stamp, "world"))
        self.accum.add_many(kept)
        self._last_filt = list(self.accum.points())
        self.stats["accum_n"] = len(self.accum)
        self.stats["ready"] = self.stats["accum_n"] >= 500 and self.stats["frames_ok"] >= 2

        filt_cloud = make_cloud(self._last_filt, stamp, "world")
        self.pub_filt.publish(filt_cloud)
        self.pub.publish(filt_cloud)
        self.pub_stats.publish(String(data=json.dumps(self.stats)))

    def _tick(self, _evt):
        # Keep latch fresh for late GridMap subscribers
        if not self._last_filt:
            return
        cloud = make_cloud(self._last_filt, rospy.Time.now(), "world")
        self.pub.publish(cloud)
        self.stats["accum_n"] = len(self._last_filt)
        self.pub_stats.publish(String(data=json.dumps(self.stats)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic-shelves", action="store_true")
    ap.add_argument("--skip", type=int, default=6)
    ap.add_argument("--voxel", type=float, default=0.05)
    ap.add_argument("--max-voxels", type=int, default=250000)
    ap.add_argument("--self-inflate", type=float, default=0.03)
    # z-min must clear Ranger deck/ground ESDF false collisions (car spheres ~z=0.05–0.25).
    # 0.12 keeps table tops/legs and shelf faces; rejects floor returns.
    ap.add_argument("--z-min", type=float, default=0.12)
    ap.add_argument("--z-max", type=float, default=3.5)
    args, _ = ap.parse_known_args()
    rospy.init_node("warehouse_cloud_to_remani")
    Feeder(args)
    rospy.spin()


if __name__ == "__main__":
    main()
