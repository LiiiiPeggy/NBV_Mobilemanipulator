#!/usr/bin/env python3
"""Minimal RGB-D object perception for AGX D455.

Backends:
  depth — depth clustering + HSV class heuristics (default; no GT poses)
  yolo  — ultralytics YOLOv8n if installed
  color — legacy HSV blobs only

Does NOT read Gazebo object ground-truth positions.
"""
from __future__ import print_function
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


COCO_TO_SCENE = {
    "bottle": "bottle",
    "cup": "bottle",
    "wine glass": "bottle",
    "bowl": "bowl",
    "banana": "banana",
    "apple": "apple",
    "book": "box",
    "cell phone": "box",
}


def backproject(uvz, K):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    u, v, z = uvz
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.array([x, y, z], dtype=np.float64)


def transform_points(pts, trans, rot):
    import tf.transformations as tft

    T = tft.quaternion_matrix(rot)
    T[0:3, 3] = trans
    out = []
    for p in pts:
        h = np.array([p[0], p[1], p[2], 1.0])
        w = T.dot(h)
        out.append(w[:3])
    return np.asarray(out)


def depth_to_meters(depth_img):
    d = depth_img.astype(np.float32)
    if d.ndim == 3:
        d = d[:, :, 0]
    # Gazebo RealSense often publishes 32FC1 meters; some stacks use 16UC1 mm
    finite = d[np.isfinite(d)]
    if finite.size and np.nanmedian(finite) > 50:
        d = d * 0.001
    return d


def classify_hsv(mean_hsv, size_m):
    """Frozen class heuristics from mean HSV + approximate size. No GT positions."""
    h, s, v = mean_hsv
    # traffic cone / bright orange
    if 5 <= h <= 25 and s > 80 and v > 80 and size_m[2] > 0.25:
        return "cone", 0.7
    # banana yellow elongated
    if 20 <= h <= 40 and s > 60 and size_m[0] > 0.12 and size_m[2] < 0.08:
        return "banana", 0.65
    # apple red
    if (h <= 10 or h >= 170) and s > 70 and v > 50 and max(size_m) < 0.12:
        return "apple", 0.65
    # mustard / yellow bottle
    if 15 <= h <= 40 and s > 70 and size_m[2] > 0.12:
        return "bottle", 0.7
    # blue cleanser
    if 90 <= h <= 130 and s > 40 and size_m[2] > 0.12:
        return "bottle", 0.65
    # bowl: wide flat
    if size_m[0] > 0.12 and size_m[2] < 0.10 and s < 80:
        return "bowl", 0.55
    # box / carton
    if size_m[2] > 0.08 and s < 90:
        return "box", 0.5
    # can: short cylinder-ish
    if 0.06 < size_m[2] < 0.14 and s > 30:
        return "can", 0.5
    return None, 0.0


class Perceiver(object):
    def __init__(self, backend="depth", world_tf_mode="tf", calib_T=None):
        self.backend = backend
        self.world_tf_mode = world_tf_mode  # tf | gazebo_link6 | calib
        self.calib_T = None if calib_T is None else np.array(calib_T, dtype=np.float64)
        self.bridge = CvBridge()
        self.buf = tf2_ros.Buffer(cache_time=rospy.Duration(30))
        tf2_ros.TransformListener(self.buf)
        self.model = None
        if backend == "yolo":
            from ultralytics import YOLO

            self.model = YOLO("yolov8n.pt")
            rospy.loginfo("Loaded YOLOv8n")

    def detect_yolo(self, bgr):
        res = self.model.predict(bgr, verbose=False, conf=0.25)[0]
        dets = []
        names = res.names
        if res.boxes is None:
            return dets
        for b in res.boxes:
            cls_id = int(b.cls.item())
            name = names.get(cls_id, str(cls_id))
            mapped = COCO_TO_SCENE.get(name)
            if not mapped:
                continue
            xyxy = b.xyxy[0].cpu().numpy().tolist()
            conf = float(b.conf.item())
            dets.append({"class_name": mapped, "coco": name, "xyxy": xyxy, "conf": conf})
        return dets

    def detect_depth_clusters(self, bgr, depth, K):
        import cv2

        h, w = depth.shape[:2]
        valid = np.isfinite(depth) & (depth > 0.35) & (depth < 3.2)
        # Remove large planar background by keeping local depth edges / compact blobs
        d8 = np.zeros((h, w), np.uint8)
        d8[valid] = np.clip((depth[valid] - 0.35) / 3.0 * 255.0, 0, 255).astype(np.uint8)
        # Morphological open to drop thin shelf edges
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(valid.astype(np.uint8) * 255, cv2.MORPH_OPEN, kernel)
        # Connected components on valid depth
        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        dets = []
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < 250 or area > 0.25 * h * w:
                continue
            x = int(stats[i, cv2.CC_STAT_LEFT])
            y = int(stats[i, cv2.CC_STAT_TOP])
            bw = int(stats[i, cv2.CC_STAT_WIDTH])
            bh = int(stats[i, cv2.CC_STAT_HEIGHT])
            if bw < 8 or bh < 8:
                continue
            comp = labels == i
            zs = depth[comp]
            zs = zs[np.isfinite(zs)]
            if zs.size < 50:
                continue
            zmed = float(np.median(zs))
            # Reject huge depth span (walls/shelves)
            if float(np.percentile(zs, 90) - np.percentile(zs, 10)) > 0.55:
                continue
            # Approximate metric size from bbox at median depth
            size_x = bw * zmed / K[0, 0]
            size_y = bh * zmed / K[1, 1]
            size_z = float(np.percentile(zs, 90) - np.percentile(zs, 10) + 0.02)
            mean_hsv = hsv[comp].mean(axis=0)
            cls, conf = classify_hsv(mean_hsv, (size_x, size_y, size_z))
            if not cls:
                continue
            # shrink bbox slightly
            pad = 2
            xyxy = [x + pad, y + pad, x + bw - pad, y + bh - pad]
            dets.append(
                {
                    "class_name": cls,
                    "coco": "depth_cluster",
                    "xyxy": xyxy,
                    "conf": float(conf),
                    "comp_mask": comp,
                    "z_hint": zmed,
                }
            )
        # NMS by IoU
        dets = self._nms(dets, 0.4)
        return dets

    def _nms(self, dets, iou_thr):
        if not dets:
            return []
        boxes = np.array(dets, dtype=object)
        order = sorted(range(len(dets)), key=lambda i: -dets[i]["conf"])
        keep = []
        while order:
            i = order.pop(0)
            keep.append(i)
            rest = []
            for j in order:
                if self._iou(dets[i]["xyxy"], dets[j]["xyxy"]) < iou_thr:
                    rest.append(j)
            order = rest
        return [dets[i] for i in keep]

    @staticmethod
    def _iou(a, b):
        x1 = max(a[0], b[0])
        y1 = max(a[1], b[1])
        x2 = min(a[2], b[2])
        y2 = min(a[3], b[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
        area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
        union = area_a + area_b - inter + 1e-6
        return inter / union

    def detect_color(self, bgr):
        import cv2

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        ranges = [
            ("bottle", (5, 80, 80), (25, 255, 255)),
            ("bottle", (90, 40, 40), (130, 255, 255)),
            ("banana", (20, 80, 80), (40, 255, 255)),
            ("apple", (0, 80, 80), (10, 255, 255)),
            ("box", (0, 0, 40), (180, 60, 200)),
        ]
        dets = []
        h, w = bgr.shape[:2]
        for cls, lo, hi in ranges:
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < 400:
                    continue
                x, y, bw, bh = cv2.boundingRect(c)
                if bw * bh > 0.4 * h * w:
                    continue
                dets.append(
                    {
                        "class_name": cls,
                        "coco": "color",
                        "xyxy": [x, y, x + bw, y + bh],
                        "conf": min(0.9, area / 5000.0),
                    }
                )
        return self._nms(dets, 0.4)

    def localize(self, depth, K, det, frame_id):
        x1, y1, x2, y2 = [int(v) for v in det["xyxy"]]
        h, w = depth.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w - 1, x2), min(h - 1, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        if "comp_mask" in det:
            patch_mask = det["comp_mask"][y1:y2, x1:x2]
            patch = depth[y1:y2, x1:x2]
            near = patch_mask & np.isfinite(patch) & (patch > 0.25) & (patch < 4.5)
        else:
            patch = depth[y1:y2, x1:x2]
            finite = np.isfinite(patch)
            near = finite & (patch > 0.25) & (patch < 4.5)
        ratio = float(near.mean()) if near.size else 0.0
        if ratio < 0.05:
            return None
        zs = patch[near]
        z = float(np.median(zs))
        zs = zs[np.abs(zs - z) < 0.25]
        if zs.size < 15:
            return None
        z = float(np.median(zs))
        ys, xs = np.where(near & (np.abs(patch - z) < 0.25))
        if xs.size < 15:
            return None
        u = float(np.median(xs) + x1)
        v = float(np.median(ys) + y1)
        p_cam = backproject((u, v, z), K)
        try:
            if self.world_tf_mode == "calib" and self.calib_T is not None:
                pw = self.calib_T.dot(np.array([p_cam[0], p_cam[1], p_cam[2], 1.0]))[:3]
            elif self.world_tf_mode == "gazebo_link6":
                # Mode A static freeze: Gazebo joints ≠ /joint_states TF.
                # Compose world<-optical as Gazebo(Link6) * TF(Link6<-optical).
                from gazebo_msgs.srv import GetLinkState
                import tf.transformations as tft

                get = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
                gz = get("cr10_Link6", "world")
                if not gz.success:
                    raise RuntimeError("get_link_state failed")
                gp = gz.link_state.pose.position
                go = gz.link_state.pose.orientation
                T_w_l = tft.quaternion_matrix([go.x, go.y, go.z, go.w])
                T_w_l[0:3, 3] = [gp.x, gp.y, gp.z]
                tfm = self.buf.lookup_transform("cr10_Link6", frame_id, rospy.Time(0), rospy.Duration(1.0))
                t = tfm.transform.translation
                r = tfm.transform.rotation
                T_l_c = tft.quaternion_matrix([r.x, r.y, r.z, r.w])
                T_l_c[0:3, 3] = [t.x, t.y, t.z]
                T_w_c = T_w_l.dot(T_l_c)
                pw = T_w_c.dot(np.array([p_cam[0], p_cam[1], p_cam[2], 1.0]))[:3]
            else:
                tfm = self.buf.lookup_transform("world", frame_id, rospy.Time(0), rospy.Duration(1.0))
                t = tfm.transform.translation
                r = tfm.transform.rotation
                pw = transform_points([p_cam], [t.x, t.y, t.z], [r.x, r.y, r.z, r.w])[0]
        except Exception as e:
            rospy.logwarn("TF fail %s", e)
            return None
        if pw[2] < -0.05 or pw[2] > 2.5:
            return None
        return {
            "class_name": det["class_name"],
            "confidence": det["conf"],
            "valid_depth_ratio": ratio,
            "center_cam": p_cam.tolist(),
            "center_world": pw.tolist(),
            "bbox_xyxy": det["xyxy"],
            "optical_frame": frame_id,
        }

    def run_once(self, timeout=10.0):
        rgb = rospy.wait_for_message("/realsense/rgb/image_raw", Image, timeout=timeout)
        depth = rospy.wait_for_message("/realsense/depth/image_raw", Image, timeout=timeout)
        info = rospy.wait_for_message("/realsense/depth/camera_info", CameraInfo, timeout=timeout)
        bgr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
        depth_img = depth_to_meters(self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough"))
        K = np.array(info.K, dtype=np.float64).reshape(3, 3)
        frame = info.header.frame_id or "d455_depth_optical_frame"
        t0 = time.time()
        if self.backend == "yolo":
            dets = self.detect_yolo(bgr)
        elif self.backend == "color":
            dets = self.detect_color(bgr)
        else:
            dets = self.detect_depth_clusters(bgr, depth_img, K)
        objs = []
        for d in dets:
            loc = self.localize(depth_img, K, d, frame)
            if loc:
                objs.append(loc)
        dt = time.time() - t0
        return {
            "backend": self.backend,
            "stamp": {"secs": rgb.header.stamp.secs, "nsecs": rgb.header.stamp.nsecs},
            "inference_s": dt,
            "n_detections": len(dets),
            "n_localized": len(objs),
            "objects": objs,
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["depth", "yolo", "color"], default="depth")
    ap.add_argument(
        "--world-tf-mode",
        choices=["tf", "gazebo_link6", "calib"],
        default="calib",
        help="calib: Mode A empirical T_world_cam; gazebo_link6/tf alternatives",
    )
    ap.add_argument(
        "--calib-json",
        default="/workspace/data/metrics/indoor_objects/T_world_cam_calib.json",
    )
    ap.add_argument("--out", required=True)
    ap.add_argument("--views", type=int, default=1)
    ap.add_argument("--sleep", type=float, default=1.0)
    args = ap.parse_args()
    rospy.init_node("perceive_rgbd_objects")
    backend = args.backend
    if backend == "yolo":
        try:
            import ultralytics  # noqa
        except Exception:
            rospy.logwarn("ultralytics not installed; falling back to depth backend")
            backend = "depth"
    calib_T = None
    if args.world_tf_mode == "calib":
        p = Path(args.calib_json)
        if p.is_file():
            calib_T = json.loads(p.read_text())["T"]
        else:
            rospy.logwarn("calib json missing (%s); falling back to gazebo_link6", p)
            args.world_tf_mode = "gazebo_link6"
    perc = Perceiver(backend=backend, world_tf_mode=args.world_tf_mode, calib_T=calib_T)
    rospy.sleep(1.0)
    views = []
    for i in range(args.views):
        r = perc.run_once()
        r["view_id"] = i
        views.append(r)
        print("view", i, "localized", r["n_localized"], "backend", r["backend"])
        rospy.sleep(args.sleep)
    out = {"views": views}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print("Wrote", args.out)
    print("PERCEIVE_RGBD_OK" if any(v["n_localized"] > 0 for v in views) else "PERCEIVE_RGBD_EMPTY")


if __name__ == "__main__":
    main()
