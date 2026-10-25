#!/usr/bin/env python3
"""NBV Mode B trajectory visualization (TopAY / remani_real preview style).

Publishes:
  /nbv/trajectory/ee_planned   (yellow Path)  — FK from REMANI PolynomialTraj
  /nbv/trajectory/ee_actual    (cyan Path)    — TF world→ee_frame (Gazebo/RSP)
  /nbv/trajectory/base_actual  (blue Path)    — /ground_truth/state
  /nbv/visualization/nbv_goal  (green Marker) — NBV camera goal + look axis

Planned EE uses URDF chain FK to cr10_Link6 so it overlays Gazebo TF.
REMANI /ee_current_pose uses a different internal EE; do not mix frames.
"""
from __future__ import print_function

import argparse
import math
from pathlib import Path as FsPath

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PolynomialTraj
from visualization_msgs.msg import Marker


def parse_joint(urdf_text, name):
    import re

    block = re.search(r'<joint[^>]*name="%s"[^>]*>.*?</joint>' % name, urdf_text, re.S)
    if not block:
        raise RuntimeError("joint %s not in URDF" % name)
    b = block.group(0)
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    axis = [0.0, 0.0, 1.0]
    m = re.search(r'<origin[^>]*xyz="([^"]+)"', b)
    if m:
        xyz = [float(x) for x in m.group(1).split()]
    m = re.search(r'<origin[^>]*rpy="([^"]+)"', b)
    if m:
        rpy = [float(x) for x in m.group(1).split()]
    m = re.search(r'<axis[^>]*xyz="([^"]+)"', b)
    if m:
        axis = [float(x) for x in m.group(1).split()]
    return xyz, rpy, axis


def load_ee_chain(urdf_path):
    text = FsPath(urdf_path).read_text()
    names = [
        "cr10_mount_joint",
        "cr10_joint1",
        "cr10_joint2",
        "cr10_joint3",
        "cr10_joint4",
        "cr10_joint5",
        "cr10_joint6",
    ]
    movable = set(names[1:])
    specs = []
    for n in names:
        xyz, rpy, axis = parse_joint(text, n)
        specs.append({"name": n, "xyz": xyz, "rpy": rpy, "axis": axis, "movable": n in movable})
    return specs


def T_origin(xyz, rpy):
    T = tft.euler_matrix(rpy[0], rpy[1], rpy[2], "sxyz")
    T[0:3, 3] = xyz
    return T


def axis_angle(axis, q):
    a = np.asarray(axis, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-9:
        return np.eye(3)
    a = a / n
    return tft.rotation_matrix(q, a)[0:3, 0:3]


def fk_link6(specs, qdict, base_xyz, base_yaw):
    T = tft.euler_matrix(0.0, 0.0, base_yaw, "sxyz")
    T[0:3, 3] = base_xyz
    for s in specs:
        T = T @ T_origin(s["xyz"], s["rpy"])
        if s["movable"]:
            R = np.eye(4)
            R[0:3, 0:3] = axis_angle(s["axis"], float(qdict.get(s["name"], 0.0)))
            T = T @ R
    return T


def _coeff_mat(piece):
    """Eigen column-major CoefficientMat (rows=num_dim or 8, cols=degree+1)."""
    degree = int(piece.num_order)
    cols = degree + 1
    data = np.asarray(piece.data, dtype=float)
    rows = data.size // cols
    return data.reshape((rows, cols), order="F"), degree


def eval_piece(piece, t):
    """Match poly_traj_utils Piece::getPos: col(degree)=c0, col(0)=c_degree."""
    coeff, degree = _coeff_mat(piece)
    pos = np.zeros(coeff.shape[0])
    tn = 1.0
    for i in range(degree, -1, -1):
        pos = pos + tn * coeff[:, i]
        tn *= t
    return pos


def eval_piece_vel(piece, t):
    coeff, degree = _coeff_mat(piece)
    vel = np.zeros(coeff.shape[0])
    tn = 1.0
    power = 1
    for i in range(degree - 1, -1, -1):
        vel = vel + power * tn * coeff[:, i]
        tn *= t
        power += 1
    return vel


def sample_traj_msgs(msg, dt=0.05):
    """Yield (t_abs, pos_vec, yaw) over ACTION_ADD pieces."""
    if msg.action != PolynomialTraj.ACTION_ADD or not msg.trajectory:
        return
    t_abs = 0.0
    singul = float(msg.singul) if msg.singul != 0 else 1.0
    for piece in msg.trajectory:
        dur = float(piece.duration)
        if dur <= 0.0:
            continue
        n = max(1, int(math.ceil(dur / dt)))
        for k in range(n + 1):
            t = min(dur, k * dur / float(n))
            pos = eval_piece(piece, t)
            vel = eval_piece_vel(piece, t)
            yaw = 0.0
            if np.linalg.norm(vel[:2]) > 1e-6:
                yaw = math.atan2(singul * vel[1], singul * vel[0])
            yield t_abs + t, pos, yaw
        t_abs += dur


class TrajViz(object):
    def __init__(self, specs, ee_frame, base_z, max_actual=5000):
        self.specs = specs
        self.ee_frame = ee_frame
        self.base_z = float(base_z)
        self.max_actual = int(max_actual)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)

        self.pub_ee_plan = rospy.Publisher("/nbv/trajectory/ee_planned", Path, queue_size=1, latch=True)
        self.pub_ee_act = rospy.Publisher("/nbv/trajectory/ee_actual", Path, queue_size=1, latch=True)
        self.pub_base_act = rospy.Publisher("/nbv/trajectory/base_actual", Path, queue_size=1, latch=True)
        self.pub_goal = rospy.Publisher("/nbv/visualization/nbv_goal", Marker, queue_size=1, latch=True)

        self.ee_actual = Path()
        self.ee_actual.header.frame_id = "world"
        self.base_actual = Path()
        self.base_actual.header.frame_id = "world"
        self.metrics = {
            "planned_points": 0,
            "ee_actual_points": 0,
            "base_actual_points": 0,
            "last_goal": None,
        }

        rospy.Subscriber("/planning/trajectory", PolynomialTraj, self._on_traj, queue_size=20)
        rospy.Subscriber("/ground_truth/state", Odometry, self._on_odom, queue_size=20)
        rospy.Subscriber("/nbv/goal_pose", PoseStamped, self._on_nbv_goal, queue_size=1)
        rospy.Subscriber("/ee_goal", PoseStamped, self._on_ee_goal_as_nbv, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick_tf)

    def _append(self, path, x, y, z, qx=0, qy=0, qz=0, qw=1):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "world"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        path.poses.append(ps)
        if len(path.poses) > self.max_actual:
            path.poses = path.poses[-self.max_actual :]

    def _on_traj(self, msg):
        if msg.action != PolynomialTraj.ACTION_ADD:
            return
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "world"
        last_yaw = 0.0
        for _t, pos, yaw in sample_traj_msgs(msg, dt=0.05):
            if yaw != 0.0:
                last_yaw = yaw
            # state: [x, y, j1..j6] (dim>=8) — joints at indices 2:8
            if pos.size < 8:
                continue
            qdict = {
                "cr10_joint1": float(pos[2]),
                "cr10_joint2": float(pos[3]),
                "cr10_joint3": float(pos[4]),
                "cr10_joint4": float(pos[5]),
                "cr10_joint5": float(pos[6]),
                "cr10_joint6": float(pos[7]),
            }
            T = fk_link6(self.specs, qdict, [float(pos[0]), float(pos[1]), self.base_z], last_yaw)
            p = T[0:3, 3]
            q = tft.quaternion_from_matrix(T)
            self._append(path, p[0], p[1], p[2], q[0], q[1], q[2], q[3])
        self.metrics["planned_points"] = len(path.poses)
        self.pub_ee_plan.publish(path)
        rospy.loginfo_throttle(2.0, "ee_planned points=%d", len(path.poses))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.base_actual.header.stamp = rospy.Time.now()
        self._append(self.base_actual, p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        self.metrics["base_actual_points"] = len(self.base_actual.poses)
        self.pub_base_act.publish(self.base_actual)

    def _tick_tf(self, _evt):
        try:
            t = self.buf.lookup_transform(
                "world", self.ee_frame, rospy.Time(0), rospy.Duration(0.05)
            )
        except Exception:
            return
        p = t.transform.translation
        o = t.transform.rotation
        self.ee_actual.header.stamp = rospy.Time.now()
        # downsample: skip if < 5 mm from last
        if self.ee_actual.poses:
            lp = self.ee_actual.poses[-1].pose.position
            if (p.x - lp.x) ** 2 + (p.y - lp.y) ** 2 + (p.z - lp.z) ** 2 < (0.005**2):
                return
        self._append(self.ee_actual, p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        self.metrics["ee_actual_points"] = len(self.ee_actual.poses)
        self.pub_ee_act.publish(self.ee_actual)

    def _publish_goal_marker(self, xyz, quat_xyzw, ns="nbv_goal"):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = 0.08
        m.scale.y = 0.04
        m.scale.z = 0.04
        m.color.r = 0.1
        m.color.g = 0.9
        m.color.b = 0.2
        m.color.a = 1.0
        m.pose.position.x = xyz[0]
        m.pose.position.y = xyz[1]
        m.pose.position.z = xyz[2]
        m.pose.orientation.x = quat_xyzw[0]
        m.pose.orientation.y = quat_xyzw[1]
        m.pose.orientation.z = quat_xyzw[2]
        m.pose.orientation.w = quat_xyzw[3]
        self.pub_goal.publish(m)
        # look axis as LINE_LIST child
        m2 = Marker()
        m2.header = m.header
        m2.ns = ns
        m2.id = 1
        m2.type = Marker.ARROW
        m2.action = Marker.ADD
        m2.scale.x = 0.03
        m2.scale.y = 0.06
        m2.scale.z = 0.06
        m2.color = m.color
        T = tft.quaternion_matrix(quat_xyzw)
        # optical / camera often -Z look; for EE goal use +X as arrow length via pose
        start = np.array(xyz, dtype=float)
        # Arrow marker uses pose orientation * +X as direction when points empty
        m2.pose = m.pose
        m2.scale.x = 0.35
        self.pub_goal.publish(m2)
        self.metrics["last_goal"] = list(xyz)

    def _on_nbv_goal(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        self._publish_goal_marker([p.x, p.y, p.z], [o.x, o.y, o.z, o.w], ns="nbv_goal")

    def _on_ee_goal_as_nbv(self, msg):
        # Optional: show REMANI EE goal in same marker topic (id namespace ee_goal)
        p = msg.pose.position
        o = msg.pose.orientation
        self._publish_goal_marker([p.x, p.y, p.z], [o.x, o.y, o.z, o.w], ns="ee_goal")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--urdf",
        default="/workspace/third_party/agx/rangerboxcr10lidar_description/urdf/rangercr10lidar.urdf",
    )
    ap.add_argument("--ee-frame", default="cr10_Link6")
    ap.add_argument("--base-z", type=float, default=0.423)
    args = ap.parse_args()
    rospy.init_node("visualize_remani_trajectory")
    urdf = FsPath(args.urdf)
    if not urdf.is_file():
        urdf = FsPath(__file__).resolve().parents[1] / (
            "third_party/agx/rangerboxcr10lidar_description/urdf/rangercr10lidar.urdf"
        )
    specs = load_ee_chain(urdf)
    TrajViz(specs, args.ee_frame, args.base_z)
    rospy.loginfo(
        "NBV traj viz: ee_planned/ee_actual/base_actual + nbv_goal (URDF FK → %s)",
        args.ee_frame,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
