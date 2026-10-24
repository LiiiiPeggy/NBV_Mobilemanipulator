#!/usr/bin/env python3
"""Mode B: broadcast world→base_link from Gazebo p3d (/ground_truth/state).

Also republishes REMANI-compatible /mm/car/odom (fake_mm semantics:
  Twist.linear = (x, y, yaw), Twist.angular = (vx, vy, 0)).
Does not publish static TF. Do not run alongside Mode A agx_world_base_tf_static.
"""
from __future__ import print_function
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class P3dAuthority(object):
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.pub_odom = rospy.Publisher("/mm/car/odom", Odometry, queue_size=1)
        self.pub_odom_std = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.prev = None
        rospy.Subscriber("/ground_truth/state", Odometry, self._cb, queue_size=1)

    def _cb(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        if any(math.isnan(v) or math.isinf(v) for v in (px, py, pz)):
            rospy.logwarn_throttle(1.0, "p3d pose NaN/Inf — skip TF/odom publish")
            return
        t = TransformStamped()
        t.header.stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = px
        t.transform.translation.y = py
        t.transform.translation.z = pz
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

        yaw = yaw_from_quat(msg.pose.pose.orientation)
        vx = float(msg.twist.twist.linear.x) if not math.isnan(msg.twist.twist.linear.x) else 0.0
        vy = float(msg.twist.twist.linear.y) if not math.isnan(msg.twist.twist.linear.y) else 0.0

        # Standard odom (pose + twist)
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.pub_odom_std.publish(odom)

        # REMANI fake_mm-compatible car odom encoding
        mm = Odometry()
        mm.header = odom.header
        mm.child_frame_id = "base_link"
        mm.pose = msg.pose
        mm.twist.twist.linear.x = msg.pose.pose.position.x
        mm.twist.twist.linear.y = msg.pose.pose.position.y
        mm.twist.twist.linear.z = yaw
        mm.twist.twist.angular.x = vx
        mm.twist.twist.angular.y = vy
        mm.twist.twist.angular.z = 0.0
        self.pub_odom.publish(mm)


def main():
    rospy.init_node("agx_mode_b_p3d_tf")
    P3dAuthority()
    rospy.loginfo("Mode B p3d TF authority: /ground_truth/state → world→base_link + /mm/car/odom")
    rospy.spin()


if __name__ == "__main__":
    main()
