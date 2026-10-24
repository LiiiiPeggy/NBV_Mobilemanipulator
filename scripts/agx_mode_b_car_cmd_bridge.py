#!/usr/bin/env python3
"""Mode B kinematic base executor for REMANI car_cmd.

REMANI mm_controller publishes Twist where:
  linear.(x,y,z) = (x, y, yaw)  desired pose in world
  angular.(x,y)  = (vx, vy)

This node smoothly updates Gazebo model state toward that pose while preserving
current Z and keeping roll/pitch near 0 (ground contact). Arm joints are
controlled separately by agx_mode_b_joint_bridge.py.

This matches fake_mm kinematics (not wheel dynamics). It is NOT a claim of
Ackermann wheel physics; it is the REMANI-compatible execution layer for Mode B.
"""
from __future__ import print_function
import math
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Twist


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CarCmdBridge(object):
    def __init__(self, model_name):
        self.model = model_name
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.target = None
        rospy.Subscriber("/mm_controller_node/car_cmd", Twist, self._on_cmd, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.02), self._tick)

    def _on_cmd(self, msg):
        self.target = (float(msg.linear.x), float(msg.linear.y), float(msg.linear.z))

    def _tick(self, _evt):
        if self.target is None:
            return
        try:
            cur = self.get_state(self.model, "world")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "get_model_state failed: %s", e)
            return
        if not cur.success:
            return
        x, y, yaw = self.target
        z = cur.pose.position.z
        # Mild lerp to avoid instantaneous teleport jumps when commands sparsify.
        cx, cy = cur.pose.position.x, cur.pose.position.y
        alpha = 0.35
        nx = cx + alpha * (x - cx)
        ny = cy + alpha * (y - cy)
        q = quat_from_yaw(yaw)
        st = ModelState()
        st.model_name = self.model
        st.pose.position.x = nx
        st.pose.position.y = ny
        st.pose.position.z = z
        st.pose.orientation.x, st.pose.orientation.y, st.pose.orientation.z, st.pose.orientation.w = q
        st.reference_frame = "world"
        try:
            self.set_state(st)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "set_model_state failed: %s", e)


def main():
    rospy.init_node("agx_mode_b_car_cmd_bridge")
    model = rospy.get_param("~model_name", "agx_ranger_cr10")
    CarCmdBridge(model)
    rospy.loginfo("Mode B car_cmd bridge on model=%s", model)
    rospy.spin()


if __name__ == "__main__":
    main()
