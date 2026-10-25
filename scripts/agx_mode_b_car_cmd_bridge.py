#!/usr/bin/env python3
"""Mode B kinematic base executor for REMANI car_cmd (fake_mm semantics).

REMANI mm_controller publishes Twist where:
  linear.(x,y,z) = (x, y, yaw)  desired pose in world
  angular.(x,y)  = (vx, vy)

Idle / settled behaviour (validated against free-roll):
  Ranger wheels are not braked under ODE. With zero external pose writes the
  chassis rolls away (10+ cm in seconds). Soft-hold snaps @ 2 Hz create a
  sawtooth that raises XY std. Therefore:

  - Latch a hold pose as soon as Gazebo reports the model (before any car_cmd).
  - While tracking a new target: exponential lerp.
  - Once inside deadband for settle_ticks: write the *exact* target every tick
    (authoritative kinematic plant). No competing writers on base pose.

Not Ackermann wheel dynamics — REMANI-compatible kinematic plant for Mode B.
"""
from __future__ import print_function

import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Twist


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class CarCmdBridge(object):
    def __init__(
        self,
        model_name,
        xy_deadband=0.01,
        yaw_deadband_deg=1.0,
        alpha=0.35,
        settle_ticks=10,
    ):
        self.model = model_name
        self.xy_deadband = float(xy_deadband)
        self.yaw_deadband = math.radians(float(yaw_deadband_deg))
        self.xy_resume = 2.0 * self.xy_deadband
        self.yaw_resume = 2.0 * self.yaw_deadband
        self.alpha = float(alpha)
        self.settle_need = int(settle_ticks)
        self.target = None  # (x, y, yaw)
        self._settled = False
        self._settle = 0
        self._writes = 0
        self._skips = 0
        self._from_cmd = False
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.Subscriber("/mm_controller_node/car_cmd", Twist, self._on_cmd, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.02), self._tick)
        self._stats_timer = rospy.Timer(rospy.Duration(5.0), self._log_stats)

    def _on_cmd(self, msg):
        nxt = (float(msg.linear.x), float(msg.linear.y), float(msg.linear.z))
        self._from_cmd = True
        if self.target is None:
            self.target = nxt
            self._settle = 0
            self._settled = False
            return
        dx = nxt[0] - self.target[0]
        dy = nxt[1] - self.target[1]
        dyaw = ang_diff(nxt[2], self.target[2])
        # Ignore spam of identical hold commands (mm_controller ~100 Hz)
        if (
            abs(dx) < 0.5 * self.xy_deadband
            and abs(dy) < 0.5 * self.xy_deadband
            and abs(dyaw) < 0.5 * self.yaw_deadband
        ):
            return
        self.target = nxt
        self._settle = 0
        self._settled = False

    def _log_stats(self, _evt):
        rospy.loginfo(
            "car_cmd bridge writes=%d skips=%d settled=%s from_cmd=%s target=%s",
            self._writes,
            self._skips,
            self._settled,
            self._from_cmd,
            None if self.target is None else (round(self.target[0], 3), round(self.target[1], 3)),
        )

    def _latch_spawn_pose(self):
        try:
            cur = self.get_state(self.model, "world")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "get_model_state failed: %s", e)
            return False
        if not cur.success:
            return False
        yaw = yaw_from_quat(cur.pose.orientation)
        self.target = (cur.pose.position.x, cur.pose.position.y, yaw)
        self._settled = True
        self._settle = self.settle_need
        rospy.loginfo(
            "car_cmd bridge latched spawn hold pose x=%.3f y=%.3f yaw=%.3f",
            self.target[0],
            self.target[1],
            self.target[2],
        )
        return True

    def _tick(self, _evt):
        if self.target is None:
            if not self._latch_spawn_pose():
                return

        try:
            cur = self.get_state(self.model, "world")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "get_model_state failed: %s", e)
            return
        if not cur.success:
            return

        x, y, yaw = self.target
        cx, cy = cur.pose.position.x, cur.pose.position.y
        cyaw = yaw_from_quat(cur.pose.orientation)
        ex = x - cx
        ey = y - cy
        eyaw = ang_diff(yaw, cyaw)
        z = cur.pose.position.z

        # Idle / spawn hold: always exact-snap (prevent free-roll).
        if not self._from_cmd:
            self._apply_pose(x, y, yaw, z)
            return

        if self._settled:
            if (
                abs(ex) <= self.xy_resume
                and abs(ey) <= self.xy_resume
                and abs(eyaw) <= self.yaw_resume
            ):
                # Exact kinematic hold every tick — authoritative plant.
                self._apply_pose(x, y, yaw, z)
                return
            self._settled = False
            self._settle = 0

        if (
            abs(ex) <= self.xy_deadband
            and abs(ey) <= self.xy_deadband
            and abs(eyaw) <= self.yaw_deadband
        ):
            self._settle += 1
            if self._settle >= self.settle_need:
                self._settled = True
            self._apply_pose(x, y, yaw, z)
            return

        self._settle = 0
        nx = cx + self.alpha * ex
        ny = cy + self.alpha * ey
        nyaw = cyaw + self.alpha * eyaw
        self._apply_pose(nx, ny, nyaw, z)

    def _apply_pose(self, x, y, yaw, z):
        q = quat_from_yaw(yaw)
        st = ModelState()
        st.model_name = self.model
        st.pose.position.x = x
        st.pose.position.y = y
        st.pose.position.z = z
        st.pose.orientation.x = q[0]
        st.pose.orientation.y = q[1]
        st.pose.orientation.z = q[2]
        st.pose.orientation.w = q[3]
        st.reference_frame = "world"
        # Zero twist so ODE does not integrate residual wheel velocity.
        st.twist.linear.x = 0.0
        st.twist.linear.y = 0.0
        st.twist.linear.z = 0.0
        st.twist.angular.x = 0.0
        st.twist.angular.y = 0.0
        st.twist.angular.z = 0.0
        try:
            self.set_state(st)
            self._writes += 1
        except Exception as e:
            rospy.logwarn_throttle(2.0, "set_model_state failed: %s", e)


def main():
    rospy.init_node("agx_mode_b_car_cmd_bridge")
    model = rospy.get_param("~model_name", "agx_ranger_cr10")
    xy_db = float(rospy.get_param("~xy_deadband", 0.01))
    yaw_db = float(rospy.get_param("~yaw_deadband_deg", 1.0))
    CarCmdBridge(model, xy_deadband=xy_db, yaw_deadband_deg=yaw_db)
    rospy.loginfo(
        "Mode B car_cmd bridge model=%s xy_db=%.3f yaw_db_deg=%.1f "
        "(spawn-hold + exact snap when settled; twist zeroed)",
        model,
        xy_db,
        yaw_db,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
