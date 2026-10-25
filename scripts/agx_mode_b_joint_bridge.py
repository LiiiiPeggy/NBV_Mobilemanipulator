#!/usr/bin/env python3
"""Mode B kinematic joint bridge (REMANI-compatible) — no /joint_states loop.

Data flow (strict one-way):
  Gazebo → /joint_states → robot_state_publisher
  Gazebo → /joint_states → this node → /mm/mani/joint_state
  REMANI joint_cmd → this node → set_model_configuration (ARM only)

Gazebo already publishes global /joint_states; this node never republishes it.
Wheels are not force-written (avoids fighting p3d / car_cmd bridge).
ARM is held kinematically at hold_hz (fake_mm-class plant; continuous hold avoids
gravity↔deadband limit cycles).
"""
from __future__ import print_function

import argparse
from pathlib import Path

import rospy
import yaml
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.srv import SetModelConfiguration
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

ARM = [
    "cr10_joint1",
    "cr10_joint2",
    "cr10_joint3",
    "cr10_joint4",
    "cr10_joint5",
    "cr10_joint6",
]


class JointBridge(object):
    def __init__(self, hold_pose, model_name, hold_hz=25.0):
        self.model = model_name
        self.cmd = {k: float(v) for k, v in (hold_pose or {}).items() if k in ARM}
        for j in ARM:
            self.cmd.setdefault(j, 0.0)
        self.last_js = dict(self.cmd)
        rospy.wait_for_service("/gazebo/set_model_configuration", timeout=60)
        self.set_cfg = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

        self.pub_mm = rospy.Publisher("/mm/mani/joint_state", JointState, queue_size=1)
        self.pub_grip = rospy.Publisher("/mm/mani/gripper_state", Bool, queue_size=1, latch=True)
        self.pub_grip.publish(Bool(data=True))

        rospy.Subscriber("/joint_states", JointState, self._on_js, queue_size=10)
        rospy.Subscriber(
            "/mm_controller_node/joint_cmd",
            JointTrajectoryControllerState,
            self._on_cmd,
            queue_size=1,
        )
        period = max(0.02, 1.0 / float(hold_hz))
        self.timer = rospy.Timer(rospy.Duration(period), self._tick)
        self._writes = 0

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.last_js[n] = float(p)
        out = JointState()
        out.header.stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        out.name = list(ARM)
        out.position = [self.last_js.get(j, self.cmd.get(j, 0.0)) for j in ARM]
        out.velocity = [0.0] * 6
        self.pub_mm.publish(out)

    def _on_cmd(self, msg):
        if not msg.desired.positions:
            return
        n = min(6, len(msg.desired.positions))
        for i in range(n):
            self.cmd[ARM[i]] = float(msg.desired.positions[i])

    def _tick(self, _evt):
        names = list(ARM)
        pos = [float(self.cmd[j]) for j in names]
        try:
            self.set_cfg(self.model, "", names, pos)
            self._writes += 1
        except Exception as e:
            rospy.logwarn_throttle(2.0, "set_model_configuration failed: %s", e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm-pose", default="")
    ap.add_argument("--model", default="agx_ranger_cr10")
    ap.add_argument("--hold-hz", type=float, default=25.0)
    args, _ = ap.parse_known_args()
    rospy.init_node("agx_mode_b_joint_bridge")
    pose = {}
    if args.arm_pose:
        pose = yaml.safe_load(Path(args.arm_pose).read_text()) or {}
    JointBridge(pose, args.model, hold_hz=args.hold_hz)
    rospy.loginfo(
        "Mode B joint bridge: no JS republish; ARM-only set_cfg @ %.1f Hz", args.hold_hz
    )
    rospy.spin()


if __name__ == "__main__":
    main()
