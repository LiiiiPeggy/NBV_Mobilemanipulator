#!/usr/bin/env python3
"""Mode B kinematic joint bridge (REMANI-compatible).

Holds/tracks CR10 via /gazebo/set_model_configuration at high rate (same class of
kinematic plant as REMANI fake_mm). Publishes /mm/mani/joint_state from Gazebo
/joint_states. Subscribes /mm_controller_node/joint_cmd for REMANI tracking.

Does not publish Mode A fake /joint_states and does not claim effort-motor physics.
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
WHEELS = [
    "fr_steering_joint",
    "fr_wheel_joint",
    "fl_steering_wheel_joint",
    "fl_wheel_joint",
    "rl_steering_wheel_joint",
    "rl_wheel_joint",
    "rr_steering_wheel_joint",
    "rr_wheel_joint",
    "gripper_finger1_joint",
]


class JointBridge(object):
    def __init__(self, hold_pose, model_name):
        self.model = model_name
        self.cmd = {k: float(v) for k, v in (hold_pose or {}).items() if k in ARM}
        for j in ARM:
            self.cmd.setdefault(j, 0.0)
        for j in WHEELS:
            self.cmd.setdefault(j, 0.0)
        self.last_js = dict(self.cmd)
        rospy.wait_for_service("/gazebo/set_model_configuration", timeout=60)
        self.set_cfg = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
        self.pub_mm = rospy.Publisher("/mm/mani/joint_state", JointState, queue_size=1)
        self.pub_grip = rospy.Publisher("/mm/mani/gripper_state", Bool, queue_size=1, latch=True)
        self.pub_grip.publish(Bool(data=True))
        rospy.Subscriber("/joint_states", JointState, self._on_js, queue_size=1)
        # Some gazebo_ros builds publish under /<model>/joint_states despite robotNamespace.
        rospy.Subscriber(
            "/%s/joint_states" % model_name, JointState, self._on_js, queue_size=1
        )
        self.pub_js = rospy.Publisher("/joint_states", JointState, queue_size=1)
        rospy.Subscriber(
            "/mm_controller_node/joint_cmd",
            JointTrajectoryControllerState,
            self._on_cmd,
            queue_size=1,
        )
        self.timer = rospy.Timer(rospy.Duration(0.02), self._tick)

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.last_js[n] = float(p)
        # Ensure global /joint_states exists for RSP even if Gazebo uses model ns.
        if msg.name:
            out_all = JointState()
            out_all.header.stamp = rospy.Time.now()
            out_all.name = list(msg.name)
            out_all.position = list(msg.position)
            out_all.velocity = list(msg.velocity) if msg.velocity else []
            self.pub_js.publish(out_all)
        out = JointState()
        out.header.stamp = rospy.Time.now()
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
        names = list(ARM) + list(WHEELS)
        pos = [float(self.cmd.get(j, 0.0)) for j in names]
        try:
            self.set_cfg(self.model, "", names, pos)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "set_model_configuration failed: %s", e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm-pose", default="")
    ap.add_argument("--model", default="agx_ranger_cr10")
    args, _ = ap.parse_known_args()
    rospy.init_node("agx_mode_b_joint_bridge")
    pose = {}
    if args.arm_pose:
        pose = yaml.safe_load(Path(args.arm_pose).read_text()) or {}
    JointBridge(pose, args.model)
    rospy.loginfo("Mode B kinematic joint bridge (hold=%s)", sorted(pose.keys()))
    rospy.spin()


if __name__ == "__main__":
    main()
