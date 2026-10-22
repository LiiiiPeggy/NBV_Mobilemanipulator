#!/usr/bin/env python3
"""Temporarily publish AGX Mode A joint_states from a pose YAML.

Does not modify arm_view_pose.yaml. Replaces /joint_states so robot_state_publisher
updates TF (Mode A has no Gazebo joint controllers). Use restore to go back.
"""
from __future__ import print_function
import argparse
import yaml
from pathlib import Path

import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelConfiguration


EXTRAS = [
    "fr_steering_joint",
    "fr_wheel_joint",
    "fl_steering_wheel_joint",
    "fl_wheel_joint",
    "rl_steering_wheel_joint",
    "rl_wheel_joint",
    "rr_steering_wheel_joint",
    "rr_wheel_joint",
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose-yaml", required=True)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--also-gazebo", action="store_true")
    args = ap.parse_args()
    pose = yaml.safe_load(Path(args.pose_yaml).read_text())
    rospy.init_node("agx_observation_pose_authority")
    if args.also_gazebo:
        rospy.wait_for_service("/gazebo/set_model_configuration", timeout=30)
        srv = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
        names = list(pose.keys())
        resp = srv("agx_ranger_cr10", "", names, [float(pose[n]) for n in names])
        print("gazebo_cfg", resp.success, resp.status_message)
    pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    rate = rospy.Rate(args.rate)
    print("Publishing observation pose from", args.pose_yaml)
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = EXTRAS + list(pose.keys())
        msg.position = [0.0] * len(EXTRAS) + [float(pose[n]) for n in pose.keys()]
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
