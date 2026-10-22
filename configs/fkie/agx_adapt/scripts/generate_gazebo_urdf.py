#!/usr/bin/env python3
"""Generate Gazebo-adapted URDF from AGX original (does not modify source).

Modes:
  static  — Mode A: <static>true</static>, camera only (no js/p3d plugins).
            Arm pose is applied at spawn via -J + joint_state_publisher.
  dynamic — Mode B: physics + gazebo joint_state + p3d (needs effort controllers later).
"""
from __future__ import print_function
import argparse
import re
from pathlib import Path

CAMERA_BLOCK = """
  <!-- ADAPTATION: D455 depth → /realsense/* for FKIE octomap -->
  <gazebo reference="d455_link">
    <sensor name="d455_depth" type="depth">
      <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <camera>
        <horizontal_fov>1.5184</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.15</near>
          <far>8.0</far>
        </clip>
      </camera>
      <plugin name="d455_controller" filename="libgazebo_ros_openni_kinect.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>10.0</updateRate>
        <cameraName>realsense</cameraName>
        <imageTopicName>rgb/image_raw</imageTopicName>
        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
        <depthImageTopicName>depth/image_raw</depthImageTopicName>
        <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
        <pointCloudTopicName>depth/points</pointCloudTopicName>
        <frameName>d455_depth_optical_frame</frameName>
        <pointCloudCutoff>0.30</pointCloudCutoff>
        <pointCloudCutoffMax>7.5</pointCloudCutoffMax>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
        <CxPrime>0</CxPrime>
        <Cx>0</Cx>
        <Cy>0</Cy>
        <focalLength>0</focalLength>
        <hackBaseline>0</hackBaseline>
      </plugin>
    </sensor>
  </gazebo>
"""

DYNAMIC_PLUGINS = """
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <robotNamespace></robotNamespace>
      <jointName>
        fr_steering_joint, fr_wheel_joint, fl_steering_wheel_joint, fl_wheel_joint,
        rl_steering_wheel_joint, rl_wheel_joint, rr_steering_wheel_joint, rr_wheel_joint,
        cr10_joint1, cr10_joint2, cr10_joint3, cr10_joint4, cr10_joint5, cr10_joint6,
        gripper_finger1_joint
      </jointName>
      <updateRate>50.0</updateRate>
    </plugin>
    <plugin name="p3d_base" filename="libgazebo_ros_p3d.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>50.0</updateRate>
      <bodyName>base_link</bodyName>
      <topicName>ground_truth/state</topicName>
      <frameName>world</frameName>
      <xyzOffsets>0 0 0</xyzOffsets>
      <rpyOffsets>0 0 0</rpyOffsets>
    </plugin>
  </gazebo>
"""

STATIC_TAG = """
  <!-- Mode A: static freeze — perception/NBV only; NOT whole-body motion control -->
  <gazebo>
    <static>true</static>
  </gazebo>
"""

ALIASES = """
  <link name="turret_link"/>
  <joint name="cr10_base_to_turret_alias" type="fixed">
    <parent link="cr10_base_link"/>
    <child link="turret_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
  <link name="robot/base_link"/>
  <joint name="base_to_robot_base_alias" type="fixed">
    <parent link="base_link"/>
    <child link="robot/base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    ap.add_argument("--mode", choices=["static", "dynamic"], default="static")
    args = ap.parse_args()

    text = Path(args.src).read_text()
    text = re.sub(
        r"<gazebo>\s*<!-- loading plugin: mimic joints[\s\S]*?</gazebo>",
        "<!-- ADAPTATION: mimic plugins removed -->",
        text,
        count=1,
    )
    text = re.sub(
        r'\s*<plugin[^>]*libroboticsgroup_gazebo_mimic_joint_plugin\.so[\s\S]*?</plugin>',
        "",
        text,
    )

    if args.mode == "static":
        extra = STATIC_TAG + CAMERA_BLOCK + ALIASES
    else:
        extra = DYNAMIC_PLUGINS + CAMERA_BLOCK + ALIASES

    if "</robot>" not in text:
        raise SystemExit("no </robot>")
    text = text.replace("</robot>", extra + "\n</robot>", 1)
    text = re.sub(
        r'<robot\s+name="[^"]+"',
        '<robot name="rangercr10lidar_gazebo_%s"' % args.mode,
        text,
        count=1,
    )

    dst = Path(args.dst)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(text)
    print("Wrote", dst, "mode", args.mode, "bytes", dst.stat().st_size)


if __name__ == "__main__":
    main()
