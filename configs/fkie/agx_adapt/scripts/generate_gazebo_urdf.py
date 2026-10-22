#!/usr/bin/env python3
"""Generate Gazebo-adapted URDF from AGX original without modifying the source.

- Removes roboticsgroup mimic plugins (often missing in the FKIE image).
- Appends D455 depth camera ROS plugin on d455_link.
- Adds joint_state + p3d plugins and FKIE frame aliases.
"""
from __future__ import print_function
import argparse
import re
from pathlib import Path

CAMERA_PLUGIN = """
  <!-- ADAPTATION (nbv_agx): D455 depth for FKIE octomap; topics match octomap_server.launch -->
  <gazebo reference="d455_link">
    <sensor name="d455_depth" type="depth">
      <!-- Align Gazebo camera (+X) with ROS optical frame of D455 -->
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
          <near>0.1</near>
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
        <pointCloudCutoff>0.2</pointCloudCutoff>
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

  <!-- Publish joint states from Gazebo -->
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

  <!-- FKIE aliases: sampling frame + namespaced base -->
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
    ap.add_argument(
        "--src",
        default="/workspace/third_party/agx/rangerboxcr10lidar_description/urdf/rangercr10lidar.urdf",
    )
    ap.add_argument(
        "--dst",
        default="/workspace/configs/fkie/agx_adapt/urdf/rangercr10lidar_gazebo.urdf",
    )
    args = ap.parse_args()
    src = Path(args.src)
    dst = Path(args.dst)
    text = src.read_text()

    # Drop mimic joint plugins block (requires roboticsgroup_gazebo_plugins)
    text = re.sub(
        r"<gazebo>\s*<!-- loading plugin: mimic joints[\s\S]*?</gazebo>",
        "<!-- ADAPTATION: mimic plugins removed (not in FKIE image) -->",
        text,
        count=1,
    )
    # Also strip any remaining roboticsgroup plugins
    text = re.sub(
        r'\s*<plugin[^>]*libroboticsgroup_gazebo_mimic_joint_plugin\.so[\s\S]*?</plugin>',
        "",
        text,
    )

    if "</robot>" not in text:
        raise SystemExit("no </robot> in source URDF")
    text = text.replace("</robot>", CAMERA_PLUGIN + "\n</robot>", 1)

    # Rename slightly so it is clear this is generated
    text = re.sub(
        r'<robot\s+name="[^"]+"',
        '<robot name="rangercr10lidar_gazebo"',
        text,
        count=1,
    )

    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(text)
    print("Wrote", dst, "bytes", dst.stat().st_size)


if __name__ == "__main__":
    main()
