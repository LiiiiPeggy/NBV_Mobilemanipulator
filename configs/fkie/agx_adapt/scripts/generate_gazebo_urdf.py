#!/usr/bin/env python3
"""Generate Gazebo-adapted URDF from AGX original (does not modify source).

Modes:
  static  — Mode A: <static>true</static>, camera only (no js/p3d plugins).
            Arm joints are converted to *fixed* with the observation pose baked
            into joint origins so Gazebo geometry matches robot_state_publisher TF.
  dynamic — Mode B: physics + gazebo joint_state + p3d (needs effort controllers later).
"""
from __future__ import print_function
import argparse
import math
import re
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None

CAMERA_BLOCK = """
  <!-- ADAPTATION: D455 depth → /realsense/* for FKIE octomap.
       CRITICAL: Gazebo depth camera looks along +X of the sensor frame.
       ROS optical frame has +Z forward. Putting sensor pose = optical rpy
       (-pi/2,0,-pi/2) makes Gazebo look along optical +X (right) — 90° error
       vs TF d455_depth_optical_frame (+Z). Keep sensor pose IDENTITY on
       d455_link (body +X = optical +Z via URDF mount) and publish with
       optical frame_id so PointCloud2/TF match rendered rays. -->
  <gazebo reference="d455_link">
    <sensor name="d455_depth" type="depth">
      <pose>0 0 0 0 0 0</pose>
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
  <!-- Arm joints baked as fixed from arm_pose yaml so Gazebo == TF at spawn. -->
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

ARM_JOINTS = [
    "cr10_joint1",
    "cr10_joint2",
    "cr10_joint3",
    "cr10_joint4",
    "cr10_joint5",
    "cr10_joint6",
    "gripper_finger1_joint",
]


def _rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]

    def mm(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    return mm(mm(Rz, Ry), Rx)


def _axis_angle_matrix(axis, q):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(q), math.sin(q)
    C = 1 - c
    return [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _matrix_to_rpy(R):
    # ZYX
    sy = math.sqrt(R[0][0] ** 2 + R[1][0] ** 2)
    if sy > 1e-8:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def bake_arm_pose_as_fixed(urdf_text, pose):
    """Convert listed revolute joints to fixed with angle baked into origin rpy."""

    def repl_joint(match):
        block = match.group(0)
        name_m = re.search(r'name="([^"]+)"', block)
        if not name_m:
            return block
        name = name_m.group(1)
        if name not in pose:
            return block
        q = float(pose[name])
        type_m = re.search(r'type="([^"]+)"', block)
        if not type_m or type_m.group(1) not in ("revolute", "continuous"):
            return block
        origin_m = re.search(
            r'<origin[^>]*xyz="([^"]+)"[^>]*rpy="([^"]+)"[^>]*/>',
            block,
        )
        if not origin_m:
            origin_m = re.search(
                r'<origin[^>]*rpy="([^"]+)"[^>]*xyz="([^"]+)"[^>]*/>',
                block,
            )
            if origin_m:
                rpy_s, xyz_s = origin_m.group(1), origin_m.group(2)
            else:
                xyz_s, rpy_s = "0 0 0", "0 0 0"
        else:
            xyz_s, rpy_s = origin_m.group(1), origin_m.group(2)
        axis_m = re.search(r'<axis[^>]*xyz="([^"]+)"', block)
        axis = [1.0, 0.0, 0.0]
        if axis_m:
            axis = [float(v) for v in axis_m.group(1).split()]
        rpy0 = [float(v) for v in rpy_s.split()]
        R0 = _rpy_matrix(*rpy0)
        Rq = _axis_angle_matrix(axis, q)
        R = _mm(R0, Rq)
        r, p, y = _matrix_to_rpy(R)
        # rebuild as fixed joint (drop limit/dynamics/axis)
        parent = re.search(r"<parent[^>]*link=\"([^\"]+)\"", block)
        child = re.search(r"<child[^>]*link=\"([^\"]+)\"", block)
        if not parent or not child:
            return block
        return (
            '  <joint name="%s" type="fixed">\n'
            '    <!-- Mode A baked q=%.6f about axis %s -->\n'
            '    <origin xyz="%s" rpy="%.8f %.8f %.8f"/>\n'
            '    <parent link="%s"/>\n'
            '    <child link="%s"/>\n'
            "  </joint>"
            % (name, q, " ".join(str(a) for a in axis), xyz_s, r, p, y, parent.group(1), child.group(1))
        )

    # Replace each arm joint block
    for jn in ARM_JOINTS:
        if jn not in pose:
            continue
        pattern = r'<joint name="%s"[\s\S]*?</joint>' % re.escape(jn)
        urdf_text, n = re.subn(pattern, repl_joint, urdf_text, count=1)
        if n != 1:
            print("WARN: joint bake miss", jn, "n=", n)
    return urdf_text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    ap.add_argument("--mode", choices=["static", "dynamic"], default="static")
    ap.add_argument(
        "--arm-pose",
        default="",
        help="YAML joint map; required for Mode A bake so Gazebo matches TF",
    )
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

    pose = {}
    if args.arm_pose:
        if yaml is None:
            raise SystemExit("PyYAML required for --arm-pose")
        pose = yaml.safe_load(Path(args.arm_pose).read_text()) or {}

    if args.mode == "static":
        if pose:
            text = bake_arm_pose_as_fixed(text, pose)
            print("Baked fixed arm joints:", sorted(k for k in pose if k in ARM_JOINTS))
        else:
            print("WARN: Mode A without --arm-pose; Gazebo arm stays at URDF zero")
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
