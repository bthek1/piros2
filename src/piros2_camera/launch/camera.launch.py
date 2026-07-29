"""
Launch the C922 via usb_cam: config/camera.yaml + arguments for the knobs.

What a launch file actually is: a Python module that *describes* a process
tree — it returns a LaunchDescription, and the launch service executes it.
Nothing here runs at import time except plain Python (which is why the
symlink resolution below works: it happens on the machine doing the launch,
i.e. the Pi).

The split with camera.yaml: stable facts about the camera live in the YAML;
things you legitimately vary run-to-run (resolution, frame rate, device) are
launch arguments with defaults.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The serial-keyed symlink survives replugs and reboots; /dev/video0 does not.
# usb_cam mangles symlinks (docs/info/camera.md#running-it), so resolve it here —
# realpath runs on the launching machine, which is the one with the camera.
CAMERA_BY_ID = '/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5461327F-video-index0'


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('piros2_camera'), 'config', 'camera.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value=os.path.realpath(CAMERA_BY_ID),
            description='V4L2 capture device, pre-resolved (usb_cam mangles by-id symlinks)'),
        DeclareLaunchArgument(
            'image_width', default_value='1280',
            description='Capture width; camera.md lists the real modes'),
        DeclareLaunchArgument(
            'image_height', default_value='720',
            description='Capture height'),
        DeclareLaunchArgument(
            'framerate', default_value='60.0',
            description='usb_cam poll rate, NOT the camera rate: 60 is deliberate, '
                        'the camera ceilings at ~30 and polling at 30 beats down to 24 '
                        '(docs/info/camera.md#running-it)'),
        DeclareLaunchArgument(
            'gain', default_value='-1',
            description='Sensor gain 0-255; -1 leaves the camera as-is. Auto-exposure '
                        'on the C922 never touches gain, so dim rooms need it raised '
                        '(e.g. gain:=128) — docs/info/camera.md#v4l2-controls'),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            # Must match the key in camera.yaml, or those parameters silently
            # do not apply.
            name='usb_cam',
            output='screen',
            # Later entries override earlier ones: the YAML is the baseline,
            # launch arguments win. Substitutions arrive as strings, so the
            # numeric ones declare their target type explicitly.
            parameters=[
                config,
                {
                    'video_device': LaunchConfiguration('video_device'),
                    'image_width': ParameterValue(
                        LaunchConfiguration('image_width'), value_type=int),
                    'image_height': ParameterValue(
                        LaunchConfiguration('image_height'), value_type=int),
                    'framerate': ParameterValue(
                        LaunchConfiguration('framerate'), value_type=float),
                    'gain': ParameterValue(
                        LaunchConfiguration('gain'), value_type=int),
                },
            ],
        ),

        # Where the camera IS, as data on /tf_static: base_link is the
        # robot's reference frame (REP-105), and this transform states the
        # camera's mounting pose in it once, latched, for every consumer.
        # REP-103 axes: x forward, y left, z up — so this reads "5 cm above
        # base_link's origin, facing the same way". Placeholder until the
        # camera is mounted somewhere deliberate; measure and update then.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_mount_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.05',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # The optical frame: image geometry uses z forward, x right, y down
        # (REP-103's optical convention) — NOT the body convention above.
        # This fixed rotation (the canonical rpy −90°,0,−90°) is pure
        # bookkeeping, but every projection, AprilTag pose and calibration
        # assumes it exists; image headers carry THIS frame's name.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_optical_frame',
            ],
        ),
    ])
