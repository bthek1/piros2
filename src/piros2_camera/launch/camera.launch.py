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
import stat

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The serial-keyed symlink survives replugs and reboots; /dev/video0 does not.
# usb_cam mangles symlinks (docs/info/camera.md#running-it), so resolve it here —
# realpath runs on the launching machine, which is the one with the camera.
CAMERA_BY_ID = '/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5461327F-video-index0'


def _device_holders(device, proc='/proc'):
    """
    Return processes (other than this one) holding `device` open.

    Each holder is a '<pid> <cmdline>' string, ready for an error
    message. /proc/<pid>/fd is a directory of symlinks to everything a process has
    open, readable for your own processes without root — the same source
    `fuser` uses. Only same-user processes are visible, which is exactly
    the population that can be a leaked camera session here.
    """
    real = os.path.realpath(device)
    holders = []
    for pid in os.listdir(proc):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        fd_dir = os.path.join(proc, pid, 'fd')
        try:
            fds = os.listdir(fd_dir)
        except OSError:  # not ours, or it exited mid-scan
            continue
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) != real:
                    continue
            except OSError:
                continue
            try:
                with open(os.path.join(proc, pid, 'cmdline'), 'rb') as f:
                    cmd = f.read().replace(b'\0', b' ').decode(
                        errors='replace').strip()
            except OSError:
                cmd = '?'
            holders.append(f'{pid} {cmd}')
            break
    return holders


def _require_camera(context):
    """
    Fail the whole launch, loudly, if the capture device is not usable.

    usb_cam does NOT do this itself: given a missing device it logs one
    ERROR and then idles forever (measured 2026-07-31), and given a device
    another process is streaming it dies much later with an unexplained
    `char*` abort (a leaked session held the C922 for 37 minutes on
    2026-08-15 and every frame the "new" session saw came from the leak).
    An OpaqueFunction runs after argument resolution on the launching
    machine — the one with the camera — and a raise here aborts the launch
    with a nonzero exit before any node starts.
    """
    device = LaunchConfiguration('video_device').perform(context)
    if not os.path.exists(device):
        raise RuntimeError(
            f'camera not detected: {device} does not exist '
            f'(by-id symlink: {CAMERA_BY_ID}). Is the C922 plugged in? '
            'Check with `just camera` / `v4l2-ctl --list-devices`.')
    if not stat.S_ISCHR(os.stat(device).st_mode):
        raise RuntimeError(
            f'camera not detected: {device} exists but is not a character '
            'device, so it cannot be a V4L2 capture node.')
    holders = _device_holders(device)
    if holders:
        raise RuntimeError(
            f'camera is already in use: {device} is held open by:\n  '
            + '\n  '.join(holders)
            + '\ncapture is exclusive (docs/info/camera.md, rule 2), so '
            'another camera session is still running — probably a leaked '
            'one. Stop it (`just stragglers` sweeps both machines) and '
            'relaunch.')
    return []


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

        OpaqueFunction(function=_require_camera),

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
            # If the camera node dies mid-run (device yanked, driver fault),
            # take the whole launch down with a nonzero exit instead of
            # leaving the static transform publishers idling as if all were
            # well — same fail-loudly rule as the pre-flight check above.
            on_exit=Shutdown(reason='usb_cam exited — camera lost'),
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
