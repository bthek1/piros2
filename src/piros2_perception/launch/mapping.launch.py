"""
Perception P3: RGB + neural depth into RTAB-Map on the dev box.

Runs the depth estimator plus RTAB-Map's two nodes — rgbd_odometry (visual
odometry: where has the camera moved) and rtabmap (SLAM: stitch posed
RGB-D frames into a map, close loops when a place is revisited). It does
NOT play the bag or decompress the stream; `just map` owns those, same
split as `just replay`, so the launch works against any /image_raw +
/camera_info + /depth source, live or replayed.

Feeding a *neural* mono depth to an RGB-D SLAM stack is a known-community
pattern, not a supported configuration: scale is whatever depth_scale says
it is, and the depth is smooth where a real sensor would be sharp. Expect
parameter tuning here — that is P3's stated deal.

The estimator is an ExecuteProcess under the venv interpreter for the same
reason as perception.launch.py (colcon's shebang has no onnxruntime — see
the package README).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

VENV_PYTHON = os.path.expanduser('~/.venvs/piros2-perception/bin/python')

# Both RTAB-Map nodes consume the same synced RGB-D pair; one remap table.
RTABMAP_REMAPS = [
    ('rgb/image', '/image_raw'),
    ('rgb/camera_info', '/camera_info'),
    ('depth/image', '/depth'),
]

RTABMAP_COMMON = {
    # Poses are reported for base_link (the TF chain from the bag's
    # /tf_static supplies base_link -> camera_optical_frame).
    'frame_id': 'base_link',
    # The depth node copies the RGB header verbatim, so RGB/depth stamps
    # match exactly — exact sync, no approximation window needed. Only
    # frames that got a depth estimate (~3 fps of the 30) pair up, and
    # that is the intended mapping rate.
    'approx_sync': False,
}


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('piros2_perception'),
        'config', 'perception.yaml')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[VENV_PYTHON, '-m', 'piros2_perception.depth_estimator',
                 '--ros-args', '--params-file', config],
            output='screen'),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[RTABMAP_COMMON, {
                # RTAB-Map's own parameters are strings by convention.
                # Neural depth will lose odometry sometimes; auto-reset
                # after 1 lost frame instead of freezing until a service
                # call.
                'Odom/ResetCountdown': '1',
            }],
            remappings=RTABMAP_REMAPS,
            output='screen'),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            # -d wipes ~/.ros/rtabmap.db on start: every run is a fresh
            # mapping session, which is what iterate-on-bags wants.
            arguments=['-d'],
            parameters=[RTABMAP_COMMON, {
                'subscribe_depth': True,
            }],
            remappings=RTABMAP_REMAPS,
            output='screen'),
    ])
