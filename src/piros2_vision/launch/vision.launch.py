"""
Camera + edge detector as one process tree.

The concept here is composition: this file does not restate the camera's
configuration, it *includes* piros2_camera's launch file and adds the
detector on top. Launch arguments given on the command line live in the
shared launch context, so the camera's own arguments pass straight through:

    ros2 launch piros2_vision vision.launch.py gain:=128 image_width:=640

`gain` and `image_width` are declared by the included camera launch, not
here — one owner per argument, no duplication to drift.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    camera_launch = os.path.join(
        get_package_share_directory('piros2_camera'),
        'launch', 'camera.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch)),
        Node(
            package='piros2_vision',
            executable='edge_detector',
            name='edge_detector',
            output='screen',
        ),
    ])
