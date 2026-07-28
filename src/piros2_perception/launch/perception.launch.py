"""
Both perception nodes on the dev box: depth estimator + cloud projector.

Deliberately does NOT include the camera launch, unlike vision.launch.py:
an IncludeLaunchDescription executes on the machine doing the launching,
and this launch runs on the dev box — including camera.launch.py here would
try to open /dev/video0 locally. The camera belongs to the Pi; `just cloud`
starts it there over SSH, same split as every recipe.

The estimator is an ExecuteProcess, not a Node, on purpose: launch_ros's
Node action execs the installed entry point, whose shebang colcon hardcodes
to the system python — which has no onnxruntime. The venv interpreter must
be named explicitly (the package README documents this trap).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

VENV_PYTHON = os.path.expanduser('~/.venvs/piros2-perception/bin/python')


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
            package='piros2_perception',
            executable='cloud_projector',
            name='cloud_projector',
            parameters=[config],
            output='screen'),
    ])
