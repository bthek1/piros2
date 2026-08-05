"""
The whole dev-box side of piros2_world, in the launch whose name says so.

Depth estimator + keypoint detector + dashboard + cloud projector + cloud
mapper: the projector/mapper joined in the world combined plan's merge —
one launch, not two overlapping ones that double-start shared nodes.

Deliberately does NOT include the camera launch, same reasoning as
perception.launch.py: an IncludeLaunchDescription executes on the machine
doing the launching, and this launch runs on the dev box — including
camera.launch.py here would try to open /dev/video0 locally. The camera
belongs to the Pi; `just world` starts it there over SSH.

The estimator is an ExecuteProcess, not a Node, on purpose: launch_ros's
Node action execs the installed entry point, whose shebang colcon hardcodes
to the system python — which has no onnxruntime. The venv interpreter must
be named explicitly (piros2_perception's README documents this trap).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

VENV_PYTHON = os.path.expanduser('~/.venvs/piros2-perception/bin/python')


def generate_launch_description():
    world_config = os.path.join(
        get_package_share_directory('piros2_world'),
        'config', 'world.yaml')
    perception_config = os.path.join(
        get_package_share_directory('piros2_perception'),
        'config', 'perception.yaml')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[VENV_PYTHON, '-m', 'piros2_perception.depth_estimator',
                 '--ros-args', '--params-file', perception_config],
            output='screen'),
        Node(
            package='piros2_world',
            executable='keypoint_detector',
            name='keypoint_detector',
            parameters=[world_config],
            output='screen'),
        Node(
            package='piros2_world',
            executable='dashboard',
            name='dashboard',
            parameters=[world_config],
            output='screen'),
        Node(
            package='piros2_perception',
            executable='cloud_projector',
            name='cloud_projector',
            parameters=[perception_config],
            output='screen'),
        Node(
            package='piros2_world',
            executable='cloud_mapper',
            name='cloud_mapper',
            parameters=[world_config],
            output='screen'),
    ])
