"""
The whole dev-box side of piros2_world, in the launch whose name says so.

Depth estimator + keypoint detector + dashboard + cloud projector +
cloud mapper + mesh marker + TSDF mesher: the projector/mapper joined in
the world combined plan's merge, the two mesh nodes in the live mesh
plan — one launch, not overlapping ones that double-start shared nodes.

Deliberately does NOT include the camera launch, same reasoning as
perception.launch.py: an IncludeLaunchDescription executes on the machine
doing the launching, and this launch runs on the dev box — including
camera.launch.py here would try to open /dev/video0 locally. The camera
belongs to the Pi; `just world` starts it there over SSH.

The estimator is an ExecuteProcess, not a Node, on purpose: launch_ros's
Node action execs the installed entry point, whose shebang colcon hardcodes
to the system python — which has no onnxruntime. The venv interpreter must
be named explicitly (piros2_perception's README documents this trap).

`odom:=rgbd` (live mesh plan P3, default `kp`) swaps the odometry:
RTAB-Map's rgbd_odometry publishes real 6-DoF odom → base_link (walls
stay put when the camera *translates*), the keypoint detector's compass
yields the TF slot (REP-105: one parent per frame — its `publish_tf`
goes false; the orientation topic keeps publishing), and an
image_transport republisher provides the raw /image_raw rgbd_odometry
needs — locally, from the compressed stream already crossing the Wi-Fi.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

VENV_PYTHON = os.path.expanduser('~/.venvs/piros2-perception/bin/python')


def generate_launch_description():
    world_config = os.path.join(
        get_package_share_directory('piros2_world'),
        'config', 'world.yaml')
    perception_config = os.path.join(
        get_package_share_directory('piros2_perception'),
        'config', 'perception.yaml')

    odom = LaunchConfiguration('odom')
    rgbd_mode = IfCondition(
        PythonExpression(["'", odom, "' == 'rgbd'"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom', default_value='kp',
            description='odom → base_link source: kp = the keypoint '
                        'detector rotation-only compass, rgbd = '
                        'RTAB-Map rgbd_odometry (6-DoF)'),
        ExecuteProcess(
            cmd=[VENV_PYTHON, '-m', 'piros2_perception.depth_estimator',
                 '--ros-args', '--params-file', perception_config],
            output='screen'),
        Node(
            package='piros2_world',
            executable='keypoint_detector',
            name='keypoint_detector',
            parameters=[world_config, {
                'publish_tf': ParameterValue(
                    PythonExpression(["'", odom, "' != 'rgbd'"]),
                    value_type=bool)}],
            output='screen'),
        # The rgbd-odometry alternative (P3): raw republisher + the
        # odometry, both only in rgbd mode. Params/remaps match
        # mapping.launch.py, sync-queue fix included.
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher',
            condition=rgbd_mode,
            parameters=[{'in_transport': 'compressed',
                         'out_transport': 'raw'}],
            remappings=[('in/compressed', '/image_raw/compressed'),
                        ('out', '/image_raw')],
            output='screen'),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            condition=rgbd_mode,
            parameters=[{
                'frame_id': 'base_link',
                'approx_sync': False,
                'topic_queue_size': 30,
                'sync_queue_size': 30,
                'Odom/ResetCountdown': '1',
            }],
            remappings=[('rgb/image', '/image_raw'),
                        ('rgb/camera_info', '/camera_info'),
                        ('depth/image', '/depth')],
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
        # Live TSDF + timed re-mesh (live mesh plan P0/P1). Venv
        # ExecuteProcess for the same reason as the depth estimator:
        # open3d is PyPI-only and colcon's shebang misses the venv.
        ExecuteProcess(
            cmd=[VENV_PYTHON, '-m', 'piros2_world.tsdf_mesher',
                 '--ros-args', '--params-file', world_config],
            output='screen'),
    ])
