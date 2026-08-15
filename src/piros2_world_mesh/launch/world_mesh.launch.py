"""
The mesh-first world session — piros2_world_mesh's whole dev-box side.

World mesh plan: a full fork of world.launch.py, running this package's
copies of the four world nodes plus the shared perception pair. Same
shape as the original (venv ExecuteProcess for the two PyPI-dependent
nodes, no camera include — the camera belongs to the Pi), with the
defaults re-posed for the surface:

- `odom` defaults to **rgbd**: the mesh is the point of this session,
  and rotation-only poses smear it the moment a hand-pan carries real
  translation (RTAB-Map measured 0.9 m of arm-arc in a "rotation-only"
  sweep). `odom:=kp` gets the compass back.
- Nodes read config/world_mesh.yaml — this package's own full config,
  free to drift from world.yaml (that freedom is why the fork exists).

`just world_mesh` (aliased `just dev`) runs it with world_mesh.rviz.
Never run alongside `just world`: same node names, same topics, same
camera.
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
    mesh_config = os.path.join(
        get_package_share_directory('piros2_world_mesh'),
        'config', 'world_mesh.yaml')
    perception_config = os.path.join(
        get_package_share_directory('piros2_perception'),
        'config', 'perception.yaml')

    odom = LaunchConfiguration('odom')
    rgbd_mode = IfCondition(
        PythonExpression(["'", odom, "' == 'rgbd'"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom', default_value='rgbd',
            description='odom → base_link source: rgbd = RTAB-Map '
                        'rgbd_odometry (6-DoF, the default here), kp = '
                        'the keypoint detector rotation-only compass'),
        ExecuteProcess(
            cmd=[VENV_PYTHON, '-m', 'piros2_perception.depth_estimator',
                 '--ros-args', '--params-file', perception_config],
            output='screen'),
        Node(
            package='piros2_world_mesh',
            executable='keypoint_detector',
            name='keypoint_detector',
            parameters=[mesh_config, {
                'publish_tf': ParameterValue(
                    PythonExpression(["'", odom, "' != 'rgbd'"]),
                    value_type=bool)}],
            output='screen'),
        # The rgbd-odometry pair (the default here): raw republisher +
        # the odometry. Params/remaps match mapping.launch.py,
        # sync-queue fix included.
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
            package='piros2_world_mesh',
            executable='dashboard',
            name='dashboard',
            parameters=[mesh_config],
            output='screen'),
        Node(
            package='piros2_perception',
            executable='cloud_projector',
            name='cloud_projector',
            parameters=[perception_config],
            output='screen'),
        Node(
            package='piros2_world_mesh',
            executable='cloud_mapper',
            name='cloud_mapper',
            parameters=[mesh_config],
            output='screen'),
        # Live TSDF + timed re-mesh. Venv ExecuteProcess for the same
        # reason as the depth estimator: open3d is PyPI-only and
        # colcon's shebang misses the venv.
        ExecuteProcess(
            cmd=[VENV_PYTHON, '-m', 'piros2_world_mesh.tsdf_mesher',
                 '--ros-args', '--params-file', mesh_config],
            output='screen'),
    ])
