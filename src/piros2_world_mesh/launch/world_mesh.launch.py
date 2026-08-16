"""
The mesh-first world session — piros2_world_mesh's whole dev-box side.

World mesh plan: a full fork of world.launch.py, running this package's
copies of the world nodes plus the shared perception pair. Same
shape as the original (venv ExecuteProcess for the two PyPI-dependent
nodes, no camera include — the camera belongs to the Pi), with the
defaults re-posed for the surface:

- `odom` defaults to **rgbd**: the mesh is the point of this session,
  and rotation-only poses smear it the moment a hand-pan carries real
  translation (RTAB-Map measured 0.9 m of arm-arc in a "rotation-only"
  sweep). `odom:=kp` gets the compass back.
- Nodes read config/world_mesh.yaml — this package's own full config,
  free to drift from world.yaml (that freedom is why the fork exists).
- No `cloud_mapper` (removed 2026-08-15): the TSDF is this session's
  fusion accumulator, and the voxel panorama duplicated it. `just
  world` keeps the mapper and its CloudMap display.

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
        # The session's single Wi-Fi reader (see camera_relay.py): every
        # other consumer of the camera stream subscribes the relay's
        # loopback copy, because five RELIABLE readers each pulling a
        # unicast copy over the Pi's Wi-Fi collapse the link into a
        # retransmit storm — measured 2026-08-16, ~2 frames/s delivered
        # per reader against 14.7 Hz for a single reader.
        Node(
            package='piros2_world_mesh',
            executable='camera_relay',
            name='camera_relay',
            output='screen'),
        # publish_rgb: the estimator republishes the exact frame it
        # inferred on as raw /image_raw, so /depth and its RGB twin carry
        # identical stamps and rgbd_odometry's exact sync pairs at depth
        # rate (~10 Hz). The old separate 60 fps republisher dropped
        # *different* frames than the estimator and pairing limped at
        # ~2 Hz — the odom TF went seconds stale and RViz's Depth3D
        # flickered between "no transform" and rendering (2026-08-15).
        # max_rate 5: the estimator is the pipeline's tempo — 5 Hz is
        # what rgbd_odometry sustains, so its TF stays current with the
        # clouds instead of trailing a queue backlog (and the GPU does
        # half the work).
        ExecuteProcess(
            cmd=[VENV_PYTHON, '-m', 'piros2_perception.depth_estimator',
                 '--ros-args', '--params-file', perception_config,
                 '-p', 'publish_rgb:=true',
                 '-p', 'max_rate:=5.0',
                 '-r', 'image_raw/compressed:=/camera_relay/compressed'],
            output='screen'),
        Node(
            package='piros2_world_mesh',
            executable='keypoint_detector',
            name='keypoint_detector',
            parameters=[mesh_config, {
                'publish_tf': ParameterValue(
                    PythonExpression(["'", odom, "' != 'rgbd'"]),
                    value_type=bool)}],
            remappings=[('image_raw/compressed', '/camera_relay/compressed')],
            output='screen'),
        # The odometry (the default here): consumes the estimator's
        # stamp-aligned /depth/rgb + /depth (publish_rgb above — no
        # republisher node in this session). Params match
        # mapping.launch.py, sync-queue hardening included. (Shrinking
        # the queues to bound TF staleness was tried 2026-08-16 and
        # made things worse — under bursty processing the two topics
        # drop different stamps and exact sync starves; the pacing fix
        # is max_rate on the estimator above.)
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
            # rgb/image reads the estimator's local twin — NEVER remap it
            # to /image_raw: usb_cam publishes raw frames on that name
            # and the subscription would pull 2.7 MB frames over Wi-Fi
            # (found live 2026-08-16, it saturated the link).
            remappings=[('rgb/image', '/depth/rgb'),
                        ('rgb/camera_info', '/camera_info'),
                        ('depth/image', '/depth')],
            output='screen'),
        Node(
            package='piros2_world_mesh',
            executable='dashboard',
            name='dashboard',
            parameters=[mesh_config],
            remappings=[('image_raw/compressed', '/camera_relay/compressed')],
            output='screen'),
        Node(
            package='piros2_perception',
            executable='cloud_projector',
            name='cloud_projector',
            parameters=[perception_config],
            output='screen'),
        # Live TSDF + timed re-mesh. Venv ExecuteProcess for the same
        # reason as the depth estimator: open3d is PyPI-only and
        # colcon's shebang misses the venv.
        ExecuteProcess(
            cmd=[VENV_PYTHON, '-m', 'piros2_world_mesh.tsdf_mesher',
                 '--ros-args', '--params-file', mesh_config,
                 '-r', 'image_raw/compressed:=/camera_relay/compressed'],
            output='screen'),
    ])
