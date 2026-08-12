"""
The mesh-first world session: world.launch.py with different defaults.

World mesh plan P0/P2. This launch owns no nodes — it includes
world.launch.py (same machine, dev box, so the include is safe) and
changes only the session's posture:

- `odom` defaults to **rgbd**: the surface is the point of this session,
  and rotation-only poses smear it the moment a hand-pan carries real
  translation (RTAB-Map measured 0.9 m of arm-arc in a "rotation-only"
  sweep). `odom:=kp` gets the compass back.
- `extra_params` points at world_mesh.yaml, layered after each node's
  own config — quality-biased tsdf_mesher settings live there, not in
  a fork of world.yaml.

`just world_mesh` (aliased `just dev`) runs it with world_mesh.rviz.
Never run alongside `just world`: same nodes, same topics, same camera.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('piros2_world')
    world_launch = os.path.join(share, 'launch', 'world.launch.py')
    mesh_overrides = os.path.join(share, 'config', 'world_mesh.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom', default_value='rgbd',
            description='odom → base_link source — rgbd (6-DoF, the '
                        'default here) or kp (the rotation-only compass)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={
                'odom': LaunchConfiguration('odom'),
                'extra_params': mesh_overrides,
            }.items()),
    ])
