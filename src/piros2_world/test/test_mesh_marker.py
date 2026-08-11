# Copyright 2026 Benedict Thekkel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Unit tests for the mesh marker: filesystem picking + the latched publish.

No RViz, no mesh files with real geometry — the node only ever handles
*paths* (RViz loads the file), so touching empty files with the right
extensions exercises everything.
"""

import os
import time

from piros2_world.mesh_marker import MeshMarker, newest_mesh
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


class CapturingPublisher:
    """Stands in for rclpy's Publisher; keeps what the node publishes."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def touch(path, mtime=None):
    path.write_bytes(b'')
    if mtime is not None:
        os.utime(path, (mtime, mtime))


# --- newest_mesh --------------------------------------------------------

def test_newest_mesh_picks_by_mtime_ply_only(tmp_path):
    now = time.time()
    touch(tmp_path / 'old.ply', now - 100)
    touch(tmp_path / 'new.ply', now)
    # Newer but not .ply: RViz's assimp rejects our GLBs (measured), and
    # room.json sits in the same directory.
    touch(tmp_path / 'newest.glb', now + 100)
    touch(tmp_path / 'newest_room.json', now + 100)
    assert newest_mesh(tmp_path).name == 'new.ply'


def test_newest_mesh_empty_dir(tmp_path):
    assert newest_mesh(tmp_path) is None


# --- the node -----------------------------------------------------------

@pytest.fixture
def node_in(tmp_path):
    rclpy.init()
    node = MeshMarker(parameter_overrides=[
        Parameter('meshes_dir', value=str(tmp_path))])
    node.pub = CapturingPublisher()
    yield node, tmp_path
    node.destroy_node()
    rclpy.shutdown()


def test_empty_dir_publishes_nothing_and_reload_recovers(node_in):
    node, meshes = node_in
    response = node.on_reload(None, Trigger.Response())
    assert not response.success
    assert not node.pub.messages

    touch(meshes / 'room.ply')
    response = node.on_reload(None, Trigger.Response())
    assert response.success
    marker = node.pub.messages[-1]
    assert marker.type == Marker.MESH_RESOURCE
    assert marker.mesh_resource.startswith('file://')
    assert marker.mesh_resource.endswith('room.ply')
    assert marker.mesh_use_embedded_materials
    assert marker.header.frame_id == 'odom'


def test_mesh_path_parameter_pins_a_specific_mesh(node_in, tmp_path):
    node, meshes = node_in
    touch(meshes / 'newer.glb')
    pinned = tmp_path / 'pinned.ply'
    touch(pinned)
    node.set_parameters([Parameter('mesh_path', value=str(pinned))])
    node.publish_mesh()
    assert node.pub.messages[-1].mesh_resource.endswith('pinned.ply')
