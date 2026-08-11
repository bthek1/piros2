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
Unit tests for the TSDF mesher's pure functions and lazy-init contract.

open3d lives only in the perception venv, so nothing here may touch it:
the marker building and triangle cap are numpy-pure, and the node-level
tests exercise exactly the paths that run before the lazy import — which
is itself the contract under test (the module must be importable and the
node constructible on the system interpreter).
"""

import numpy as np
from piros2_world.tsdf_mesher import (
    cap_triangles,
    marker_from_mesh,
    TsdfMesher,
)
import pytest
import rclpy
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


# --- pure functions -----------------------------------------------------

def test_cap_triangles_passthrough_under_limit():
    tris = np.arange(30).reshape(10, 3)
    kept, dropped = cap_triangles(tris, 10)
    assert dropped == 0
    assert np.array_equal(kept, tris)


def test_cap_triangles_subsamples_evenly():
    tris = np.arange(300).reshape(100, 3)
    kept, dropped = cap_triangles(tris, 40)
    assert len(kept) + dropped == 100
    assert len(kept) <= 40
    # Even coverage: first and (near-)last triangles both survive.
    assert kept[0][0] == 0
    assert kept[-1][0] >= 290 - 3 * (100 // 40)


def test_marker_from_mesh_flattens_triangles():
    vertices = np.array([[0., 0., 0.], [1., 0., 0.],
                         [0., 1., 0.], [0., 0., 1.]])
    triangles = np.array([[0, 1, 2], [0, 2, 3]])
    colours = np.array([[0., 0.5, 1.], [2., -1., 0.5],
                        [1., 1., 1.], [0., 0., 0.]])
    marker = marker_from_mesh(vertices, triangles, colours,
                              'odom', Header().stamp)
    assert marker.type == Marker.TRIANGLE_LIST
    assert marker.header.frame_id == 'odom'
    # TRIANGLE_LIST: three points per triangle, verbatim.
    assert len(marker.points) == 6
    assert len(marker.colors) == 6
    assert marker.points[1].x == 1.0
    # Out-of-range colours clamp instead of wrapping.
    assert marker.colors[1].r == 1.0
    assert marker.colors[1].g == 0.0


# --- the node, before any open3d exists ---------------------------------

@pytest.fixture
def node():
    rclpy.init()
    node = TsdfMesher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_node_constructs_without_open3d(node):
    """The lazy-import contract: nothing touches open3d until a pair."""
    assert node.volume is None
    assert node.integrated == 0


def test_reset_before_first_frame_succeeds(node):
    response = node.on_reset(None, Trigger.Response())
    assert response.success
    assert node.volume is None
