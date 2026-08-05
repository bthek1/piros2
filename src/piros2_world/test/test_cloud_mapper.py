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
Unit tests for the cloud mapper: pure accumulation first, then the node.

VoxelMap and rotation_from_quaternion take plain arrays on purpose — the
semantics tests need no ROS graph. The node-level tests stub the tf2
buffer (a lookup is just a method call) and reuse the capturing-publisher
trick, so the whole file runs without a camera, TF tree, or discovery.
"""

import cv2
from geometry_msgs.msg import TransformStamped
import numpy as np
from piros2_perception.cloud_projector import POINT_DTYPE
from piros2_world.cloud_mapper import (
    CloudMapper,
    rotation_from_quaternion,
    VoxelMap,
)
from piros2_world.keypoint_detector import quaternion_from_rotation
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger


def structured(xyz, colour=1.0):
    """Build a POINT_DTYPE array from an Nx3 list of coordinates."""
    xyz = np.asarray(xyz, dtype=np.float32)
    points = np.empty(len(xyz), dtype=POINT_DTYPE)
    points['x'], points['y'], points['z'] = xyz.T
    points['rgb'] = colour
    return points


# --- rotation_from_quaternion -------------------------------------------

def test_quaternion_round_trip():
    """The mapper's conversion must invert the detector's exactly."""
    assert np.allclose(rotation_from_quaternion(0., 0., 0., 1.), np.eye(3))
    truth = cv2.Rodrigues(np.array([0.3, -0.5, 0.8]))[0]
    back = rotation_from_quaternion(*quaternion_from_rotation(truth))
    assert np.allclose(back, truth, atol=1e-12)


# --- VoxelMap -----------------------------------------------------------

def test_overlapping_clouds_merge_to_the_voxel_union():
    vmap = VoxelMap(voxel_size=0.1, max_voxels=1000)
    # Two clouds sharing one voxel (the origin region), each bringing one
    # voxel of its own.
    vmap.add(structured([[0.01, 0.01, 0.01], [0.51, 0.0, 0.0]], colour=1.0))
    vmap.add(structured([[0.02, 0.02, 0.02], [0.0, 0.51, 0.0]], colour=2.0))

    assert len(vmap.voxels) == 3
    # The shared voxel took the later cloud's point: latest wins.
    assert vmap.voxels[(0, 0, 0)]['rgb'] == 2.0
    assert vmap.voxels[(5, 0, 0)]['rgb'] == 1.0


def test_revisited_voxel_updates_instead_of_duplicating():
    vmap = VoxelMap(voxel_size=0.1, max_voxels=1000)
    for colour in (1.0, 2.0, 3.0):
        vmap.add(structured([[0.05, 0.05, 0.05]], colour=colour))
    assert len(vmap.voxels) == 1
    assert vmap.as_array()['rgb'][0] == 3.0


def test_cap_stops_growth_loudly_but_keeps_updating():
    vmap = VoxelMap(voxel_size=0.1, max_voxels=2)
    first = structured([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], colour=1.0)
    assert vmap.add(first) is False          # under the cap: silent
    overflow = structured([[2.0, 0.0, 0.0]], colour=2.0)
    assert vmap.add(overflow) is True        # the one loud moment
    assert vmap.add(overflow) is False       # said once, not repeatedly
    assert len(vmap.voxels) == 2
    # Known voxels still take updates after saturation.
    vmap.add(structured([[0.05, 0.0, 0.0]], colour=9.0))
    assert vmap.voxels[(0, 0, 0)]['rgb'] == 9.0


def test_clear_resets_map_and_saturation():
    vmap = VoxelMap(voxel_size=0.1, max_voxels=1)
    vmap.add(structured([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
    assert vmap.saturated
    vmap.clear()
    assert len(vmap.voxels) == 0
    assert not vmap.saturated
    assert vmap.as_array().size == 0


# --- the node -----------------------------------------------------------

class CapturingPublisher:
    """Stands in for rclpy's Publisher; keeps what the node publishes."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class StubTfBuffer:
    """Stands in for tf2's Buffer; a lookup is just a method call."""

    def __init__(self, transform):
        self.transform = transform

    def lookup_transform(self, target, source, time):
        return self.transform


def transform(qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    tf = TransformStamped()
    tf.header.frame_id = 'odom'
    tf.child_frame_id = 'camera_optical_frame'
    tf.transform.rotation.x = qx
    tf.transform.rotation.y = qy
    tf.transform.rotation.z = qz
    tf.transform.rotation.w = qw
    return tf


def cloud_msg(points) -> PointCloud2:
    """Only what the mapper reads: the frame id and the raw bytes."""
    msg = PointCloud2()
    msg.header.frame_id = 'camera_optical_frame'
    msg.height = 1
    msg.width = points.size
    msg.point_step = POINT_DTYPE.itemsize
    msg.row_step = POINT_DTYPE.itemsize * points.size
    msg.data = points.tobytes()
    return msg


@pytest.fixture
def node():
    rclpy.init()
    node = CloudMapper()
    node.pub_map = CapturingPublisher()
    node.tf_buffer = StubTfBuffer(transform())
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_clouds_accumulate_and_republish_in_odom(node):
    node.on_cloud(cloud_msg(structured([[0.0, 0.0, 1.0], [1.0, 0.0, 2.0]])))
    node.on_timer()

    assert len(node.pub_map.messages) == 1
    out = node.pub_map.messages[0]
    assert out.header.frame_id == 'odom'
    merged = np.frombuffer(out.data, dtype=POINT_DTYPE)
    assert merged.size == 2


def test_two_orientations_widen_the_map(node):
    """The panorama effect: the same optical cloud lands in two places."""
    ahead = structured([[0.0, 0.0, 2.0]])
    node.on_cloud(cloud_msg(ahead))
    # The camera turns 90° (about an axis the point is NOT on): identical
    # optical-frame points must land in a different region of odom,
    # growing the map instead of overwriting.
    half = np.sin(np.pi / 4)
    node.tf_buffer.transform = transform(qy=half, qw=half)
    node.on_cloud(cloud_msg(ahead))

    assert len(node.map.voxels) == 2


def test_far_points_are_dropped(node):
    node.on_cloud(cloud_msg(structured([[0.0, 0.0, 2.0], [0.0, 0.0, 50.0]])))
    assert len(node.map.voxels) == 1


def test_clear_service_empties_the_map(node):
    node.on_cloud(cloud_msg(structured([[0.0, 0.0, 1.0]])))
    response = node.on_clear(None, Trigger.Response())
    assert response.success
    assert len(node.map.voxels) == 0
