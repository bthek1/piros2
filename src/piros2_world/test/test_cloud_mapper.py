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

VoxelMap takes plain arrays on purpose — the semantics tests need no ROS
graph. The node-level tests stub the tf2 buffer (a lookup is just a
method call) and reuse the capturing-publisher trick, so the whole file
runs without a camera, TF tree, or discovery.
"""

from geometry_msgs.msg import TransformStamped
import numpy as np
from piros2_perception.cloud_projector import POINT_DTYPE
from piros2_world.cloud_mapper import (
    CloudMapper,
    pack_rgb,
    unpack_rgb,
    VoxelMap,
)
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger


def structured(xyz, colour=(128, 128, 128)):
    """Build a POINT_DTYPE array from an Nx3 list, one (r, g, b) colour."""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    points = np.empty(len(xyz), dtype=POINT_DTYPE)
    points['x'], points['y'], points['z'] = xyz.T
    points['rgb'] = pack_rgb(np.tile(colour, (len(xyz), 1)))
    return points


def published_colours(vmap, min_weight=0):
    """Map as {(r, g, b) rows} for order-free comparison."""
    return unpack_rgb(vmap.as_array(min_weight)['rgb'])


# --- VoxelMap -----------------------------------------------------------

def test_rgb_pack_round_trip():
    colours = np.array([[0, 0, 0], [255, 128, 1], [10, 200, 255]])
    assert np.array_equal(unpack_rgb(pack_rgb(colours)), colours)


def test_overlapping_clouds_merge_to_the_voxel_union():
    vmap = VoxelMap(voxel_size=0.1, max_voxels=1000)
    # Two clouds sharing one voxel (the origin region), each bringing one
    # voxel of its own.
    vmap.add(structured([[0.01, 0.01, 0.01], [0.51, 0.0, 0.0]],
                        colour=(100, 0, 0)))
    vmap.add(structured([[0.02, 0.02, 0.02], [0.0, 0.51, 0.0]],
                        colour=(200, 0, 0)))

    assert len(vmap) == 3
    reds = sorted(published_colours(vmap)[:, 0])
    # Own voxels keep their cloud's colour; the shared one averaged.
    assert reds == [100, 150, 200]


def test_fusion_averages_noise_toward_the_surface():
    """The doc's core claim: averaging distances is what denoises."""
    rng = np.random.default_rng(3)
    vmap = VoxelMap(voxel_size=0.1, max_voxels=10)
    sigma = 0.01
    samples = 1.05 + rng.normal(0.0, sigma, size=30)
    for z in samples:
        vmap.add(structured([[0.05, 0.05, z]]))
    assert len(vmap) == 1
    fused_error = abs(vmap.as_array()['z'][0] - 1.05)
    # The fused surface sits far closer to truth than a lone sample
    # would on average — sigma/sqrt(n) against sigma.
    assert fused_error < sigma / 2
    assert fused_error < np.abs(samples - 1.05).max()


def test_min_weight_holds_back_single_observations():
    vmap = VoxelMap(voxel_size=0.1, max_voxels=10)
    vmap.add(structured([[0.05, 0.0, 0.0]]))            # seen once
    twice = structured([[0.55, 0.0, 0.0]])
    vmap.add(twice)
    vmap.add(twice)                                     # seen twice
    assert len(vmap) == 2
    assert vmap.as_array(min_weight=0).size == 2
    assert vmap.as_array(min_weight=2).size == 1
    assert np.isclose(vmap.as_array(min_weight=2)['x'][0], 0.55)


def test_weight_cap_lets_new_evidence_displace_old():
    """A moved chair must win: capped inertia, not a full history."""
    vmap = VoxelMap(voxel_size=0.1, max_voxels=10, max_weight=4)
    for _ in range(20):
        vmap.add(structured([[0.05, 0.05, 0.05]], colour=(100, 100, 100)))
    for _ in range(20):
        vmap.add(structured([[0.05, 0.05, 0.05]], colour=(200, 200, 200)))
    # With inertia capped at 4, twenty new looks pull the mean almost
    # all the way to the new colour; uncapped it would sit near 150.
    assert published_colours(vmap)[0, 0] > 190


def test_cap_stops_growth_loudly_but_keeps_updating():
    vmap = VoxelMap(voxel_size=0.1, max_voxels=2)
    first = structured([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                       colour=(100, 0, 0))
    assert vmap.add(first) is False          # under the cap: silent
    overflow = structured([[2.0, 0.0, 0.0]])
    assert vmap.add(overflow) is True        # the one loud moment
    assert vmap.add(overflow) is False       # said once, not repeatedly
    assert len(vmap) == 2
    # Known voxels still fuse new evidence after saturation.
    vmap.add(structured([[0.05, 0.0, 0.0]], colour=(200, 0, 0)))
    assert sorted(published_colours(vmap)[:, 0]) == [100, 150]


def test_clear_resets_map_and_saturation():
    vmap = VoxelMap(voxel_size=0.1, max_voxels=1)
    vmap.add(structured([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
    assert vmap.saturated
    vmap.clear()
    assert len(vmap) == 0
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
    cloud = cloud_msg(structured([[0.0, 0.0, 1.0], [1.0, 0.0, 2.0]]))
    node.on_cloud(cloud)
    node.on_timer()
    # One look is below the default min_weight — nothing credible yet.
    first = np.frombuffer(node.pub_map.messages[0].data, dtype=POINT_DTYPE)
    assert first.size == 0

    node.on_cloud(cloud)
    node.on_timer()
    assert len(node.pub_map.messages) == 2
    out = node.pub_map.messages[1]
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

    assert len(node.map) == 2


def test_far_points_are_dropped(node):
    node.on_cloud(cloud_msg(structured([[0.0, 0.0, 2.0], [0.0, 0.0, 50.0]])))
    assert len(node.map) == 1


def test_clear_service_empties_the_map(node):
    node.on_cloud(cloud_msg(structured([[0.0, 0.0, 1.0]])))
    response = node.on_clear(None, Trigger.Response())
    assert response.success
    assert len(node.map) == 0
