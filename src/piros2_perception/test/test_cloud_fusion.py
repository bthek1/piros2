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
Unit tests for the TSDF fusion: synthetic wall, known K, hand-fed frames.

The plan's P1/P2 claims made executable without a graph: a flat wall at
2 m must pull the cells around z = 2 to a zero crossing within a few
frames, free space in front must stay empty, occluded space behind must
stay untouched, and the weight must cap at w_max. TF is stubbed with an
identity transform, so the grid is laid out directly in camera
coordinates (z forward).
"""

import cv2
from geometry_msgs.msg import TransformStamped
import numpy as np
from piros2_perception.cloud_fusion import CloudFusion
from piros2_perception.cloud_projector import POINT_DTYPE
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

WIDTH, HEIGHT = 160, 120
FX = FY = 100.0
CX, CY = 80.0, 60.0
VOXEL = 0.04
TRUNC = 0.12
WALL = 2.0

# Grid in camera coordinates (identity TF): a slab straddling the wall.
OVERRIDES = [
    '-p', 'grid_origin:=[-0.8, -0.6, 1.6]',
    '-p', 'grid_size:=[1.6, 1.2, 0.8]',
    '-p', f'voxel_size:={VOXEL}',
    '-p', f'truncation:={TRUNC}',
    '-p', 'w_max:=5.0',
    '-p', 'w_min:=3.0',
]


class IdentityBuffer:
    """Stands in for the tf2 buffer: camera frame == map frame."""

    def lookup_transform(self, target, source, time):
        return TransformStamped()


class CapturingPublisher:

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args'] + OVERRIDES)
    node = CloudFusion()
    node.tf_buffer = IdentityBuffer()
    node.pub_map = CapturingPublisher()
    node.on_info(make_camera_info())
    yield node
    node.destroy_node()
    rclpy.shutdown()


def make_camera_info() -> CameraInfo:
    msg = CameraInfo()
    msg.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    return msg


def make_depth(metres: float) -> Image:
    msg = Image()
    msg.height, msg.width = HEIGHT, WIDTH
    msg.encoding = '32FC1'
    msg.data = np.full((HEIGHT, WIDTH), metres, np.float32).tobytes()
    msg.header.frame_id = 'camera_optical_frame'
    return msg


def make_colour(bgr=(0, 0, 255)) -> CompressedImage:
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    img[:] = bgr
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = cv2.imencode(
        '.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 100])[1].tobytes()
    return msg


def fuse(node, metres, frames):
    for _ in range(frames):
        node.on_pair(make_depth(metres), make_colour())


def surface_mask(node):
    return (node.w >= 3.0) & (np.abs(node.d) < VOXEL)


def test_wall_converges_to_two_metres(node):
    fuse(node, WALL, 5)
    surf = surface_mask(node)
    assert surf.sum() > 100
    z = node.centres[surf][:, 2]
    # Every surface cell's centre sits within one voxel of the wall.
    assert np.all(np.abs(z - WALL) <= VOXEL + 1e-6)


def test_free_space_in_front_stays_empty(node):
    fuse(node, WALL, 5)
    # Cells clearly in front of the wall were observed (weight grew) but
    # hold D at +truncation: seen-through, not surface.
    front = node.centres[..., 2] < WALL - 2 * TRUNC
    observed = front & (node.w > 0)
    assert observed.sum() > 1000
    assert np.all(node.d[observed] >= TRUNC - 1e-6)


def test_occluded_space_behind_stays_untouched(node):
    fuse(node, WALL, 5)
    # The camera never saw behind the wall; those cells must keep their
    # virgin state — this is what truncation is *for*.
    behind = node.centres[..., 2] > WALL + 2 * TRUNC
    assert behind.sum() > 1000
    assert np.all(node.w[behind] == 0.0)


def test_weight_saturates_at_w_max(node):
    fuse(node, WALL, 10)
    assert node.w.max() == pytest.approx(5.0)  # w_max override


def test_incremental_pull_moves_less_each_frame(node):
    # The push/pull itself: a cell's first observation moves D a lot,
    # the fifth barely — the running average converging.
    cell = np.unravel_index(
        np.argmin(np.abs(node.centres[..., 2] - WALL)
                  + np.abs(node.centres[..., 0])
                  + np.abs(node.centres[..., 1])), node.d.shape)
    deltas = []
    for _ in range(5):
        before = node.d[cell]
        fuse(node, WALL, 1)
        deltas.append(abs(node.d[cell] - before))
    assert deltas[0] > deltas[-1]


def test_surface_takes_the_wall_colour(node):
    fuse(node, WALL, 5)
    surf = surface_mask(node)
    rgb = node.colour[surf]
    # Pure red wall (JPEG smear allowed): R dominates, B negligible.
    assert np.all(rgb[:, 0] > 200)
    assert np.all(rgb[:, 2] < 32)


def test_seed_shell_before_any_frame(node):
    node.publish_map()
    out = node.pub_map.messages[0]
    pts = np.frombuffer(out.data, POINT_DTYPE)
    assert out.header.frame_id == 'base_link'
    assert pts.size > 0
    # Every point is the faint seed grey — nothing observed yet.
    assert np.all(pts['rgb'].view(np.uint32) == 0x003C3C3C)
    # And every point sits on one of the grid's six outer faces: the
    # seed renders as an empty room, never as points filling the volume.
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1)
    lo = node.centres_flat.min(axis=0)
    hi = node.centres_flat.max(axis=0)
    on_face = (np.abs(xyz - lo) < 1e-5) | (np.abs(xyz - hi) < 1e-5)
    assert np.all(on_face.any(axis=1))


def test_map_publishes_surface_after_fusion(node):
    fuse(node, WALL, 5)
    node.publish_map()
    pts = np.frombuffer(node.pub_map.messages[0].data, POINT_DTYPE)
    coloured = pts[pts['rgb'].view(np.uint32) != 0x003C3C3C]
    assert coloured.size > 100
    assert np.all(np.abs(coloured['z'] - WALL) <= VOXEL + 1e-6)
