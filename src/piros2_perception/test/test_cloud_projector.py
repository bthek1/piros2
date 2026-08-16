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
Unit test for the cloud projector: synthetic depth plane, known K.

The plan's acceptance test made executable: a flat wall at 2 m, projected
through a hand-picked K matrix, must come back flat, at 2 m, and with the
geometry the pinhole model predicts. No graph, no cameras — the callback is
fed by hand and the published cloud is decoded straight from its bytes,
which also pins the PointCloud2 memory layout.
"""

import cv2
from geometry_msgs.msg import TransformStamped
import numpy as np
from piros2_perception.cloud_projector import (
    CloudProjector, POINT_DTYPE, rotation_matrix)
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import LookupException

WIDTH, HEIGHT = 160, 120
FX = FY = 100.0
CX, CY = 80.0, 60.0


class CapturingPublisher:
    """Stands in for rclpy's Publisher; keeps what the node publishes."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    rclpy.init()
    node = CloudProjector()
    node.pub_points = CapturingPublisher()
    node.on_info(make_camera_info())
    yield node
    node.destroy_node()
    rclpy.shutdown()


def make_camera_info() -> CameraInfo:
    msg = CameraInfo()
    msg.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    return msg


def make_depth(metres: float) -> Image:
    """Build a flat 32FC1 depth plane at the given distance."""
    msg = Image()
    msg.height, msg.width = HEIGHT, WIDTH
    msg.encoding = '32FC1'
    msg.data = np.full((HEIGHT, WIDTH), metres, np.float32).tobytes()
    msg.header.frame_id = 'camera_optical_frame'
    msg.header.stamp.sec = 12345
    return msg


def make_colour(bgr=(0, 0, 255)) -> CompressedImage:
    """Build a solid-colour JPEG frame (default: pure red)."""
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    img[:] = bgr
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = cv2.imencode(
        '.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 100])[1].tobytes()
    return msg


def decode(cloud_msg) -> np.ndarray:
    """Read the published byte buffer back through the declared layout."""
    return np.frombuffer(cloud_msg.data, dtype=POINT_DTYPE)


def test_flat_wall_comes_back_flat_at_two_metres(node):
    node.on_pair(make_depth(2.0), make_colour())

    assert len(node.pub_points.messages) == 1
    out = node.pub_points.messages[0]
    points = decode(out)

    # Subsample 4 over 160x120 → 40 * 30 points, all valid.
    assert points.size == (WIDTH // 4) * (HEIGHT // 4)
    assert out.width == points.size
    assert out.point_step == 16
    # Flat and at distance: every z exactly 2 m in the optical frame.
    assert np.allclose(points['z'], 2.0)


def test_projection_matches_the_pinhole_model(node):
    node.on_pair(make_depth(2.0), make_colour())
    points = decode(node.pub_points.messages[0])

    # Pixel (0, 0) is the first subsampled point:
    # x = (0 - cx) * z / fx = (0 - 80) * 2 / 100 = -1.6, y likewise -1.2.
    assert points['x'][0] == pytest.approx((0 - CX) * 2.0 / FX)
    assert points['y'][0] == pytest.approx((0 - CY) * 2.0 / FY)
    # The frame spans symmetric extents around the optical axis.
    assert points['x'].max() == pytest.approx(-points['x'].min(), abs=0.2)


def test_colour_is_packed_rgb(node):
    node.on_pair(make_depth(2.0), make_colour(bgr=(0, 0, 255)))
    points = decode(node.pub_points.messages[0])

    packed = points['rgb'].view(np.uint32)
    red = (packed >> 16) & 0xFF
    blue = packed & 0xFF
    # JPEG is lossy so allow a little smear, but red must dominate utterly.
    assert (red > 240).all()
    assert (blue < 16).all()


def test_far_clip_drops_background(node):
    node.on_pair(make_depth(25.0), make_colour())

    out = node.pub_points.messages[0]
    # A wall past far_clip (20 m) is "background, no idea" — no points.
    assert out.width == 0


def test_header_carries_the_optical_frame(node):
    node.on_pair(make_depth(2.0), make_colour())
    out = node.pub_points.messages[0]
    assert out.header.frame_id == 'camera_optical_frame'
    assert out.header.stamp.sec == 12345


def test_no_camera_info_publishes_nothing(node):
    node.k = None
    node.on_pair(make_depth(2.0), make_colour())
    assert node.pub_points.messages == []


@pytest.fixture
def odom_node():
    # output_frame reaches the node the same way the world_mesh launch
    # sets it — as a parameter override.
    rclpy.init(args=['--ros-args', '-p', 'output_frame:=odom'])
    node = CloudProjector()
    assert node.tf_buffer is not None  # the param wired TF up
    node.pub_points = CapturingPublisher()
    node.on_info(make_camera_info())
    yield node
    node.destroy_node()
    rclpy.shutdown()


class FakeBuffer:
    """Stands in for tf2's Buffer: one canned transform, or none."""

    def __init__(self, transform):
        self.transform = transform

    def lookup_transform(self, target, source, time):
        if self.transform is None:
            raise LookupException('odom does not exist yet')
        return self.transform


def make_transform(tx, ty, tz) -> TransformStamped:
    tf = TransformStamped()
    tf.transform.translation.x = tx
    tf.transform.translation.y = ty
    tf.transform.translation.z = tz
    tf.transform.rotation.w = 1.0
    return tf


def test_output_frame_poses_cloud_in_world(odom_node):
    odom_node.tf_buffer = FakeBuffer(make_transform(1.0, 2.0, 3.0))
    odom_node.on_pair(make_depth(2.0), make_colour())

    assert len(odom_node.pub_points.messages) == 1
    out = odom_node.pub_points.messages[0]
    points = decode(out)
    # Identity rotation, so the wall stays flat, shifted by the
    # translation; and the header now names the world frame while the
    # honest stamp survives.
    assert out.header.frame_id == 'odom'
    assert out.header.stamp.sec == 12345
    assert np.allclose(points['z'], 2.0 + 3.0)


def test_output_frame_without_tf_publishes_nothing(odom_node):
    odom_node.tf_buffer = FakeBuffer(None)
    odom_node.on_pair(make_depth(2.0), make_colour())

    assert odom_node.pub_points.messages == []


def test_rotation_matrix_quarter_turn_about_z():
    half = np.sin(np.pi / 4)
    rot = rotation_matrix(0.0, 0.0, half, np.cos(np.pi / 4))
    # +90° about z carries x onto y.
    assert np.allclose(rot @ np.array([1.0, 0.0, 0.0]),
                       [0.0, 1.0, 0.0], atol=1e-9)
