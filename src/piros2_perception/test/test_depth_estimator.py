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
Unit test for the depth estimator: fake ONNX session, no weights, no graph.

Same technique as piros2_vision's test — a callback is just a method, and
capturing publishers keeps DDS out entirely. The addition here: the ML
session is injected, so the test controls exactly what "the model" returns
and 100 MB of weights (plus the venv that loads them) stay out of the test
environment. What remains under test is everything this package actually
wrote: decode, preprocessing, inversion, resizing, headers, preview.
"""

import cv2
from cv_bridge import CvBridge
import numpy as np
from piros2_perception.depth_estimator import DepthEstimator, MODEL_SIZE
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage


class FakeSession:
    """Stands in for onnxruntime; returns a fixed vertical gradient."""

    class _Input:
        name = 'pixel_values'

    def __init__(self):
        self.last_feed = None
        # Inverse depth rising down the image: top of frame "far" (0.5),
        # bottom "near" (5.0) — a floor seen from a standing camera.
        column = np.linspace(0.5, 5.0, MODEL_SIZE, dtype=np.float32)
        self.output = np.tile(column[:, np.newaxis], (1, MODEL_SIZE))

    def get_inputs(self):
        return [self._Input()]

    def run(self, _outputs, feed):
        self.last_feed = feed
        return [self.output[np.newaxis]]


class CapturingPublisher:
    """Stands in for rclpy's Publisher; keeps what the node publishes."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    rclpy.init()
    node = DepthEstimator(session=FakeSession())
    node.pub_depth = CapturingPublisher()
    node.pub_preview = CapturingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def make_frame(width=160, height=120) -> CompressedImage:
    """Build a JPEG-compressed grey frame, headers as the camera sets them."""
    img = np.full((height, width, 3), 128, dtype=np.uint8)
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = cv2.imencode('.jpg', img)[1].tobytes()
    msg.header.frame_id = 'camera_optical_frame'
    msg.header.stamp.sec = 12345
    return msg


def test_depth_shape_encoding_and_header(node):
    node.on_frame(make_frame())

    assert len(node.pub_depth.messages) == 1
    out = node.pub_depth.messages[0]
    # 32FC1 at the *input* resolution: depth aligns 1:1 with the frame even
    # though the model saw a 518x518 resample.
    assert out.encoding == '32FC1'
    assert (out.height, out.width) == (120, 160)
    assert out.header.frame_id == 'camera_optical_frame'
    assert out.header.stamp.sec == 12345


def test_inverse_depth_is_inverted(node):
    node.on_frame(make_frame())

    depth = CvBridge().imgmsg_to_cv2(node.pub_depth.messages[0])
    # The fake model said "bottom of frame is closest" in inverse depth;
    # published *distance* must therefore grow towards the top.
    assert depth[10, :].mean() > depth[110, :].mean()
    assert (depth > 0).all()


def test_preprocessing_is_imagenet_normalised(node):
    node.on_frame(make_frame())

    x = node.session.last_feed['pixel_values']
    assert x.shape == (1, 3, MODEL_SIZE, MODEL_SIZE)
    assert x.dtype == np.float32
    # Mid-grey in, so every channel must sit within ImageNet-normalised
    # range rather than raw 0..255 — the classic missed-normalisation bug.
    assert abs(x).max() < 3.0


def test_preview_is_jpeg_with_header(node):
    node.on_frame(make_frame())

    assert len(node.pub_preview.messages) == 1
    preview = node.pub_preview.messages[0]
    assert preview.format == 'jpeg'
    assert bytes(preview.data[:2]) == b'\xff\xd8'
    assert preview.header.frame_id == 'camera_optical_frame'


@pytest.fixture
def rgb_node():
    # publish_rgb reaches the node as a global parameter override — the
    # same route the world_mesh launch uses (`-p publish_rgb:=true`).
    rclpy.init(args=['--ros-args', '-p', 'publish_rgb:=true'])
    node = DepthEstimator(session=FakeSession())
    assert node.pub_rgb is not None  # the param wired the publisher up
    node.pub_depth = CapturingPublisher()
    node.pub_preview = CapturingPublisher()
    node.pub_rgb = CapturingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_no_rgb_twin_by_default(node):
    node.on_frame(make_frame())

    assert node.pub_rgb is None


def test_rgb_twin_is_stamp_identical_to_depth(rgb_node):
    rgb_node.on_frame(make_frame())

    assert len(rgb_node.pub_rgb.messages) == 1
    rgb = rgb_node.pub_rgb.messages[0]
    depth = rgb_node.pub_depth.messages[0]
    assert rgb.encoding == 'bgr8'
    assert (rgb.height, rgb.width) == (depth.height, depth.width)
    # The whole point: exact-sync consumers must find a pair for every
    # depth frame, so the twin carries the depth's header verbatim.
    assert rgb.header.stamp == depth.header.stamp
    assert rgb.header.frame_id == depth.header.frame_id


def test_max_rate_skips_back_to_back_frames(node):
    node.set_parameters([rclpy.parameter.Parameter(
        'max_rate', rclpy.parameter.Parameter.Type.DOUBLE, 5.0)])
    node.on_frame(make_frame())
    node.on_frame(make_frame())  # arrives "immediately" — inside 200 ms

    # The second frame must be skipped before decode: one depth out.
    assert len(node.pub_depth.messages) == 1


def test_unlimited_rate_processes_every_frame(node):
    node.on_frame(make_frame())
    node.on_frame(make_frame())

    assert len(node.pub_depth.messages) == 2


def test_undecodable_frame_is_skipped(node):
    bad = CompressedImage()
    bad.format = 'jpeg'
    bad.data = b'not a jpeg'
    node.on_frame(bad)

    assert node.pub_depth.messages == []
    assert node.pub_preview.messages == []
