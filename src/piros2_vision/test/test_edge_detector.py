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
Unit test for the edge detector: no camera, no graph, no discovery.

The technique worth internalising: a node's callback is just a method. Feed
it a hand-built message and capture what it would have published by replacing
the publishers — the whole DDS layer stays out of the test, which is what
makes it fast and deterministic.
"""

import cv_bridge
import numpy as np
from piros2_vision.edge_detector import EdgeDetector
import pytest
import rclpy
from sensor_msgs.msg import Image


class CapturingPublisher:
    """Stands in for rclpy's Publisher; keeps what the node publishes."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    rclpy.init()
    node = EdgeDetector()
    node.pub_image = CapturingPublisher()
    node.pub_jpeg = CapturingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def make_frame() -> Image:
    """Build a black frame with a white rectangle: edges exactly where drawn."""
    img = np.zeros((120, 160, 3), dtype=np.uint8)
    img[40:80, 50:110] = 255
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding='rgb8')
    msg.header.frame_id = 'camera_link'
    msg.header.stamp.sec = 12345
    return msg


def test_edges_are_annotated(node):
    node.on_frame(make_frame())

    assert len(node.pub_image.messages) == 1
    out = node.pub_image.messages[0]
    assert out.encoding == 'bgr8'
    assert (out.height, out.width) == (120, 160)

    annotated = cv_bridge.CvBridge().imgmsg_to_cv2(out)
    green = (annotated == (0, 255, 0)).all(axis=2)
    # The rectangle's outline must be marked green...
    assert green[40, 50:110].any(), 'no edge found along the top border'
    assert green[40:80, 50].any(), 'no edge found along the left border'
    # ...and the flat interior must not be.
    assert not green[55:65, 70:90].any(), 'edges inside a flat region'


def test_header_is_preserved(node):
    """Downstream consumers need the frame's identity, not our publish time."""
    node.on_frame(make_frame())
    out = node.pub_image.messages[0]
    assert out.header.frame_id == 'camera_link'
    assert out.header.stamp.sec == 12345


def test_compressed_variant_is_jpeg(node):
    node.on_frame(make_frame())

    assert len(node.pub_jpeg.messages) == 1
    jpeg = node.pub_jpeg.messages[0]
    assert jpeg.format == 'jpeg'
    # JPEG magic bytes — proves real encoded data, not an empty buffer.
    assert bytes(jpeg.data[:2]) == b'\xff\xd8'
    assert jpeg.header.frame_id == 'camera_link'
