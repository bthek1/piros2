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
Unit test for the keypoint detector: no camera, no graph, no discovery.

Same technique as the piros2_vision test — a callback is just a method, and
capturing publishers keeps DDS out entirely. A chessboard pattern is the
synthetic input of choice for a corner detector: ORB cannot *not* find
features on one.
"""

import cv2
import numpy as np
from piros2_world.keypoint_detector import KeypointDetector
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage


class CapturingPublisher:
    """Stands in for rclpy's Publisher; keeps what the node publishes."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    rclpy.init()
    node = KeypointDetector()
    node.pub_image = CapturingPublisher()
    node.pub_count = CapturingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def make_frame(squares=8, square_px=20) -> CompressedImage:
    """Build a JPEG chessboard frame — corners everywhere, by construction."""
    tile = np.zeros((2 * square_px, 2 * square_px), dtype=np.uint8)
    tile[:square_px, :square_px] = 255
    tile[square_px:, square_px:] = 255
    board = np.tile(tile, (squares // 2, squares // 2))
    img = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = cv2.imencode('.jpg', img)[1].tobytes()
    msg.header.frame_id = 'camera_optical_frame'
    msg.header.stamp.sec = 12345
    return msg


def test_keypoints_found_and_counted(node):
    node.on_frame(make_frame())

    assert len(node.pub_count.messages) == 1
    count = node.pub_count.messages[0].data
    assert count > 0, 'ORB found nothing on a chessboard'
    # The cap is a real bound, not a target — but the count must respect it.
    assert count <= node.get_parameter('max_features').value


def test_annotated_image_round_trips(node):
    frame = make_frame()
    node.on_frame(frame)

    assert len(node.pub_image.messages) == 1
    out = node.pub_image.messages[0]
    assert out.format == 'jpeg'
    # JPEG magic bytes — real encoded data, not an empty buffer.
    assert bytes(out.data[:2]) == b'\xff\xd8'

    original = cv2.imdecode(np.frombuffer(frame.data, np.uint8),
                            cv2.IMREAD_COLOR)
    annotated = cv2.imdecode(np.frombuffer(out.data, np.uint8),
                             cv2.IMREAD_COLOR)
    # Same geometry as the input, and visibly annotated: the overlay is
    # drawn pure green, a colour a grey chessboard cannot produce even
    # through JPEG artefacts.
    assert annotated.shape == original.shape
    greenish = (annotated[:, :, 1].astype(int)
                - annotated[:, :, 2].astype(int)) > 100
    assert greenish.any(), 'no keypoint overlay found in the output'


def test_count_matches_detection(node):
    """The published count is the detector's own answer for that frame."""
    frame = make_frame()
    node.on_frame(frame)

    img = cv2.imdecode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    expected = len(node.orb.detect(gray, None))
    assert node.pub_count.messages[0].data == expected


def test_header_is_preserved(node):
    """Downstream consumers need the frame's identity, not our publish time."""
    node.on_frame(make_frame())
    out = node.pub_image.messages[0]
    assert out.header.frame_id == 'camera_optical_frame'
    assert out.header.stamp.sec == 12345


def test_undecodable_frame_is_skipped(node):
    bad = CompressedImage()
    bad.format = 'jpeg'
    bad.data = b'not a jpeg'
    node.on_frame(bad)

    assert node.pub_image.messages == []
    assert node.pub_count.messages == []
