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
Unit test for the camera relay.

The callback is a method and publishers are captured — the relay's one
job is to be byte- and header-invisible.
"""

from piros2_world_mesh.camera_relay import CameraRelay
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage


class CapturingPublisher:

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    rclpy.init()
    node = CameraRelay()
    node.pub = CapturingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def make_frame() -> CompressedImage:
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = b'\xff\xd8 payload bytes'
    msg.header.frame_id = 'camera_optical_frame'
    msg.header.stamp.sec = 12345
    msg.header.stamp.nanosec = 678
    return msg


def test_relay_is_byte_and_header_identical(node):
    frame = make_frame()
    node.on_frame(frame)

    assert len(node.pub.messages) == 1
    out = node.pub.messages[0]
    assert bytes(out.data) == bytes(frame.data)
    assert out.format == frame.format
    # Stamp-based consumers downstream must not be able to tell the
    # relay was in the path.
    assert out.header.stamp == frame.header.stamp
    assert out.header.frame_id == frame.header.frame_id


def test_relay_counts_arrivals(node):
    for _ in range(3):
        node.on_frame(make_frame())

    assert node.received == 3
    assert len(node.pub.messages) == 3
