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
Unit tests for the dashboard: pure functions first, then the node.

rates and stats_lines take plain dicts and deques on purpose — most of
this file needs no ROS graph at all, matching how the perception tests keep
onnxruntime out. The node-level tests reuse the capturing-publisher trick.
"""

from collections import deque

import cv2
import numpy as np
from piros2_world_mesh.dashboard import Dashboard, rates, stats_lines
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32


def make_lines(**overrides):
    """Call stats_lines with a healthy one-feed baseline, then overrides."""
    args = {
        'fps': {'camera': 30.0, 'depth': 0.0, 'keypoints': 0.0},
        'totals': {'camera': 150, 'depth': 0, 'keypoints': 0},
        'last_seen': {'camera': 99.9, 'depth': None, 'keypoints': None},
        'now': 100.0,
        'stale_after': 2.0,
        'keypoint_count': None,
        'matched_count': None,
    }
    args.update(overrides)
    return stats_lines(**args)


# --- rates ----------------------------------------------------------------

def test_rate_counts_arrivals_over_the_window():
    # 30 arrivals in the last second, measured over a 5 s window in which
    # the stream ran throughout.
    times = deque(100.0 + i / 30.0 for i in range(150))
    now = 105.0
    fps = rates({'camera': times}, now, window=5.0)
    assert fps['camera'] == pytest.approx(30.0, rel=0.05)


def test_rate_prunes_and_decays_when_a_stream_stops():
    times = deque(100.0 + i / 30.0 for i in range(30))  # stopped at ~101 s
    fps = rates({'camera': times}, now=104.0, window=5.0)
    # Dead for 3 of the last 4 seconds: the rate must have decayed well
    # below the live 30/s, not report the historical spacing.
    assert 0.0 < fps['camera'] < 10.0
    # And once the window slides past the burst entirely, zero — with the
    # deque physically pruned, not just ignored.
    fps = rates({'camera': times}, now=200.0, window=5.0)
    assert fps['camera'] == 0.0
    assert len(times) == 0


def test_rate_handles_empty_and_singleton():
    fps = rates({'a': deque(), 'b': deque([99.99])}, now=100.0, window=5.0)
    assert fps == {'a': 0.0, 'b': 0.0}


# --- stats_lines ----------------------------------------------------------

def test_live_feed_line_shows_rate():
    camera = next(text for text, _ in make_lines() if 'camera' in text)
    assert '30.0/s' in camera
    assert 'STALE' not in camera


def test_stale_feed_gets_flagged_on_receipt_time_only():
    """A feed that stops must be flagged — measured on receipt time only."""
    # The newest arrival is 10 s old; nothing here ever reads a header
    # stamp (the camera's stamps are faulty).
    lines = make_lines(last_seen={'camera': 90.0, 'depth': None,
                                  'keypoints': None})
    text, colour = next(line for line in lines if 'camera' in line[0])
    assert 'STALE 10.0s' in text
    assert colour == (0, 0, 255), 'a stale feed must read as red'


def test_never_seen_feed_shows_zero_rate_not_a_fake_age():
    depth = next(text for text, _ in make_lines() if 'depth' in text)
    assert 'STALE' not in depth
    assert '0.0/s' in depth


def test_matched_percentage_is_a_display_ratio():
    lines = make_lines(keypoint_count=200, matched_count=150)
    matched = next(text for text, _ in lines if 'matched' in text)
    assert '75%' in matched
    # No counts yet: dashes, never a division by zero.
    lines = make_lines()
    assert any('-' in text for text, _ in lines if 'matched' in text)


# --- the node -------------------------------------------------------------

class CapturingPublisher:
    """Stands in for rclpy's Publisher; keeps what the node publishes."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    rclpy.init()
    node = Dashboard()
    node.pub_stats = CapturingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_timer_publishes_even_with_no_feeds_at_all(node):
    """The dashboard must render its own honesty, not wait for data."""
    node.on_timer()
    assert len(node.pub_stats.messages) == 1


def test_stats_panel_is_a_cell_sized_jpeg(node):
    node.on_feed('camera', CompressedImage())
    count = Int32()
    count.data = 42
    node.on_count(count)
    node.on_timer()

    out = node.pub_stats.messages[0]
    assert out.format == 'jpeg'
    assert bytes(out.data[:2]) == b'\xff\xd8'
    stats = cv2.imdecode(np.frombuffer(out.data, np.uint8),
                         cv2.IMREAD_COLOR)
    assert stats.shape == (node.get_parameter('cell_height').value,
                           node.get_parameter('cell_width').value, 3)
    # Not an empty black cell: the rendered text is visible.
    assert (stats > 0).any()


def test_arrivals_are_recorded_per_stream(node):
    for _ in range(3):
        node.on_feed('camera', CompressedImage())
    node.on_feed('depth', CompressedImage())

    assert node.totals == {'camera': 3, 'depth': 1, 'keypoints': 0}
    assert len(node.arrivals['camera']) == 3
    # Receipt times come from the node's own clock — the message carries a
    # stamp, but it is never read (the camera's stamps are faulty).
    assert node.last_seen['camera'] is not None
    assert node.last_seen['keypoints'] is None


def test_keyframe_line_appears_only_when_reported():
    lines = make_lines(keyframe_count=17)
    assert any('keyframes' in text and '17' in text for text, _ in lines)
    # An old detector that never publishes the topic: no fabricated row.
    lines = make_lines()
    assert not any('keyframes' in text for text, _ in lines)
