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

compose_grid and rates take plain arrays and deques on purpose — most of
this file needs no ROS graph at all, matching how the perception tests keep
onnxruntime out. The node-level tests reuse the capturing-publisher trick.
"""

from collections import deque

import cv2
import numpy as np
from piros2_world.dashboard import compose_grid, Dashboard, rates
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32

CELL = (160, 90)


def solid(colour, size=(120, 200)):
    """Build a solid-colour BGR frame — input size is arbitrary on purpose."""
    frame = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    frame[:] = colour
    return frame


# --- compose_grid ---------------------------------------------------------

def test_grid_geometry_and_placement():
    frames = {'camera': solid((255, 0, 0)),
              'depth': solid((0, 255, 0)),
              'keypoints': solid((0, 0, 255))}
    grid = compose_grid(frames, CELL)

    width, height = CELL
    assert grid.shape == (2 * height, 2 * width, 3)
    # Each source lands in its own quadrant, resized to the cell.
    assert tuple(grid[height // 2, width // 2]) == (255, 0, 0)
    assert tuple(grid[height // 2, width + width // 2]) == (0, 255, 0)
    assert tuple(grid[height + height // 2, width // 2]) == (0, 0, 255)
    # Bottom-right is the stats cell — black until the node draws on it.
    assert tuple(grid[height + height // 2, width + width // 2]) == (0, 0, 0)


def test_missing_feed_gets_placeholder_not_crash():
    grid = compose_grid({'camera': solid((255, 255, 255))}, CELL)
    width, height = CELL
    # The absent depth quadrant is a placeholder: mostly dark, but labelled
    # (some non-black pixels from the "waiting for depth" text).
    quadrant = grid[:height, width:]
    assert quadrant.mean() < 32
    assert (quadrant > 0).any()


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
    node.pub = CapturingPublisher()
    node.pub_stats = CapturingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def make_image(colour=(128, 128, 128)) -> CompressedImage:
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = cv2.imencode('.jpg', solid(colour))[1].tobytes()
    return msg


def test_timer_publishes_a_jpeg_mosaic(node):
    node.on_image('camera', make_image())
    count = Int32()
    count.data = 42
    node.on_count(count)
    node.on_timer()

    assert len(node.pub.messages) == 1
    out = node.pub.messages[0]
    assert out.format == 'jpeg'
    assert bytes(out.data[:2]) == b'\xff\xd8'
    grid = cv2.imdecode(np.frombuffer(out.data, np.uint8), cv2.IMREAD_COLOR)
    assert grid.shape == (2 * node.get_parameter('cell_height').value,
                          2 * node.get_parameter('cell_width').value, 3)


def test_timer_publishes_even_with_no_feeds_at_all(node):
    """The dashboard must render its own honesty, not wait for data."""
    node.on_timer()
    assert len(node.pub.messages) == 1


def test_stats_cell_is_published_standalone(node):
    """The stats panel rides its own topic for RViz, cell-sized."""
    node.on_timer()

    assert len(node.pub_stats.messages) == 1
    out = node.pub_stats.messages[0]
    assert bytes(out.data[:2]) == b'\xff\xd8'
    stats = cv2.imdecode(np.frombuffer(out.data, np.uint8),
                         cv2.IMREAD_COLOR)
    assert stats.shape == (node.get_parameter('cell_height').value,
                           node.get_parameter('cell_width').value, 3)
    # Not an empty black cell: the rendered text is visible.
    assert (stats > 0).any()


def test_stale_panel_gets_a_banner(node):
    """A feed that stops must be flagged — measured on receipt time only."""
    node.on_image('camera', make_image((30, 30, 30)))
    # Rewind the receipt record far past the threshold; the header stamp is
    # deliberately irrelevant (the camera's stamps are faulty).
    node.last_seen['camera'] -= 10.0
    node.on_timer()

    grid = cv2.imdecode(np.frombuffer(node.pub.messages[0].data, np.uint8),
                        cv2.IMREAD_COLOR)
    banner = grid[:20, :node.get_parameter('cell_width').value]
    # The STALE banner is drawn in pure red (BGR 0,0,160); through JPEG it
    # stays strongly red-dominant, which a dark grey frame cannot be.
    reddish = (banner[:, :, 2].astype(int)
               - banner[:, :, 1].astype(int)) > 80
    assert reddish.mean() > 0.3, 'no STALE banner on a dead feed'


def test_arrivals_are_recorded_per_stream(node):
    for _ in range(3):
        node.on_image('camera', make_image())
    node.on_image('depth', make_image())

    assert node.totals == {'camera': 3, 'depth': 1, 'keypoints': 0}
    assert len(node.arrivals['camera']) == 3
    # Receipt times come from the node's own clock — the message carries a
    # stamp, but it is never read (the camera's stamps are faulty).
    assert node.last_seen['camera'] is not None
    assert node.last_seen['keypoints'] is None
