"""
Measure the world session's feeds and render the live stats panel.

The multi-subscription node. The design decision worth internalising is
*latest-wins, no synchroniser*: cloud_projector pairs depth with its exact
source frame because projecting mismatched pairs would be wrong, so it uses
message_filters — a stats panel wants "the newest of each" from streams
that run at deliberately different rates (~30 fps camera vs ~13 fps depth),
so each callback just overwrites a slot and a wall-timer renders whatever
is there. Same toolbox, opposite choice, both correct for their job.

Rates are measured by counting arrivals against THIS node's clock. Header
stamps on this camera lag wall clock ~0.73 s by fault
(docs/info/camera.md#timestamps) — a stamp-age staleness gate would mark
live feeds dead, so staleness too is measured against receipt time only.

The panel publishes on /world/stats/compressed for the RViz stats Image
panel. Until 2026-08-12 this node also composed the feeds into a 2×2
mosaic on /world/dashboard/compressed; the per-feed RViz panels had made
that a publisher without subscribers, so the mosaic — and the decode of
every incoming frame, which existed only to feed it — was removed.
Arrival times and counts are all the stats need, so the feeds are never
decoded here at all now.

Runs on the dev box; subscribes to compressed streams and publishes one
small JPEG, so the dashboard adds almost nothing to the Wi-Fi.
"""

from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32

# RELIABLE for megabyte-class messages, same reasoning as every image
# subscription in this repo: BEST_EFFORT delivers zero frames once a message
# fragments past the socket buffer (docs/info/troubleshooting.md), and the
# RELIABLE-by-default viewers never match a BEST_EFFORT publisher.
BIG_FRAME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

# The measured feeds, in stats-panel line order.
PANELS = ('camera', 'depth', 'keypoints')

FONT = cv2.FONT_HERSHEY_SIMPLEX

GREY = (200, 200, 200)
GREEN = (0, 255, 0)
RED = (0, 0, 255)


def rates(arrivals, now, window):
    """
    Convert arrival deques to messages/sec, measured on one clock.

    Pure function: arrivals is {name: deque of receipt times in seconds},
    pruned here to the window. Rate = count over the *window*, not over
    first-to-last spacing — a stream that stopped mid-window decays toward
    zero instead of reporting its old rate forever.
    """
    result = {}
    for name, times in arrivals.items():
        while times and now - times[0] > window:
            times.popleft()
        span = min(window, now - times[0]) if times else 0.0
        # Fewer than 2 arrivals is "no measurable rate", and a tiny span
        # right after startup would explode the division.
        result[name] = (len(times) / span) if len(times) > 1 and span > 0.1 \
            else 0.0
    return result


def stats_lines(fps, totals, last_seen, now, stale_after,
                keypoint_count, matched_count, keyframe_count=None):
    """
    Build the stats panel as (text, BGR colour) lines.

    Pure function so the staleness and formatting rules are testable
    without ROS. A feed is STALE when its newest arrival — receipt time,
    never header.stamp — is older than stale_after; a feed that has never
    arrived just shows its zero rate rather than a fabricated age.
    """
    lines = [('world dashboard', GREY)]
    for name in PANELS:
        last = last_seen[name]
        age = None if last is None else now - last
        if age is not None and age > stale_after:
            lines.append((
                f'{name:<10} STALE {age:4.1f}s  total {totals[name]}', RED))
        else:
            lines.append((
                f'{name:<10} {fps[name]:5.1f}/s   total {totals[name]}',
                GREEN if fps[name] > 0 else RED))
    current = '-' if keypoint_count is None else str(keypoint_count)
    # Percentage is a display concern: the wire carries raw counts, and the
    # two slots are latest-wins so a ratio computed here can briefly pair
    # adjacent frames — fine for a human-facing gauge.
    if keypoint_count and matched_count is not None:
        matched = f'{100.0 * matched_count / keypoint_count:.0f}%'
    else:
        matched = '-'
    lines.append((f'keypoints in frame: {current}', GREY))
    lines.append((f'matched (window):   {matched}', GREY))
    # The room memory (relocalization plan): how many keyframes the
    # detector has stored. None = the topic never arrived (old detector).
    if keyframe_count is not None:
        lines.append((f'keyframes stored:   {keyframe_count}', GREY))
    return lines


def render_panel(lines, size):
    """Draw (text, colour) lines onto a black panel of (width, height)."""
    width, height = size
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    for i, (text, colour) in enumerate(lines):
        cv2.putText(panel, text, (16, 34 + 30 * i), FONT, 0.6, colour, 2)
    return panel


class Dashboard(Node):

    def __init__(self):
        super().__init__('dashboard')

        # Panel size: matches one cell of the retired 2x2 mosaic, which is
        # also a comfortable RViz side-panel size.
        self.declare_parameter('cell_width', 640)
        self.declare_parameter('cell_height', 360)
        # Rates average over this many seconds; short = jumpy, long = slow
        # to show a stall.
        self.declare_parameter('rate_window', 5.0)
        # A feed whose newest arrival (by receipt time) is older than this
        # gets a STALE line.
        self.declare_parameter('stale_after', 2.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('jpeg_quality', 80)

        self.arrivals = {name: deque() for name in PANELS}
        self.totals = {name: 0 for name in PANELS}
        self.last_seen = {name: None for name in PANELS}
        self.keypoint_count = None
        self.matched_count = None
        self.keyframe_count = None

        self.create_subscription(
            CompressedImage, 'image_raw/compressed',
            lambda msg: self.on_feed('camera', msg), BIG_FRAME_QOS)
        self.create_subscription(
            CompressedImage, 'depth/preview/compressed',
            lambda msg: self.on_feed('depth', msg), BIG_FRAME_QOS)
        self.create_subscription(
            CompressedImage, 'keypoints/compressed',
            lambda msg: self.on_feed('keypoints', msg), BIG_FRAME_QOS)
        self.create_subscription(
            Int32, 'keypoints/count', self.on_count, BIG_FRAME_QOS)
        self.create_subscription(
            Int32, 'keypoints/matched', self.on_matched, BIG_FRAME_QOS)
        self.create_subscription(
            Int32, 'keypoints/keyframes', self.on_keyframes, BIG_FRAME_QOS)

        self.pub_stats = self.create_publisher(
            CompressedImage, 'world/stats/compressed', BIG_FRAME_QOS)

        # A wall-timer, not a callback chain: the panel refreshes at its
        # own steady pace no matter which feeds are fast, slow, or absent.
        self.create_timer(
            1.0 / self.get_parameter('publish_rate').value, self.on_timer)

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_feed(self, name, _msg):
        # Receipt is the measurement; the payload is never decoded — the
        # feeds are displayed full-size from their own topics in RViz.
        now = self.now_sec()
        self.arrivals[name].append(now)
        self.totals[name] += 1
        self.last_seen[name] = now

    def on_count(self, msg):
        self.keypoint_count = msg.data

    def on_matched(self, msg):
        self.matched_count = msg.data

    def on_keyframes(self, msg):
        self.keyframe_count = msg.data

    def on_timer(self):
        now = self.now_sec()
        fps = rates(self.arrivals, now,
                    self.get_parameter('rate_window').value)
        lines = stats_lines(
            fps, self.totals, self.last_seen, now,
            self.get_parameter('stale_after').value,
            self.keypoint_count, self.matched_count, self.keyframe_count)
        panel = render_panel(lines, (self.get_parameter('cell_width').value,
                                     self.get_parameter('cell_height').value))

        out = CompressedImage()
        # Our own stamp: this image is a composition made now, not any one
        # camera frame — and the sources' stamps are known-faulty anyway.
        out.header.stamp = self.get_clock().now().to_msg()
        out.format = 'jpeg'
        out.data = cv2.imencode(
            '.jpg', panel,
            [cv2.IMWRITE_JPEG_QUALITY,
             self.get_parameter('jpeg_quality').value])[1].tobytes()
        self.pub_stats.publish(out)


def main():
    rclpy.init()
    node = Dashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
