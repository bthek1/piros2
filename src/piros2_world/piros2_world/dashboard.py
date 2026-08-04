"""
Compose camera + depth + keypoints + live stats into one dashboard image.

The multi-subscription node. The design decision worth internalising is
*latest-wins, no synchroniser*: cloud_projector pairs depth with its exact
source frame because projecting mismatched pairs would be wrong, so it uses
message_filters — a dashboard wants "the newest of each" from streams that
run at deliberately different rates (~30 fps camera vs ~13 fps depth), so
each callback just overwrites a slot and a wall-timer composes whatever is
there. Same toolbox, opposite choice, both correct for their job.

Rates are measured by counting arrivals against THIS node's clock. Header
stamps on this camera lag wall clock ~0.73 s by fault
(docs/info/camera.md#timestamps) — a stamp-age staleness gate would mark
live panels dead, so staleness too is measured against receipt time only.

Runs on the dev box; subscribes and publishes JPEG only, so the dashboard
adds one compressed stream to the Wi-Fi, not a raw one.
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

# Panel order in the 2x2 mosaic; the fourth cell is the stats panel.
PANELS = ('camera', 'depth', 'keypoints')

FONT = cv2.FONT_HERSHEY_SIMPLEX


def compose_grid(frames, cell_size):
    """
    Resize up to three BGR frames (or None) into a 2x2 mosaic.

    Pure function: frames is {name: array-or-None} keyed by PANELS, and the
    bottom-right cell comes back black for the caller to draw stats into.
    Missing feeds become labelled placeholders, so the dashboard is honest
    about what it is not receiving.
    """
    width, height = cell_size
    cells = []
    for name in PANELS:
        frame = frames.get(name)
        if frame is None:
            cell = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(cell, f'waiting for {name}', (10, height // 2),
                        FONT, 0.7, (128, 128, 128), 2)
        else:
            cell = cv2.resize(frame, (width, height),
                              interpolation=cv2.INTER_AREA)
        cells.append(cell)
    stats_cell = np.zeros((height, width, 3), dtype=np.uint8)
    top = np.hstack((cells[0], cells[1]))
    bottom = np.hstack((cells[2], stats_cell))
    return np.vstack((top, bottom))


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


class Dashboard(Node):

    def __init__(self):
        super().__init__('dashboard')

        self.declare_parameter('cell_width', 640)
        self.declare_parameter('cell_height', 360)
        # Rates average over this many seconds; short = jumpy, long = slow
        # to show a stall.
        self.declare_parameter('rate_window', 5.0)
        # A panel whose newest frame (by receipt time) is older than this
        # gets a STALE banner.
        self.declare_parameter('stale_after', 2.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('jpeg_quality', 80)

        self.frames = {name: None for name in PANELS}
        self.arrivals = {name: deque() for name in PANELS}
        self.totals = {name: 0 for name in PANELS}
        self.last_seen = {name: None for name in PANELS}
        self.keypoint_count = None
        self.matched_count = None

        self.create_subscription(
            CompressedImage, 'image_raw/compressed',
            lambda msg: self.on_image('camera', msg), BIG_FRAME_QOS)
        self.create_subscription(
            CompressedImage, 'depth/preview/compressed',
            lambda msg: self.on_image('depth', msg), BIG_FRAME_QOS)
        self.create_subscription(
            CompressedImage, 'keypoints/compressed',
            lambda msg: self.on_image('keypoints', msg), BIG_FRAME_QOS)
        self.create_subscription(
            Int32, 'keypoints/count', self.on_count, BIG_FRAME_QOS)
        self.create_subscription(
            Int32, 'keypoints/matched', self.on_matched, BIG_FRAME_QOS)

        self.pub = self.create_publisher(
            CompressedImage, 'world/dashboard/compressed', BIG_FRAME_QOS)

        # A wall-timer, not a callback chain: the mosaic refreshes at its
        # own steady pace no matter which feeds are fast, slow, or absent.
        self.create_timer(
            1.0 / self.get_parameter('publish_rate').value, self.on_timer)

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_image(self, name, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                             cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn(f'undecodable {name} frame, skipping')
            return
        now = self.now_sec()
        self.frames[name] = frame
        self.arrivals[name].append(now)
        self.totals[name] += 1
        self.last_seen[name] = now

    def on_count(self, msg):
        self.keypoint_count = msg.data

    def on_matched(self, msg):
        self.matched_count = msg.data

    def on_timer(self):
        now = self.now_sec()
        window = self.get_parameter('rate_window').value
        stale_after = self.get_parameter('stale_after').value
        cell = (self.get_parameter('cell_width').value,
                self.get_parameter('cell_height').value)

        fps = rates(self.arrivals, now, window)
        grid = compose_grid(self.frames, cell)
        self._mark_stale(grid, cell, now, stale_after)
        self._draw_stats(grid, cell, fps)

        out = CompressedImage()
        # Our own stamp: this image is a composition made now, not any one
        # camera frame — and the sources' stamps are known-faulty anyway.
        out.header.stamp = self.get_clock().now().to_msg()
        out.format = 'jpeg'
        quality = self.get_parameter('jpeg_quality').value
        out.data = cv2.imencode(
            '.jpg', grid, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()
        self.pub.publish(out)

    def _mark_stale(self, grid, cell, now, stale_after):
        width, height = cell
        origins = {'camera': (0, 0), 'depth': (width, 0),
                   'keypoints': (0, height)}
        for name, (x, y) in origins.items():
            last = self.last_seen[name]
            if last is None or now - last <= stale_after:
                continue
            cv2.rectangle(grid, (x, y), (x + width, y + 28), (0, 0, 160), -1)
            cv2.putText(grid, f'STALE {now - last:.1f}s', (x + 8, y + 21),
                        FONT, 0.6, (255, 255, 255), 2)

    def _draw_stats(self, grid, cell, fps):
        width, height = cell
        x0, y0 = width, height  # bottom-right cell
        lines = [('world dashboard', (200, 200, 200))]
        for name in PANELS:
            lines.append((
                f'{name:<10} {fps[name]:5.1f}/s   total {self.totals[name]}',
                (0, 255, 0) if fps[name] > 0 else (0, 0, 255)))
        current = ('-' if self.keypoint_count is None
                   else str(self.keypoint_count))
        # Percentage is a display concern: the wire carries raw counts,
        # and the two slots are latest-wins so a ratio computed here can
        # briefly pair adjacent frames — fine for a human-facing gauge.
        if self.keypoint_count and self.matched_count is not None:
            matched = f'{100.0 * self.matched_count / self.keypoint_count:.0f}%'
        else:
            matched = '-'
        lines.append((f'keypoints in frame: {current}', (200, 200, 200)))
        lines.append((f'matched (window):   {matched}', (200, 200, 200)))
        for i, (text, colour) in enumerate(lines):
            cv2.putText(grid, text, (x0 + 16, y0 + 34 + 30 * i),
                        FONT, 0.6, colour, 2)


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
