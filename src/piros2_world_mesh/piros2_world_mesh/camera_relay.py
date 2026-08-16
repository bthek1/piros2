"""
One Wi-Fi copy of the camera stream, fanned out locally.

DDS delivers a topic to each RELIABLE subscriber as its *own* unicast
stream, so every dev-box consumer of /image_raw/compressed pulls a full
copy across the Pi's Wi-Fi link. Measured 2026-08-16 (720p MJPG, 42-60
fps): one reader receives the full rate at ~1.3 MiB/s of link traffic;
five readers (estimator, detector, dashboard, mesher, RViz) collapse the
link into a reliable-retransmit storm — 14+ MiB/s of traffic yet ~2
frames/s actually completed per reader, which starved the whole session
(depth at 2 Hz, odometry TF seconds stale, the RViz Depth3D flicker).

This node is the session's single Wi-Fi reader: it subscribes the Pi's
compressed stream once and republishes it unchanged on
camera_relay/compressed, where every local consumer reads it over
loopback for free. Adding consumers no longer adds Wi-Fi load.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

# RELIABLE for megabyte-class messages, same reasoning as every other
# big-frame consumer in this repo: BEST_EFFORT delivers zero large frames
# once they fragment (docs/info/troubleshooting.md).
BIG_FRAME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)


class CameraRelay(Node):

    def __init__(self):
        super().__init__('camera_relay')
        self.received = 0
        self.last_logged_count = 0
        self.last_logged_at = None

        self.sub = self.create_subscription(
            CompressedImage, 'image_raw/compressed',
            self.on_frame, BIG_FRAME_QOS)
        self.pub = self.create_publisher(
            CompressedImage, 'camera_relay/compressed', BIG_FRAME_QOS)

    def on_frame(self, msg: CompressedImage):
        # Republished byte-for-byte, header included: the relay must be
        # invisible to stamp-based consumers downstream.
        self.pub.publish(msg)
        self.received += 1

        # Rate measured against our own receipt clock, never header.stamp
        # (the 0.73 s camera fault — docs/info/camera.md#timestamps).
        now = self.get_clock().now()
        if self.last_logged_at is None:
            self.last_logged_at = now
        elapsed = (now - self.last_logged_at).nanoseconds / 1e9
        if elapsed >= 5.0:
            rate = (self.received - self.last_logged_count) / elapsed
            self.get_logger().info(f'relaying {rate:.1f} frames/s')
            self.last_logged_count = self.received
            self.last_logged_at = now


def main():
    rclpy.init()
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
