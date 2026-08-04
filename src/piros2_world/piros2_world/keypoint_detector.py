"""
Subscribe /image_raw/compressed, publish ORB keypoints as image + count.

The feature-detection primer node. Two concepts on top of the edge detector:

- A classical detector (ORB) is cheap enough for the full 30 fps stream on
  CPU — the contrast with the neural depth node's ~13 fps is deliberate,
  and the dashboard exists to make that gap visible.
- One frame can fan out into different *kinds* of topics: an image for
  humans and a scalar for stats. The count rides a plain std_msgs/Int32 —
  a custom message would need its own rosidl ament_cmake package, which a
  single integer does not justify.

Runs on the dev box against the compressed stream already in flight; only
JPEG ever crosses the Wi-Fi.
"""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32

# RELIABLE for megabyte-class messages, same reasoning as piros2_vision's
# edge detector: BEST_EFFORT delivers zero frames once a message fragments
# past the socket buffer (measured — docs/info/troubleshooting.md), and a
# BEST_EFFORT publisher is invisible to the RELIABLE-by-default viewers.
# Depth 1 = always the freshest frame, never a backlog.
BIG_FRAME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)


class KeypointDetector(Node):

    def __init__(self):
        super().__init__('keypoint_detector')

        # ORB's feature cap bounds the per-frame cost; the JPEG quality
        # trades Wi-Fi bytes for viewer fidelity. Both live in
        # config/world.yaml — parameters, not baked-in constants.
        self.declare_parameter('max_features', 500)
        self.declare_parameter('jpeg_quality', 80)
        self.orb = cv2.ORB_create(
            nfeatures=self.get_parameter('max_features').value)

        self.sub = self.create_subscription(
            CompressedImage, 'image_raw/compressed',
            self.on_frame, BIG_FRAME_QOS)
        # Published by hand on the conventional <topic>/compressed name so
        # stock viewers find it — image_transport is C++-only and gives a
        # Python publisher no automatic compressed variant.
        self.pub_image = self.create_publisher(
            CompressedImage, 'keypoints/compressed', BIG_FRAME_QOS)
        self.pub_count = self.create_publisher(
            Int32, 'keypoints/count', BIG_FRAME_QOS)

    def on_frame(self, msg: CompressedImage):
        entry = self.get_clock().now()

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                             cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('undecodable frame, skipping')
            return

        # ORB detects on greyscale; detect() alone is enough — descriptors
        # exist for *matching* across frames, which is out of scope here.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints = self.orb.detect(gray, None)

        # DRAW_RICH_KEYPOINTS renders size and orientation, so the overlay
        # shows *what ORB thinks it found*, not just where.
        annotated = cv2.drawKeypoints(
            frame, keypoints, None, color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Keep the incoming stamp: the keypoints describe that frame, not
        # the moment detection finished.
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        quality = self.get_parameter('jpeg_quality').value
        out.data = cv2.imencode(
            '.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()
        self.pub_image.publish(out)

        count = Int32()
        count.data = len(keypoints)
        self.pub_count.publish(count)

        # Cost measured against our own clock only — this camera's header
        # stamps lag ~0.73 s by fault (docs/info/camera.md#timestamps) and
        # prove nothing about the pipeline.
        done = self.get_clock().now()
        self.get_logger().info(
            f'{len(keypoints)} keypoints, '
            f'{(done - entry).nanoseconds / 1e6:.1f} ms/frame',
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = KeypointDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
