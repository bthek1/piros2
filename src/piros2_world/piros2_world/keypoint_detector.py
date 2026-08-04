"""
Subscribe /image_raw/compressed, publish ORB keypoints as image + counts.

The feature-detection primer node. Three concepts on top of the edge
detector:

- A classical detector (ORB) is cheap enough for the full 30 fps stream on
  CPU — the contrast with the neural depth node's ~13 fps is deliberate,
  and the dashboard exists to make that gap visible.
- Descriptors turn detection into *tracking*: each keypoint gets a 256-bit
  binary fingerprint of its neighbourhood, and Hamming-matching against
  the previous frame's fingerprints separates re-observed points (drawn
  green) from new ones (drawn yellow). Those matched pairs are the raw
  material of visual odometry — camera pose falls out of how they move.
- One frame can fan out into different *kinds* of topics: an image for
  humans and scalars for stats. The counts ride plain std_msgs/Int32 —
  a custom message would need its own rosidl ament_cmake package, which
  two integers do not justify.

Caveat for anyone reading the matched count as tracking quality: usb_cam's
60 Hz grab timer republishes each camera frame ~twice, so every other
"previous frame" is pixel-identical and matches trivially.

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
        # trades Wi-Fi bytes for viewer fidelity. All live in
        # config/world.yaml — parameters, not baked-in constants.
        self.declare_parameter('max_features', 500)
        self.declare_parameter('jpeg_quality', 80)
        # Hamming bits (out of 256) above which a match is rejected as a
        # lookalike rather than the same physical point.
        self.declare_parameter('match_max_distance', 64)
        self.orb = cv2.ORB_create(
            nfeatures=self.get_parameter('max_features').value)
        # Brute force is fine at <=500 features; NORM_HAMMING because ORB
        # descriptors are binary strings, not float vectors. crossCheck
        # keeps only mutual best matches — one-to-one by construction,
        # which prunes most false pairings before any geometry exists.
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.prev_descriptors = None

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
        self.pub_matched = self.create_publisher(
            Int32, 'keypoints/matched', BIG_FRAME_QOS)

    def on_frame(self, msg: CompressedImage):
        entry = self.get_clock().now()

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                             cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('undecodable frame, skipping')
            return

        # ORB detects on greyscale. detectAndCompute (not detect) because
        # matching needs the descriptors; compute may drop a few keypoints
        # whose descriptor patch falls off the image edge.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        keypoints = keypoints or ()

        # Match against the previous frame by descriptor, then reject
        # matches whose Hamming distance says "similar-looking corner",
        # not "same physical point". queryIdx indexes THIS frame's
        # keypoints; trainIdx (unused until pose estimation) the previous.
        matched_idx = set()
        if descriptors is not None and self.prev_descriptors is not None:
            max_distance = self.get_parameter('match_max_distance').value
            matches = self.matcher.match(descriptors, self.prev_descriptors)
            matched_idx = {m.queryIdx for m in matches
                           if m.distance <= max_distance}
        self.prev_descriptors = descriptors

        matched = [kp for i, kp in enumerate(keypoints) if i in matched_idx]
        fresh = [kp for i, kp in enumerate(keypoints)
                 if i not in matched_idx]

        # Yellow = new this frame, green = re-observed from the previous
        # frame (drawn second, so tracked points win any overlap).
        # DRAW_RICH_KEYPOINTS renders size and orientation, so the overlay
        # shows *what ORB thinks it found*, not just where.
        annotated = cv2.drawKeypoints(
            frame, fresh, None, color=(0, 255, 255),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        annotated = cv2.drawKeypoints(
            annotated, matched, None, color=(0, 255, 0),
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

        matched_count = Int32()
        matched_count.data = len(matched)
        self.pub_matched.publish(matched_count)

        # Cost measured against our own clock only — this camera's header
        # stamps lag ~0.73 s by fault (docs/info/camera.md#timestamps) and
        # prove nothing about the pipeline.
        done = self.get_clock().now()
        self.get_logger().info(
            f'{len(keypoints)} keypoints ({len(matched)} matched to prev), '
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
