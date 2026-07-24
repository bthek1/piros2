"""Subscribe /image_raw, publish edges as an annotated /image_processed.

The first node that transforms data rather than moving it. Three concepts:

- cv_bridge converts between sensor_msgs/Image and numpy arrays — zero-copy
  where it can, explicit about encodings where it cannot.
- One node can hold subscriptions and publications at once; the executor
  fires the callback per frame, and everything this node does happens there.
- Sensor streams want BEST_EFFORT QoS: a dropped frame is better than a
  stale one arriving late. RELIABLE would make the camera re-send frames we
  no longer want.

Runs on the Pi, next to the camera: /image_raw at 720p RGB is ~83 MB/s and
must never cross the Wi-Fi. Only the JPEG streams leave this machine.
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image

# Our processing (~47 ms measured at 720p) exceeds the 33 ms frame interval,
# so the node cannot keep up with the camera and something must drop frames.
# Two ways to do that:
#   1. QoS depth 1 — let DDS keep only the latest. Tried; with CycloneDDS on
#      this setup a BEST_EFFORT/KEEP_LAST-1 subscription starved (one frame
#      at match time, then silence), so it is not used.
#   2. The stock sensor profile (BEST_EFFORT, depth 5) plus an explicit
#      freshness gate at callback entry — stale frames return immediately,
#      the queue drains at near-zero cost, and only fresh frames pay for
#      processing. This is what runs; measured latency in the node's log.
# Without either, the depth-5 queue sits permanently full and every frame
# waits in it: 380 ms end-to-end measured before the gate existed.
STALE_FRAME_MS = 60.0


class EdgeDetector(Node):

    def __init__(self):
        super().__init__('edge_detector')
        self.bridge = CvBridge()

        # BEST_EFFORT is compatible with a RELIABLE publisher (the pair
        # degrades to the weaker policy) — a BEST_EFFORT *publisher* with a
        # RELIABLE subscriber would not be, which is the asymmetry that
        # bites in RViz.
        self.sub = self.create_subscription(
            Image, 'image_raw', self.on_frame, qos_profile_sensor_data)
        self.dropped = 0

        self.pub_image = self.create_publisher(
            Image, 'image_processed', qos_profile_sensor_data)
        # image_transport is C++-only, so a Python publisher gets no automatic
        # compressed variant. Publish it by hand on the conventional name —
        # rqt_image_view and RViz find <topic>/compressed by suffix.
        self.pub_jpeg = self.create_publisher(
            CompressedImage, 'image_processed/compressed',
            qos_profile_sensor_data)

    def on_frame(self, msg: Image):
        # Two numbers tell different stories: queue age (stamp -> callback
        # entry) is backlog/transport, processing is our own cost. Logging
        # only their sum makes those indistinguishable.
        entry = self.get_clock().now()

        # The freshness gate: work slower than the frame interval means the
        # queue always holds stale frames — skip them cheaply here instead
        # of paying 47 ms to process history nobody wants.
        age_ms = (entry - Time.from_msg(msg.header.stamp)).nanoseconds / 1e6
        if age_ms > STALE_FRAME_MS:
            self.dropped += 1
            return

        # 'bgr8' is what OpenCV expects; cv_bridge converts from the wire
        # encoding (rgb8 from usb_cam) and flags impossible requests loudly.
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Cheap enough to run inline (~a few ms at 720p with NEON). Anything
        # slower than the frame interval must leave the executor thread, or
        # the subscription queue backs up and latency compounds.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 80, 160)

        # Annotate rather than replace: edges drawn green over the original,
        # so the output is self-explanatory in a viewer.
        annotated = frame.copy()
        annotated[edges > 0] = (0, 255, 0)

        # Keep the incoming stamp: downstream consumers (and our own latency
        # number) care when the *frame* happened, not when we finished.
        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = msg.header
        self.pub_image.publish(out)

        jpeg = CompressedImage()
        jpeg.header = msg.header
        jpeg.format = 'jpeg'
        jpeg.data = cv2.imencode(
            '.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        self.pub_jpeg.publish(jpeg)

        # End-to-end pipeline latency, split into its two components.
        # Throttled so the log stays readable at 30 Hz.
        done = self.get_clock().now()
        queue_ms = (entry - Time.from_msg(msg.header.stamp)).nanoseconds / 1e6
        proc_ms = (done - entry).nanoseconds / 1e6
        self.get_logger().info(
            f'latency {queue_ms + proc_ms:.1f} ms '
            f'(queue {queue_ms:.1f} + processing {proc_ms:.1f}), '
            f'{self.dropped} stale frames dropped',
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = EdgeDetector()
    try:
        # SIGINT surfaces as KeyboardInterrupt, SIGTERM (systemd, `timeout`,
        # launch teardown) as ExternalShutdownException — both are normal
        # ways for a node to die and neither deserves a traceback.
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
