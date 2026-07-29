"""
Subscribe /image_raw, publish edges as an annotated /image_processed.

The first node that transforms data rather than moving it. Three concepts:

- cv_bridge converts between sensor_msgs/Image and numpy arrays — zero-copy
  where it can, explicit about encodings where it cannot.
- One node can hold subscriptions and publications at once; the executor
  fires the callback per frame, and everything this node does happens there.
- QoS theory says sensor streams want BEST_EFFORT (drop, don't retransmit).
  Measured reality here says otherwise — see the note above the QoS profile.

Runs on the Pi, next to the camera: /image_raw at 720p RGB is ~83 MB/s and
must never cross the Wi-Fi. Only the JPEG streams leave this machine.
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image

# RELIABLE, deliberately against the "sensor data = BEST_EFFORT" doctrine.
# A 720p rgb8 frame is a 2.7 MB message -> ~1800 UDP fragments even on
# loopback, and with default socket buffers at least one fragment of every
# frame drops. BEST_EFFORT never retransmits, so no frame EVER completes:
# measured zero delivery from a best-effort subscription while a reliable one
# received instantly (`ros2 topic echo --qos-reliability ...` proves it
# either way). The doctrine assumes messages small enough to survive intact;
# megabyte frames are not that. Depth 1 = always the freshest frame.
BIG_FRAME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

# Do NOT measure latency (or gate freshness) against msg.header.stamp on this
# camera: `ros2 topic delay /image_raw` shows the stamps lag wall clock by a
# steady ~0.73 s even on a freshly started camera, while the frames themselves
# are visibly live — a UVC/driver timestamping fault, not a queue
# (docs/info/camera.md#timestamps). A first version of this node gated on stamp
# age and silently dropped 100% of frames. Only spans between our *own* clock
# reads are trustworthy here.


class EdgeDetector(Node):

    def __init__(self):
        super().__init__('edge_detector')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, 'image_raw', self.on_frame, BIG_FRAME_QOS)

        # Publishers are RELIABLE too: a RELIABLE *subscriber* (RViz and
        # rqt_image_view default to it) never matches a BEST_EFFORT
        # publisher — that asymmetry would make our topics silently
        # invisible in the standard viewers.
        self.pub_image = self.create_publisher(
            Image, 'image_processed', BIG_FRAME_QOS)
        # image_transport is C++-only, so a Python publisher gets no automatic
        # compressed variant. Publish it by hand on the conventional name —
        # rqt_image_view and RViz find <topic>/compressed by suffix.
        self.pub_jpeg = self.create_publisher(
            CompressedImage, 'image_processed/compressed', BIG_FRAME_QOS)

    def on_frame(self, msg: Image):
        entry = self.get_clock().now()

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

        # Per-frame processing cost, measured with one clock (ours). The
        # stamp-based "age" is logged too but labelled for what it is —
        # dominated by the camera's ~0.73 s timestamping fault, not by any
        # queue in this process. Throttled so the log stays readable.
        done = self.get_clock().now()
        proc_ms = (done - entry).nanoseconds / 1e6
        stamp_lag_ms = (entry - Time.from_msg(msg.header.stamp)
                        ).nanoseconds / 1e6
        self.get_logger().info(
            f'processing {proc_ms:.1f} ms/frame '
            f'(stamp lag {stamp_lag_ms:.0f} ms — camera clock fault, '
            f'not pipeline latency)',
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
