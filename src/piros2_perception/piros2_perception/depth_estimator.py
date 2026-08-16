"""
Subscribe /image_raw/compressed, publish neural depth as /depth (+ preview).

The ROS-to-ML boundary node. Three concepts:

- A node can decode its own transport: JPEG bytes -> numpy is two lines of
  OpenCV, so subscribing the compressed topic directly keeps the Wi-Fi
  budget at ~2 MB/s with no republisher in the chain.
- The ML runtime lives outside ROS: onnxruntime is PyPI-only, installed in
  ~/.venvs/piros2-perception (--system-site-packages, so rclpy still
  resolves). The import is lazy and the session injectable, which is also
  what lets the unit tests run without 100 MB of weights.
- Derived sensor data keeps honest headers: /depth carries the *input
  frame's* stamp and camera_optical_frame id, because the depth describes
  that frame, not the moment inference finished.

Runs on the dev box — the Pi stays a sensor head; nothing here crosses the
network except the compressed stream already in flight.
"""

import os
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

# RELIABLE for megabyte-class messages, same reasoning as piros2_vision's
# edge detector: BEST_EFFORT delivers zero large frames once they fragment
# past the socket buffer (measured, docs/info/troubleshooting.md). The compressed
# frames here are ~100-200 kB and the 32FC1 depth is 3.7 MB — both sides of
# this node stay RELIABLE/KEEP_LAST-1: always the freshest frame, no backlog.
BIG_FRAME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

# Depth Anything V2 Small (ViT-S/14): RGB in, ImageNet-normalised, spatial
# dims a multiple of 14. 518 = 37*14 is the size the model was trained at.
MODEL_SIZE = 518
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

# Anchored on __file__, not the CWD — the same convention as the linter
# tests. Under --symlink-install resolve() follows the install-space symlink
# back to the source tree, where `just fetch-model` puts the weights.
DEFAULT_MODEL_PATH = str(
    Path(__file__).resolve().parents[1] / 'models'
    / 'depth_anything_v2_small.onnx')


class DepthEstimator(Node):

    def __init__(self, session=None):
        super().__init__('depth_estimator')
        self.bridge = CvBridge()

        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        # Monocular depth is *relative*: the model says "twice as far", never
        # "three metres". This scale turns its inverse-depth output into
        # metres-ish values (z = depth_scale / output) and is arbitrary until
        # P2's tape-measure check pins it. Tune, don't trust.
        self.declare_parameter('depth_scale', 10.0)
        # Republish the decoded input frame as raw /depth/rgb, stamped
        # identically to /depth, for exact-sync consumers (rgbd_odometry).
        # A separate 60 fps republisher node cannot pair with us: it and
        # this node each drop *different* frames (republish decodes ~14 Hz
        # of the camera's 42-60, inference keeps ~10), so their stamp sets
        # rarely intersect and exact sync limps at ~2 Hz with seconds of
        # sawtooth delay — measured 2026-08-15, no queue depth fixes it.
        # Sourcing raw from the frame we actually processed makes every
        # /depth stamp pairable by construction. The name is deliberately
        # NOT image_raw: usb_cam already publishes raw /image_raw, and a
        # dev-box subscriber on a shared name matches the Pi's publisher
        # too — 2.7 MB frames over Wi-Fi, the exact thing the compressed
        # transport exists to prevent (found live 2026-08-16: rgbd's
        # /image_raw remap was silently streaming raw across the link and
        # saturating it). Off by default: only rgbd sessions want it.
        self.declare_parameter('publish_rgb', False)
        # Pace the whole depth pipeline: frames arriving faster than this
        # are skipped before decoding. This node is the tempo for /depth,
        # the clouds, the mesher AND rgbd_odometry's TF — unpaced (~10 Hz
        # in daylight) it outruns rgbd's ~4-5 Hz, whose TF stamps then
        # trail the clouds by ~0.8 s while it chews queue backlog, and
        # RViz flaps "could not transform" per cloud (measured
        # 2026-08-16). 0 = unlimited.
        self.declare_parameter('max_rate', 0.0)
        self.last_processed = None

        self.session = session if session is not None else self._load_model()
        self.input_name = self.session.get_inputs()[0].name

        self.sub = self.create_subscription(
            CompressedImage, 'image_raw/compressed',
            self.on_frame, BIG_FRAME_QOS)
        self.pub_depth = self.create_publisher(Image, 'depth', BIG_FRAME_QOS)
        self.pub_preview = self.create_publisher(
            CompressedImage, 'depth/preview/compressed', BIG_FRAME_QOS)
        self.pub_rgb = None
        if self.get_parameter('publish_rgb').value:
            self.pub_rgb = self.create_publisher(
                Image, 'depth/rgb', BIG_FRAME_QOS)

    def _load_model(self):
        # Imported here, not at module top: onnxruntime comes from the pip
        # venv (~/.venvs/piros2-perception — see the package README), and
        # importing it lazily lets the unit tests load this module under the
        # system interpreter and inject a fake session instead.
        import onnxruntime

        path = os.path.expanduser(self.get_parameter('model_path').value)
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f'model not found at {path} — run `just fetch-model`')
        # The CUDA/cuDNN libraries come from pip (the onnxruntime-gpu
        # [cuda,cudnn] extras — see the README) and sit inside site-packages
        # where the system loader never looks; preload_dlls() dlopens them
        # first so the CUDA provider can link against them.
        onnxruntime.preload_dlls()
        # CUDA first, CPU as the fallback: onnxruntime falls back *silently*
        # when the GPU libs are missing or broken, so log which provider
        # actually won rather than assuming.
        session = onnxruntime.InferenceSession(
            path,
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
        self.get_logger().info(
            f'inference provider: {session.get_providers()[0]}')
        return session

    def on_frame(self, msg: CompressedImage):
        entry = self.get_clock().now()

        max_rate = self.get_parameter('max_rate').value
        if (max_rate > 0.0 and self.last_processed is not None
                and (entry - self.last_processed).nanoseconds
                < 1e9 / max_rate):
            return
        self.last_processed = entry

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                             cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('undecodable frame, skipping')
            return
        height, width = frame.shape[:2]

        # Inference dominates the frame budget (~300 ms on this CPU, so a
        # few fps). That is fine on purpose: KEEP_LAST-1 drops the frames we
        # cannot afford, and mapping does not need 30 fps.
        relative = self._infer(frame)

        # Model output is relative *inverse* depth — bigger means closer.
        # Invert to distance and clip the far tail, where 1/x explodes on
        # values the model means as "background, no idea".
        scale = self.get_parameter('depth_scale').value
        depth = np.clip(scale / np.maximum(relative, 1e-3), 0.0, 100.0)
        depth = cv2.resize(depth, (width, height),
                           interpolation=cv2.INTER_LINEAR)

        # The raw twin goes out first so exact sync can complete the pair
        # the moment /depth lands. bgr8 as decoded — consumers that care
        # (rtabmap does not) can convert.
        if self.pub_rgb is not None:
            rgb_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            rgb_msg.header = msg.header
            self.pub_rgb.publish(rgb_msg)

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header = msg.header
        self.pub_depth.publish(depth_msg)

        # Preview colourises the *relative* map normalised per frame: it is
        # for eyeballing "near bright, far dark", not for measuring.
        lo, hi = float(relative.min()), float(relative.max())
        norm = (relative - lo) / max(hi - lo, 1e-6)
        colour = cv2.applyColorMap((norm * 255).astype(np.uint8),
                                   cv2.COLORMAP_INFERNO)
        colour = cv2.resize(colour, (width, height),
                            interpolation=cv2.INTER_LINEAR)
        preview = CompressedImage()
        preview.header = msg.header
        preview.format = 'jpeg'
        preview.data = cv2.imencode(
            '.jpg', colour, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        self.pub_preview.publish(preview)

        # Measured against our own clock only — this camera's header stamps
        # lag ~0.73 s by fault and prove nothing (docs/info/camera.md#timestamps).
        done = self.get_clock().now()
        self.get_logger().info(
            f'inference+publish {(done - entry).nanoseconds / 1e6:.0f} '
            'ms/frame',
            throttle_duration_sec=5.0)

    def _infer(self, frame_bgr):
        rgb = cv2.cvtColor(
            cv2.resize(frame_bgr, (MODEL_SIZE, MODEL_SIZE),
                       interpolation=cv2.INTER_AREA),
            cv2.COLOR_BGR2RGB)
        x = (rgb.astype(np.float32) / 255.0 - IMAGENET_MEAN) / IMAGENET_STD
        x = x.transpose(2, 0, 1)[np.newaxis]
        out = self.session.run(None, {self.input_name: x})[0]
        return np.squeeze(out).astype(np.float32)


def main():
    rclpy.init()
    node = DepthEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
