"""
Subscribe /depth + /image_raw/compressed + /camera_info, publish /points.

This is where the K matrix earns its keep. Three concepts:

- The pinhole model, met once by hand: a pixel (u, v) with depth z becomes
  x = (u - cx) * z / fx,  y = (v - cy) * z / fy — fx/fy are the focal
  length in pixel units, cx/cy where the optical axis hits the sensor.
  Vectorised in numpy over the whole image; no loops, no library doing the
  thinking for us.
- PointCloud2 memory layout: the message is a flat byte buffer described by
  its `fields` — here 16 bytes per point (x, y, z float32 + RGB packed into
  a float32's bits, the convention RViz expects). A numpy structured array
  with the same dtype IS the wire format; `tobytes()` is the whole
  serialisation.
- Paired-topic time sync: depth arrives ~300 ms after the RGB frame it was
  computed from, so the two streams are matched by *header stamp* with
  message_filters — which only works because the depth node kept honest
  headers. The camera's ~0.73 s stamp fault cancels out here: both stamps
  carry the same fault, and sync only compares them to each other.

Everything is published in camera_optical_frame (z forward, x right,
y down); RViz walks TF from base_link to pose the cloud in the world.
"""

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import (CameraInfo, CompressedImage, Image, PointCloud2,
                             PointField)

# RELIABLE for the same measured reason as everywhere else in this repo:
# BEST_EFFORT delivers zero fragmented megabyte messages.
BIG_FRAME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

# One point = 16 bytes. The struct below IS the PointCloud2 wire layout;
# 'rgb' is 0x00RRGGBB packed into the bit pattern of a float32 (RViz's
# convention for coloured clouds).
POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'),
                        ('z', '<f4'), ('rgb', '<f4')])

POINT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]


class CloudProjector(Node):

    def __init__(self):
        super().__init__('cloud_projector')

        # Every 4th pixel in both axes ≈ 57k points at 720p — plenty for a
        # room view at 1/16th the bandwidth and build cost.
        self.declare_parameter('subsample', 4)
        # Depth beyond this is the model saying "background, no idea"; the
        # 1/x inversion makes those values explode, so cut them.
        self.declare_parameter('far_clip', 20.0)

        self.k = None
        self.create_subscription(
            CameraInfo, 'camera_info', self.on_info, 10)

        # Stamp-matched pair: ApproximateTime tolerates jitter, but the
        # depth node copies its input header verbatim, so matches are in
        # fact exact. The queue must hold RGB frames long enough for the
        # slower depth to catch up (~300 ms at up to 30 fps).
        depth_sub = message_filters.Subscriber(
            self, Image, 'depth', qos_profile=BIG_FRAME_QOS)
        colour_sub = message_filters.Subscriber(
            self, CompressedImage, 'image_raw/compressed',
            qos_profile=BIG_FRAME_QOS)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [depth_sub, colour_sub], queue_size=30, slop=0.05)
        self.sync.registerCallback(self.on_pair)

        self.pub_points = self.create_publisher(
            PointCloud2, 'points', BIG_FRAME_QOS)

    def on_info(self, msg: CameraInfo):
        self.k = np.array(msg.k).reshape(3, 3)

    def on_pair(self, depth_msg: Image, colour_msg: CompressedImage):
        if self.k is None or self.k[0, 0] == 0.0:
            self.get_logger().warn(
                'no usable camera_info yet — is the camera calibrated?',
                throttle_duration_sec=5.0)
            return
        entry = self.get_clock().now()

        depth = np.frombuffer(depth_msg.data, np.float32).reshape(
            depth_msg.height, depth_msg.width)
        colour = cv2.imdecode(np.frombuffer(colour_msg.data, np.uint8),
                              cv2.IMREAD_COLOR)
        if colour is None or colour.shape[:2] != depth.shape:
            self.get_logger().warn('depth/colour size mismatch, skipping')
            return

        step = self.get_parameter('subsample').value
        far = self.get_parameter('far_clip').value
        fx, fy = self.k[0, 0], self.k[1, 1]
        cx, cy = self.k[0, 2], self.k[1, 2]

        # The projection, on the subsampled grid. meshgrid gives every
        # (u, v) pair; the maths then runs on whole arrays at once.
        z = depth[::step, ::step]
        uu, vv = np.meshgrid(
            np.arange(0, depth_msg.width, step, dtype=np.float32),
            np.arange(0, depth_msg.height, step, dtype=np.float32))
        valid = (z > 0.0) & (z < far)

        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy

        rgb = colour[::step, ::step]
        packed = ((rgb[..., 2].astype(np.uint32) << 16)
                  | (rgb[..., 1].astype(np.uint32) << 8)
                  | rgb[..., 0].astype(np.uint32)).view(np.float32)

        cloud = np.empty(int(valid.sum()), dtype=POINT_DTYPE)
        cloud['x'] = x[valid]
        cloud['y'] = y[valid]
        cloud['z'] = z[valid]
        cloud['rgb'] = packed[valid]

        out = PointCloud2()
        # The depth frame's header verbatim: same stamp, and crucially the
        # optical frame id — RViz uses it to transform the cloud via TF.
        out.header = depth_msg.header
        out.height = 1
        out.width = cloud.size
        out.fields = POINT_FIELDS
        out.is_bigendian = False
        out.point_step = POINT_DTYPE.itemsize
        out.row_step = POINT_DTYPE.itemsize * cloud.size
        out.is_dense = True
        out.data = cloud.tobytes()
        self.pub_points.publish(out)

        done = self.get_clock().now()
        self.get_logger().info(
            f'{cloud.size} points in '
            f'{(done - entry).nanoseconds / 1e6:.1f} ms',
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = CloudProjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
