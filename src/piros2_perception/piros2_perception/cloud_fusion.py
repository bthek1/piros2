"""
A persistent map the camera feed edits, instead of a cloud it replaces.

cloud_projector rebuilds its output from scratch every frame; this node
holds state — a fixed lattice of cells covering (by default) a 2 m cube in
front of the camera — and every synced depth frame nudges the cells it can
see towards what was observed. Three ideas, met by hand:

- The grid is a truncated signed distance field (TSDF). Each cell stores
  D, how far in front of (+) or behind (−) the observed surface it sits,
  clipped to a band ±truncation, and a weight w counting observations.
  A new frame updates D as a running average, D ← (w·D + d_obs)/(w + 1),
  with w capped at w_max — early observations move a cell a lot, later
  ones barely, and the cap is the saturation that makes the map treat
  settled geometry as static. The rendered surface is wherever D crosses
  zero, so as D is nudged the visible point slides along the camera ray:
  the seed cube deforms into the room.
- Pose comes from tf2, not from an assumption. Cells live in map_frame
  (base_link for a fixed camera); fusing a frame means asking the TF tree
  where camera_optical_frame was at that frame's stamp. This is the
  repo's first *dynamic* TF consumer — a Buffer fed by a
  TransformListener, queried per message — and it is what later lets a
  moving camera work by just repointing map_frame at odom.
- Publishing is decoupled from fusing. Fusion runs per synced pair
  (~3 fps, the depth node's pace); the map is extracted and published on
  its own ~1 Hz timer, because the map changes slowly and extraction has
  its own cost. Unobserved cells (w = 0) publish as a faint strided
  lattice so the initial state — the seed cube — is visible in RViz
  before a single frame has been fused.

Everything is published in map_frame; RViz needs no transform gymnastics.
"""

import cv2
import message_filters
import numpy as np
from piros2_perception.cloud_projector import (BIG_FRAME_QOS, POINT_DTYPE,
                                               POINT_FIELDS)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Unobserved lattice cells render in this faint grey (0x003C3C3C packed).
SEED_GREY = np.uint32(0x003C3C3C).view(np.float32)


def quat_to_matrix(x, y, z, w):
    """Quaternion → 3×3 rotation matrix, by hand (no scipy on purpose)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], np.float32)


class CloudFusion(Node):

    def __init__(self):
        super().__init__('cloud_fusion')

        # The frame the map lives in. base_link while the camera is
        # static; odom once an odometry node exists to publish it.
        self.declare_parameter('map_frame', 'base_link')
        # Grid placement in map_frame metres: min corner + extents.
        # Default: a 2 m cube starting at the camera and extending
        # forward (+x in base_link), centred sideways, half a metre
        # below the mount to 1.5 m above it.
        self.declare_parameter('grid_origin', [0.0, -1.0, -0.5])
        self.declare_parameter('grid_size', [2.0, 2.0, 2.0])
        self.declare_parameter('voxel_size', 0.02)
        # Half-width of the fused band around a surface. Cells further
        # behind a measured surface than this were never actually seen
        # and must not be touched.
        self.declare_parameter('truncation', 0.06)
        # Saturation: observations stop shifting a cell once w hits this.
        self.declare_parameter('w_max', 50.0)
        # A cell needs this many observations before it counts as surface.
        self.declare_parameter('w_min', 3.0)
        # P2 hysteresis: a saturated cell re-opens only after
        # reopen_frames consecutive disagreements larger than this.
        self.declare_parameter('change_threshold', 0.15)
        self.declare_parameter('reopen_frames', 5)
        self.declare_parameter('publish_rate', 1.0)
        # Every Nth unobserved cell per axis in the seed-lattice display.
        self.declare_parameter('seed_stride', 5)
        # Depth beyond this is the model saying "background, no idea".
        self.declare_parameter('far_clip', 20.0)

        origin = np.array(self.get_parameter('grid_origin').value,
                          np.float32)
        size = np.array(self.get_parameter('grid_size').value, np.float32)
        self.voxel = float(self.get_parameter('voxel_size').value)
        self.trunc = float(self.get_parameter('truncation').value)
        shape = np.maximum(np.round(size / self.voxel), 1).astype(int)

        # The map's whole state, ~32 MB at the default 100³ cells. D
        # starts at +truncation ("no surface seen yet"), weights at zero.
        self.d = np.full(shape, self.trunc, np.float32)
        self.w = np.zeros(shape, np.float32)
        self.colour = np.zeros((*shape, 3), np.float32)
        # Cell-centre coordinates in map_frame, computed once: the grid
        # never moves, only its values do.
        ix, iy, iz = np.meshgrid(*(np.arange(n) for n in shape),
                                 indexing='ij')
        self.centres = (origin
                        + (np.stack([ix, iy, iz], axis=-1) + 0.5)
                        * self.voxel).astype(np.float32)
        # Flat views over the same memory: the fusion maths wants (N,)
        # and (N, 3); the publisher wants the 3D shape for striding.
        self.d_flat = self.d.reshape(-1)
        self.w_flat = self.w.reshape(-1)
        self.colour_flat = self.colour.reshape(-1, 3)
        self.centres_flat = self.centres.reshape(-1, 3)

        self.get_logger().info(
            f'grid {shape[0]}x{shape[1]}x{shape[2]} cells '
            f'({self.d.size:,} @ {self.voxel} m) in '
            f"'{self.get_parameter('map_frame').value}'")

        self.k = None
        self.pairs_seen = 0
        self.create_subscription(
            CameraInfo, 'camera_info', self.on_info, 10)

        # Same stamp-matched pair as cloud_projector, for the same
        # reason: fusion needs the depth and the colour of one moment.
        depth_sub = message_filters.Subscriber(
            self, Image, 'depth', qos_profile=BIG_FRAME_QOS)
        colour_sub = message_filters.Subscriber(
            self, CompressedImage, 'image_raw/compressed',
            qos_profile=BIG_FRAME_QOS)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [depth_sub, colour_sub], queue_size=30, slop=0.05)
        self.sync.registerCallback(self.on_pair)

        # The tf2 pattern: a Buffer accumulates every transform heard on
        # /tf and /tf_static; lookups against it happen at message
        # stamps, in callbacks, whenever — the listener just feeds it.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_map = self.create_publisher(
            PointCloud2, 'map_points', BIG_FRAME_QOS)
        self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.publish_map)

    def on_info(self, msg: CameraInfo):
        self.k = np.array(msg.k).reshape(3, 3)

    def on_pair(self, depth_msg: Image, colour_msg: CompressedImage):
        """Fuse one depth+colour pair into the grid (the P1 update)."""
        if self.k is None or self.k[0, 0] == 0.0:
            self.get_logger().warn(
                'no usable camera_info yet — is the camera calibrated?',
                throttle_duration_sec=5.0)
            return
        # Where was the camera when this frame was taken? Asked of tf2 at
        # the frame's own stamp: static TF answers timelessly now; the
        # same lookup follows a moving camera once odometry exists.
        try:
            tf = self.tf_buffer.lookup_transform(
                depth_msg.header.frame_id,
                self.get_parameter('map_frame').value,
                Time.from_msg(depth_msg.header.stamp))
        except TransformException as err:
            self.get_logger().warn(
                f'no TF for pair: {err}', throttle_duration_sec=5.0)
            return
        entry = self.get_clock().now()

        depth = np.frombuffer(depth_msg.data, np.float32).reshape(
            depth_msg.height, depth_msg.width)
        colour = cv2.imdecode(np.frombuffer(colour_msg.data, np.uint8),
                              cv2.IMREAD_COLOR)
        if colour is None or colour.shape[:2] != depth.shape:
            self.get_logger().warn('depth/colour size mismatch, skipping')
            return

        q, t = tf.transform.rotation, tf.transform.translation
        rot = quat_to_matrix(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], np.float32)

        # Projective association, whole grid at once: every cell centre
        # into the camera frame, through K, onto a pixel. The depth image
        # value at that pixel is what the camera measured *along the ray
        # through this cell*.
        cam = self.centres_flat @ rot.T + trans
        z = cam[:, 2]
        sel = np.nonzero(z > 0.05)[0]  # behind-camera cells: not visible

        fx, fy = self.k[0, 0], self.k[1, 1]
        cx, cy = self.k[0, 2], self.k[1, 2]
        u = np.floor(fx * cam[sel, 0] / z[sel] + cx).astype(np.int64)
        v = np.floor(fy * cam[sel, 1] / z[sel] + cy).astype(np.int64)
        inb = ((u >= 0) & (u < depth_msg.width)
               & (v >= 0) & (v < depth_msg.height))
        sel, u, v = sel[inb], u[inb], v[inb]

        measured = depth[v, u]
        far = self.get_parameter('far_clip').value
        # sdf: how far this cell sits in front of the measured surface.
        # Cells more than `truncation` behind it are occluded — the
        # camera never saw them, so they must not be updated.
        sdf = measured - z[sel]
        seen = (measured > 0.0) & (measured < far) & (sdf > -self.trunc)
        sel, sdf = sel[seen], sdf[seen]
        d_obs = np.minimum(sdf, self.trunc)

        # The incremental push/pull: a running average that early
        # observations dominate and late ones barely move, with the
        # weight capped — the saturation the plan is named for.
        w_max = self.get_parameter('w_max').value
        w_old = self.w_flat[sel]
        self.d_flat[sel] = ((w_old * self.d_flat[sel] + d_obs)
                            / (w_old + 1.0))
        self.w_flat[sel] = np.minimum(w_old + 1.0, w_max)

        # Colour only means anything near the surface; average it with
        # the same weights (BGR from cv2 → stored RGB).
        near = np.abs(sdf) < self.trunc
        nsel = sel[near]
        rgb_new = colour[v[seen][near], u[seen][near], ::-1]
        self.colour_flat[nsel] = (
            (w_old[near, None] * self.colour_flat[nsel] + rgb_new)
            / (w_old[near, None] + 1.0))

        self.pairs_seen += 1
        done = self.get_clock().now()
        self.get_logger().info(
            f'pair {self.pairs_seen}: {sel.size:,} cells updated '
            f'({nsel.size:,} near-surface) in '
            f'{(done - entry).nanoseconds / 1e6:.1f} ms, '
            f'max w {self.w_flat.max():.0f}',
            throttle_duration_sec=5.0)

    def publish_map(self):
        w_min = self.get_parameter('w_min').value
        stride = self.get_parameter('seed_stride').value

        # Surface = observed cells whose D straddles zero (within one
        # cell of the crossing). |D| < voxel gives a shell ~2 cells
        # thick — readable without marching cubes.
        surface = (self.w >= w_min) & (np.abs(self.d) < self.voxel)

        # The seed lattice: a sparse sample of never-observed cells, so
        # the unfused map is visible instead of an empty display.
        seed = np.zeros_like(surface)
        seed[::stride, ::stride, ::stride] = True
        seed &= self.w == 0

        rgb = self.colour.astype(np.uint32)
        packed = ((rgb[..., 0] << 16) | (rgb[..., 1] << 8)
                  | rgb[..., 2]).view(np.float32)

        cloud = np.empty(int(surface.sum() + seed.sum()), POINT_DTYPE)
        n = int(surface.sum())
        for axis, name in enumerate('xyz'):
            cloud[name][:n] = self.centres[surface][:, axis]
            cloud[name][n:] = self.centres[seed][:, axis]
        cloud['rgb'][:n] = packed[surface]
        cloud['rgb'][n:] = SEED_GREY

        out = PointCloud2()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter('map_frame').value
        out.height = 1
        out.width = cloud.size
        out.fields = POINT_FIELDS
        out.is_bigendian = False
        out.point_step = POINT_DTYPE.itemsize
        out.row_step = POINT_DTYPE.itemsize * cloud.size
        out.is_dense = True
        out.data = cloud.tobytes()
        self.pub_map.publish(out)


def main():
    rclpy.init()
    node = CloudFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
