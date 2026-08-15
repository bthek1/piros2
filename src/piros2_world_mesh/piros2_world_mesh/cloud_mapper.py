"""
Subscribe /points, accumulate a voxel map in odom, publish /world/map_points.

World 3D plan P3, upgraded by the world fusion plan's P2. The concepts:

- The *consumer* side of TF: a Buffer + TransformListener assembles the
  tree other nodes broadcast, and one lookup_transform composes the whole
  chain — dynamic odom → base_link from the keypoint detector, static
  base_link → camera_optical_frame from the camera launch. The lookup asks
  for Time() = latest, never the cloud's stamp: this camera's stamps lag
  ~0.73 s by fault (docs/info/camera.md#timestamps), so a stamp-matched
  lookup would pose every cloud with where the camera pointed earlier.
  At hand-rotation speeds the smear is small; the plan says so honestly.
- A dict keyed by voxel index is a *bounded* map: revisited space updates
  in place instead of duplicating, so memory scales with scene volume,
  not runtime.
- Fusion (P2): each voxel keeps a **running weighted average** of the
  points that land in it — the TSDF lesson applied to the structure we
  already had. Averaging is what kills depth noise: latest-wins kept one
  noisy sample per voxel forever, so surfaces sat a full sample's error
  off true and shimmered on every revisit. The weight is capped so a
  scene change can still displace stale evidence, and voxels seen fewer
  than min_weight times (noise at surface edges, usually) are held back
  from the published map — observations, not one-off guesses.
- The map admits its limits out loud: points beyond max_range are dropped
  (monocular depth degrades with distance — far guesses would smear the
  panorama), and at max_voxels it stops growing and logs once. No silent
  eviction. ~/clear re-zeros it, the mirror of the detector's ~/reset.

Rotation-only odometry upstream means the result is a panorama from one
viewpoint, not a walkable map — see the plan's honest-scope section.
"""

import numpy as np
from piros2_perception.cloud_projector import POINT_DTYPE, POINT_FIELDS
from piros2_world_mesh.se3 import (
    make_transform,
    rotation_from_quaternion,
    transform_points,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

# Same reasoning as every big-message QoS in this repo.
BIG_FRAME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)


def unpack_rgb(packed):
    """0x00RRGGBB float32 bit patterns -> Nx3 float (r, g, b) in 0-255."""
    bits = np.ascontiguousarray(packed).view(np.uint32)
    return np.column_stack([(bits >> 16) & 255,
                            (bits >> 8) & 255,
                            bits & 255]).astype(np.float64)


def pack_rgb(rgb):
    """Nx3 (r, g, b) floats -> 0x00RRGGBB float32 bit patterns."""
    r, g, b = np.clip(np.round(rgb), 0, 255).astype(np.uint32).T
    return ((r << 16) | (g << 8) | b).view(np.float32)


class VoxelMap:
    """
    Voxel index -> running weighted average of the points seen there.

    Pure Python + numpy, no ROS — the fusion semantics live here so the
    tests can exercise them without a graph. Per add(), each touched
    voxel gains ONE observation (a cloud's points falling in it are
    averaged first): the weight counts independent looks, exactly the
    TSDF weight's semantics, so min_weight thresholds mean "confirmed by
    N clouds", not "hit by N pixels of one frame".

    Storage is array-backed with a dict from voxel key to row: dict
    membership stays O(1) while the per-voxel arithmetic runs vectorised
    over each cloud's unique voxels (numpy per-point loops would cost
    microseconds each, hundreds of ms per cloud). The mean update is the
    standard running form, mean += (new - mean) / (w + 1), with w capped
    at max_weight so accumulated evidence never becomes immovable — a
    moved chair should eventually win the argument.
    """

    def __init__(self, voxel_size, max_voxels, max_weight=64):
        self.voxel_size = voxel_size
        self.max_voxels = max_voxels
        self.max_weight = max_weight
        self.index = {}
        # x y z r g b per row; rows are assigned once and never move.
        self.mean = np.zeros((max_voxels, 6))
        self.weight = np.zeros(max_voxels)
        self.saturated = False

    def __len__(self):
        return len(self.index)

    def add(self, points):
        """
        Fold a POINT_DTYPE array (already in map frame) into the map.

        Returns True exactly once: the first add that hits the voxel cap.
        After that, known voxels keep updating; new ones are dropped.
        """
        xyz = np.column_stack([points['x'], points['y'], points['z']])
        values = np.column_stack([xyz, unpack_rgb(points['rgb'])])
        keys = np.floor(xyz / self.voxel_size).astype(np.int64)

        # Collapse this cloud to one mean value per touched voxel.
        unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
        sums = np.zeros((len(unique_keys), 6))
        np.add.at(sums, inverse, values)
        counts = np.bincount(inverse, minlength=len(unique_keys))
        cloud_mean = sums / counts[:, None]

        # The only per-voxel Python work: dict lookup / row assignment.
        rows = np.empty(len(unique_keys), dtype=np.int64)
        known = np.ones(len(unique_keys), dtype=bool)
        first_saturation = False
        for i, key in enumerate(map(tuple, unique_keys.tolist())):
            row = self.index.get(key)
            if row is None:
                if len(self.index) < self.max_voxels:
                    row = len(self.index)
                    self.index[key] = row
                else:
                    known[i] = False
                    if not self.saturated:
                        self.saturated = True
                        first_saturation = True
                    continue
            rows[i] = row

        rows = rows[known]
        w = np.minimum(self.weight[rows], self.max_weight)[:, None]
        self.mean[rows] = (self.mean[rows] * w + cloud_mean[known]) / (w + 1)
        self.weight[rows] += 1
        return first_saturation

    def as_array(self, min_weight=0):
        """Return POINT_DTYPE rows for voxels seen >= min_weight times."""
        used = len(self.index)
        rows = np.flatnonzero(self.weight[:used] >= min_weight)
        out = np.empty(len(rows), dtype=POINT_DTYPE)
        mean = self.mean[rows]
        out['x'], out['y'], out['z'] = mean[:, :3].T.astype(np.float32)
        out['rgb'] = pack_rgb(mean[:, 3:])
        return out

    def clear(self):
        self.index.clear()
        self.weight[:] = 0.0
        self.saturated = False


class CloudMapper(Node):

    def __init__(self):
        super().__init__('cloud_mapper')

        # 5 cm voxels resolve furniture without ballooning the dict.
        self.declare_parameter('voxel_size', 0.05)
        # The map republishes on its own clock, decoupled from the cloud
        # rate — same design as the dashboard's wall-timer.
        self.declare_parameter('map_publish_rate', 1.0)
        # Hard cap; hitting it logs once and growth stops.
        self.declare_parameter('max_voxels', 200000)
        # Beyond this range (metres, from the camera) monocular depth is
        # more guess than measurement — drop it before it smears the map.
        self.declare_parameter('max_range', 6.0)
        # Fusion (world fusion plan P2): publish only voxels confirmed by
        # at least this many clouds — a one-look voxel is usually depth
        # noise at a surface edge, not furniture.
        self.declare_parameter('min_weight', 2)
        # Running-average inertia cap: evidence older than this many
        # observations weighs no more than this, so a changed scene can
        # still displace it.
        self.declare_parameter('max_weight', 64)

        self.map = VoxelMap(
            self.get_parameter('voxel_size').value,
            self.get_parameter('max_voxels').value,
            self.get_parameter('max_weight').value)

        # The listener feeds every /tf and /tf_static message into the
        # buffer; lookups then interpolate/compose from what arrived.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PointCloud2, 'points', self.on_cloud, BIG_FRAME_QOS)
        self.pub_map = self.create_publisher(
            PointCloud2, 'world/map_points', BIG_FRAME_QOS)
        self.create_timer(
            1.0 / self.get_parameter('map_publish_rate').value,
            self.on_timer)
        self.create_service(Trigger, '~/clear', self.on_clear)

    def on_cloud(self, msg: PointCloud2):
        # The projector's structured array IS the wire format, so parsing
        # is one frombuffer with the shared dtype — no field walking.
        cloud = np.frombuffer(msg.data, dtype=POINT_DTYPE)
        if cloud.size == 0:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', msg.header.frame_id, Time())
        except TransformException as error:
            self.get_logger().warn(
                f'no transform odom ← {msg.header.frame_id} yet ({error})',
                throttle_duration_sec=5.0)
            return

        xyz = np.column_stack([cloud['x'], cloud['y'], cloud['z']])
        # Range is measured in the optical frame, where the camera is the
        # origin — exactly "distance from the lens".
        near = np.linalg.norm(xyz, axis=1) <= \
            self.get_parameter('max_range').value

        # The lookup is T_wc in se3.py's notation: odom ← optical, i.e.
        # it maps camera-frame points into the map frame.
        q = tf.transform.rotation
        t = tf.transform.translation
        t_wc = make_transform(
            rotation_from_quaternion(q.x, q.y, q.z, q.w),
            [t.x, t.y, t.z])
        world = transform_points(t_wc, xyz[near])

        points = np.empty(world.shape[0], dtype=POINT_DTYPE)
        points['x'], points['y'], points['z'] = world.T.astype(np.float32)
        points['rgb'] = cloud['rgb'][near]
        if self.map.add(points):
            self.get_logger().warn(
                f'voxel cap ({self.map.max_voxels}) reached — the map '
                f'stops growing; call ~/clear to start over')

    def on_timer(self):
        merged = self.map.as_array(self.get_parameter('min_weight').value)
        out = PointCloud2()
        # Our own stamp and the map's own frame: this cloud is an
        # accumulation made now, not any single camera frame.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.height = 1
        out.width = merged.size
        out.fields = POINT_FIELDS
        out.is_bigendian = False
        out.point_step = POINT_DTYPE.itemsize
        out.row_step = POINT_DTYPE.itemsize * merged.size
        out.is_dense = True
        out.data = merged.tobytes()
        self.pub_map.publish(out)
        self.get_logger().info(
            f'map holds {len(self.map)} voxels, '
            f'{merged.size} credible published',
            throttle_duration_sec=10.0)

    def on_clear(self, request, response):
        self.map.clear()
        response.success = True
        response.message = 'map cleared'
        return response


def main():
    rclpy.init()
    node = CloudMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
