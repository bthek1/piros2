"""
Subscribe /points, accumulate a voxel map in odom, publish /world/map_points.

World 3D plan P3 — the panorama accumulator. Three concepts:

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
  not runtime. Latest wins per voxel — no averaging, no forgetting.
- The map admits its limits out loud: points beyond max_range are dropped
  (monocular depth degrades with distance — far guesses would smear the
  panorama), and at max_voxels it stops growing and logs once. No silent
  eviction. ~/clear re-zeros it, the mirror of the detector's ~/reset.

Rotation-only odometry upstream means the result is a panorama from one
viewpoint, not a walkable map — see the plan's honest-scope section.
"""

import numpy as np
from piros2_perception.cloud_projector import POINT_DTYPE, POINT_FIELDS
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


def rotation_from_quaternion(x, y, z, w):
    """Quaternion → rotation matrix; the detector's conversion, inverted."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class VoxelMap:
    """
    A dict from voxel index to the latest point seen in that voxel.

    Pure Python + numpy, no ROS — the accumulation semantics live here so
    the tests can exercise them without a graph. The plain insertion loop
    IS the latest-wins rule (later points overwrite earlier); ~25 ms per
    50k-point cloud, acceptable at the cloud rates this repo sees.
    """

    def __init__(self, voxel_size, max_voxels):
        self.voxel_size = voxel_size
        self.max_voxels = max_voxels
        self.voxels = {}
        self.saturated = False

    def add(self, points):
        """
        Fold a POINT_DTYPE array (already in map frame) into the map.

        Returns True exactly once: the first add that hits the voxel cap.
        After that, known voxels keep updating; new ones are dropped.
        """
        keys = np.floor(np.column_stack(
            [points['x'], points['y'], points['z']])
            / self.voxel_size).astype(np.int64)
        first_saturation = False
        for key, point in zip(map(tuple, keys.tolist()), points):
            if key in self.voxels or len(self.voxels) < self.max_voxels:
                self.voxels[key] = point
            elif not self.saturated:
                self.saturated = True
                first_saturation = True
        return first_saturation

    def as_array(self):
        if not self.voxels:
            return np.empty(0, dtype=POINT_DTYPE)
        return np.array(list(self.voxels.values()), dtype=POINT_DTYPE)

    def clear(self):
        self.voxels.clear()
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

        self.map = VoxelMap(
            self.get_parameter('voxel_size').value,
            self.get_parameter('max_voxels').value)

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

        q = tf.transform.rotation
        t = tf.transform.translation
        world = (xyz[near]
                 @ rotation_from_quaternion(q.x, q.y, q.z, q.w).T
                 + np.array([t.x, t.y, t.z]))

        points = np.empty(world.shape[0], dtype=POINT_DTYPE)
        points['x'], points['y'], points['z'] = world.T.astype(np.float32)
        points['rgb'] = cloud['rgb'][near]
        if self.map.add(points):
            self.get_logger().warn(
                f'voxel cap ({self.map.max_voxels}) reached — the map '
                f'stops growing; call ~/clear to start over')

    def on_timer(self):
        merged = self.map.as_array()
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
            f'map holds {merged.size} voxels',
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
