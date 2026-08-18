"""
Integrate live depth into a TSDF and publish a re-meshed surface.

Live mesh plan P0 — the live pipeline's fusion stage. The offline
pipeline (tools/recon/) proved the machinery; this node runs the same
`VoxelBlockGrid` in-session. The concepts:

- A TSDF is a *fusion accumulator*: each depth frame updates a running
  weighted average of signed distance per voxel, which is what kills
  depth noise (world fusion plan P1/P2). The mesh is *extracted* from
  it on a timer — accumulation at frame rate, meshing at human rate,
  the same decoupling as the dashboard's wall-timer.
- The mesh ships as a TRIANGLE_LIST Marker — real triangles in the
  message — because RViz caches `mesh_resource` files by URI: rewriting
  one file shows stale geometry forever, and unique names leak RViz
  memory every refresh. Tens of MB per refresh is fine on loopback and
  must never cross the Wi-Fi; this node is dev-box-only by design.
- open3d is lazy-imported on the first synced pair (the
  depth_estimator's onnxruntime pattern): it lives in the perception
  venv, so this node runs via the venv interpreter (`python -m`) while
  the colcon test run, on the system interpreter, can still import the
  module and unit-test the pure marker functions.

Pose honesty: frames are posed by a latest-only TF lookup (the 0.73 s
stamp fault rule), and under rotation-only odometry a hand pan's real
translation smears the mesh. P2 aligns depth scale; P3 offers real
6-DoF poses; this phase pretends neither.

The surface can follow the graph (SLAM plan P3): a TSDF integrates a
pose into every voxel it touches, so a loop closure that moves the past
poses cannot move the past surface — unless the frames are kept. This
node keeps a thinned memory of the frames it integrated (aligned depth
as uint16 at reduced resolution, the JPEG bytes, and the *odom* pose
each was posed with) and, when the detector's optimised keyframe
trajectory (`/world/trajectory` + its odom twin) moves the per-frame
correction beyond a threshold, rebuilds the volume from that memory at
the corrected poses — measured cost is what makes this viable, ~10 ms a
frame on the GPU. Live frames integrate at `world_frame ← optical`
(map when a SLAM backend owns map → odom, so the surface lives in the
corrected frame from the start).

The surface can outlive the session (world mesh plan P3): `~/save`
extracts now and writes meshes/live_<stamp>.ply — full detail, the
Marker's triangle cap is an rclpy build-cost concern that does not
apply to a file. The PLY is hand-written ASCII, the projector's
hand-built-PointCloud2 lesson again, which also keeps the path free of
open3d and unit-testable on the system interpreter.
"""

import os
import threading
import time

import cv2
from geometry_msgs.msg import Point
import message_filters
from nav_msgs.msg import Path
import numpy as np
from piros2_world_mesh.depth_align import ScaleAligner
from piros2_world_mesh.mesh_fill import complete_mesh
from piros2_world_mesh.se3 import invert, make_transform, rotation_from_quaternion
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

# Same reasoning as every big-message QoS in this repo.
BIG_FRAME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

# Latched: one refresh serves a late-joining RViz.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)


def marker_from_mesh(vertices, triangles, colours, frame_id, stamp):
    """
    Build a TRIANGLE_LIST Marker from mesh arrays.

    vertices Nx3 float, triangles Mx3 int, colours Nx3 float in [0,1].
    TRIANGLE_LIST carries no index buffer, so every triangle contributes
    its three vertices verbatim — the flatten below is the format.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'live_mesh'
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    # Per-vertex colours only apply when the base colour has alpha.
    marker.color.r = marker.color.g = marker.color.b = 1.0
    marker.color.a = 1.0

    flat = vertices[triangles.reshape(-1)]
    flat_colours = np.clip(colours[triangles.reshape(-1)], 0.0, 1.0)
    # Bulk-building rclpy messages is the slow part (~1 us per object);
    # comprehensions + float() keep it tolerable at the refresh rate.
    marker.points = [Point(x=float(x), y=float(y), z=float(z))
                     for x, y, z in flat]
    marker.colors = [ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
                     for r, g, b in flat_colours]
    return marker


def ply_from_mesh(vertices, triangles, colours):
    """
    Serialise mesh arrays to ASCII PLY bytes.

    vertices Nx3 float, triangles Mx3 int (indices into vertices),
    colours Nx3 float in [0,1], clamped here. Hand-written for the same
    reason cloud_projector hand-builds PointCloud2 — the format is the
    lesson — and RViz's assimp, open3d and `just view-mesh` all read it.
    """
    rgb = np.clip(np.round(np.asarray(colours, dtype=float) * 255.0),
                  0, 255).astype(np.uint8)
    header = [
        'ply',
        'format ascii 1.0',
        f'element vertex {len(vertices)}',
        'property float x',
        'property float y',
        'property float z',
        'property uchar red',
        'property uchar green',
        'property uchar blue',
        f'element face {len(triangles)}',
        'property list uchar int vertex_indices',
        'end_header',
    ]
    vertex_lines = [f'{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}'
                    for (x, y, z), (r, g, b) in zip(vertices, rgb)]
    face_lines = [f'3 {a} {b} {c}' for a, b, c in triangles]
    return '\n'.join(header + vertex_lines + face_lines + ['']).encode()


class TsdfMesher(Node):

    def __init__(self, **node_kwargs):
        super().__init__('tsdf_mesher', **node_kwargs)

        # 2 cm voxels: the mono-noise resolution the fusion plan
        # measured; finer would mesh the depth model's error.
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('depth_max', 6.0)
        # Meshing runs on this period, decoupled from the frame rate.
        self.declare_parameter('refresh_period', 10.0)
        # Voxels seen fewer times than this don't mesh (TSDF weight).
        self.declare_parameter('weight_threshold', 3.0)
        # Mesh completion (mesh-completion plan P2, replacing the old
        # fill_hole_radius): drop noise-flake components below this
        # triangle count (the P0 census found ~370 of them under ~64
        # triangles on a live scan)…
        self.declare_parameter('min_component_triangles', 30)
        # …then close every interior boundary loop — each component's
        # largest loop is its frontier and stays open; this guard stops
        # a frontier-sized loop being bridged silently.
        self.declare_parameter('fill_max_hole_radius', 0.25)
        # Filled patches are *assumed* surface; tint them magenta to see
        # exactly what was invented.
        self.declare_parameter('fill_debug_tint', False)
        # Marker triangle budget — enforced by quadric decimation, never
        # by dropping triangles (the old even-subsampling cap peppered
        # the whole surface with pinholes; mesh-completion plan P1).
        self.declare_parameter('max_triangles', 60000)
        # Per-frame depth scale alignment (live mesh plan P2): ray-cast
        # the TSDF from the frame's pose, median-ratio the overlap, and
        # scale the frame before integrating — the ±4% wobble fix.
        self.declare_parameter('align', True)
        self.declare_parameter('align_min_overlap', 0.2)
        self.declare_parameter('align_max_correction', 0.15)
        # ~/save writes PLYs here (relative to the CWD, which the session
        # recipes pin to the repo root); git-ignored like the offline
        # pipeline's meshes.
        self.declare_parameter('save_dir', 'meshes')
        # Also write a Poisson-closed companion (live_<stamp>_closed.ply)
        # on save: genuinely watertight — it closes the frontier too,
        # smoothly and blobbily, which is fiction the live mesh refuses
        # but downstream tools that demand closed input want. Never
        # replaces the honest PLY (mesh-completion plan P4).
        self.declare_parameter('save_watertight', False)
        # SLAM plan P3: the frame the surface lives in (odom, or map when
        # a backend publishes map → odom), and the rebuild-from-memory
        # that lets the surface follow a graph optimisation: how many
        # frames to remember (thinned evenly when full), the depth
        # downsample they are kept at, how far the map must have moved
        # to be worth a rebuild, and the minimum spacing of rebuilds.
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('rebuild', False)
        self.declare_parameter('rebuild_keep', 400)
        self.declare_parameter('rebuild_downsample', 2)
        self.declare_parameter('rebuild_min_shift_m', 0.02)
        self.declare_parameter('rebuild_min_shift_deg', 1.0)
        self.declare_parameter('rebuild_min_interval_s', 8.0)
        # ~/save also dumps the frame memory (depth, JPEG, odom pose,
        # applied correction per frame) as live_<stamp>_frames.npz — the
        # P3 gate re-integrates the two halves of a loop bag separately
        # from it (tools/verify/mesh_split.py). Off unless asked.
        self.declare_parameter('save_frames', False)

        # open3d state, created lazily on the first synced pair.
        self.o3d = None
        self.volume = None
        self.device = None
        self.k_matrix = None
        self.integrated = 0
        # Rolling applied-scale history: the P2 check reads its spread.
        self.recent_scales = []
        self.aligner = None  # built with the volume (params read once)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo, 'camera_info', self.on_camera_info, BIG_FRAME_QOS)
        depth_sub = message_filters.Subscriber(
            self, Image, 'depth', qos_profile=BIG_FRAME_QOS)
        colour_sub = message_filters.Subscriber(
            self, CompressedImage, 'image_raw/compressed',
            qos_profile=BIG_FRAME_QOS)
        # The depth node copies RGB headers verbatim, so pairs match
        # exactly; the small slop only forgives executor jitter.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [depth_sub, colour_sub], queue_size=30, slop=0.05)
        self.sync.registerCallback(self.on_pair)

        self.pub_mesh = self.create_publisher(
            Marker, 'world/mesh_live', LATCHED_QOS)
        self.create_timer(
            self.get_parameter('refresh_period').value, self.on_refresh)
        # Frame memory + the graph's latest word (P3).
        self.frames = []            # dicts: stamp_ns, depth_u16, jpeg, ...
        self.traj_map = None        # (stamps, 4x4 list) optimised
        self.traj_odom = None       # (stamps, 4x4 list) odom at capture
        self.rebuilds = 0
        self.last_rebuild_at = None
        # The refresh's heavy half runs off the executor thread (P3
        # finding: complete + decimate cost 12-15 s at 720k triangles and
        # starved integration to a handful of frames per minute).
        self.refresh_thread = None
        if self.get_parameter('rebuild').value:
            self.create_subscription(
                Path, 'world/trajectory', self.on_trajectory, LATCHED_QOS)
            self.create_subscription(
                Path, 'world/trajectory_odom', self.on_trajectory_odom,
                LATCHED_QOS)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_service(Trigger, '~/save', self.on_save)

    def ensure_volume(self):
        """Create the (lazy) open3d state; returns False if unavailable."""
        if self.volume is not None:
            return True
        try:
            import open3d as o3d
        except ImportError:
            self.get_logger().error(
                'open3d not importable — this node must run under the '
                'perception venv interpreter (see world.launch.py)',
                throttle_duration_sec=30.0)
            return False
        self.o3d = o3d
        self.device = o3d.core.Device(
            'CUDA:0' if o3d.core.cuda.is_available() else 'CPU:0')
        self.volume = o3d.t.geometry.VoxelBlockGrid(
            attr_names=('tsdf', 'weight', 'color'),
            attr_dtypes=(o3d.core.float32,) * 3,
            attr_channels=((1), (1), (3)),
            voxel_size=self.get_parameter('voxel_size').value,
            block_resolution=8,
            block_count=60000,
            device=self.device)
        self.aligner = ScaleAligner(
            self.get_parameter('align_min_overlap').value,
            self.get_parameter('align_max_correction').value)
        self.get_logger().info(f'TSDF volume on {self.device}')
        return True

    def on_camera_info(self, msg: CameraInfo):
        if self.k_matrix is not None:
            return
        k = np.array(msg.k).reshape(3, 3)
        if k[0, 0] <= 0.0:
            return
        self.k_matrix = k

    def on_pair(self, depth_msg: Image, colour_msg: CompressedImage):
        if self.k_matrix is None:
            self.get_logger().warn('no intrinsics yet — not integrating',
                                   throttle_duration_sec=10.0)
            return
        if not self.ensure_volume():
            return
        world = self.get_parameter('world_frame').value
        t_odom_optical = self.lookup('odom', depth_msg.header.frame_id)
        if t_odom_optical is None:
            return
        # The correction in force right now (identity when the surface
        # lives in odom); kept per frame so a rebuild knows what each
        # frame was posed with.
        t_world_odom = np.eye(4)
        if world != 'odom':
            t_world_odom = self.lookup(world, 'odom')
            if t_world_odom is None:
                return

        entry = self.get_clock().now()
        depth = np.frombuffer(depth_msg.data, np.float32).reshape(
            depth_msg.height, depth_msg.width)
        colour = cv2.imdecode(np.frombuffer(colour_msg.data, np.uint8),
                              cv2.IMREAD_COLOR)
        if colour is None:
            return

        t_wc = t_world_odom @ t_odom_optical

        o3c = self.o3d.core
        # integrate() accepts (float, float) or (uint16, uint8) depth/
        # colour dtype pairs only; our depth is float32 metres, so the
        # colour goes float32 [0,1] to match.
        depth_img = self.o3d.t.geometry.Image(
            o3c.Tensor(np.ascontiguousarray(depth))).to(self.device)
        colour_img = self.o3d.t.geometry.Image(o3c.Tensor(
            np.ascontiguousarray(
                colour[:, :, ::-1].astype(np.float32) / 255.0))).to(
            self.device)
        intrinsic = o3c.Tensor(self.k_matrix, o3c.float64)
        extrinsic = o3c.Tensor(invert(t_wc), o3c.float64)
        depth_max = self.get_parameter('depth_max').value
        # depth is already metres (32FC1), so depth_scale is 1.
        blocks = self.volume.compute_unique_block_coordinates(
            depth_img, intrinsic, extrinsic, 1.0, depth_max)

        # P2: cancel this frame's scale wobble before it is folded in.
        # Ray-cast the TSDF from the frame's own pose for the expected
        # depth; the ScaleAligner corrects only the fast deviation from
        # its rolling baseline — conforming to the map directly is
        # unstable (the renderer reads ~a voxel far, and that bias
        # compounds; measured, see depth_align.py).
        scale = 1.0
        if self.get_parameter('align').value and self.integrated > 0:
            cast = self.volume.ray_cast(
                blocks, intrinsic, extrinsic,
                depth_msg.width, depth_msg.height,
                render_attributes=['depth'], depth_scale=1.0,
                depth_min=0.1, depth_max=depth_max,
                weight_threshold=3.0)
            expected = cast['depth'].cpu().numpy().reshape(
                depth_msg.height, depth_msg.width)
            scale, overlap = self.aligner.scale_for(expected, depth)
            if scale != 1.0:
                depth_img = self.o3d.t.geometry.Image(o3c.Tensor(
                    np.ascontiguousarray(depth * np.float32(scale)))).to(
                    self.device)
                blocks = self.volume.compute_unique_block_coordinates(
                    depth_img, intrinsic, extrinsic, 1.0, depth_max)
        self.recent_scales = (self.recent_scales + [scale])[-100:]

        self.volume.integrate(blocks, depth_img, colour_img, intrinsic,
                              intrinsic, extrinsic, 1.0, depth_max)
        self.integrated += 1
        if self.get_parameter('rebuild').value:
            self.remember_frame(depth_msg, depth * np.float32(scale),
                                colour_msg, t_odom_optical, t_world_odom)
        cost = (self.get_clock().now() - entry).nanoseconds / 1e6
        scales = np.array(self.recent_scales)
        self.get_logger().info(
            f'{self.integrated} frames integrated, {cost:.1f} ms/frame, '
            f'align scale {scales.mean():.3f} ± {scales.std():.3f} '
            f'(last {len(scales)})',
            throttle_duration_sec=10.0)

    def lookup(self, target, source):
        """Latest TF as a 4x4, or None (warned, throttled)."""
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException as error:
            self.get_logger().warn(
                f'no transform {target} ← {source} yet ({error})',
                throttle_duration_sec=5.0)
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        return make_transform(
            rotation_from_quaternion(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])

    # ------------------------------------------------------------------
    # SLAM plan P3: frame memory and rebuild-from-memory.

    def remember_frame(self, depth_msg, depth_aligned, colour_msg,
                       t_odom_optical, t_world_odom):
        """Keep what a rebuild needs; thin evenly when over budget."""
        f = max(1, int(self.get_parameter('rebuild_downsample').value))
        small = depth_aligned[::f, ::f]
        depth_u16 = np.clip(small * 1000.0, 0, 65535).astype(np.uint16)
        self.frames.append({
            'stamp_ns': depth_msg.header.stamp.sec * 1_000_000_000
            + depth_msg.header.stamp.nanosec,
            'depth_u16': depth_u16,
            'jpeg': bytes(colour_msg.data),
            't_odom_optical': t_odom_optical,
            'applied': t_world_odom,
            'factor': f,
        })
        keep = self.get_parameter('rebuild_keep').value
        if len(self.frames) > keep:
            # Drop every other frame of the older half: coverage over
            # the whole session, denser where it is recent.
            half = len(self.frames) // 2
            self.frames = self.frames[:half:2] + self.frames[half:]

    def on_trajectory(self, msg: Path):
        self.traj_map = self._path_arrays(msg)
        self.maybe_rebuild()

    def on_trajectory_odom(self, msg: Path):
        self.traj_odom = self._path_arrays(msg)
        self.maybe_rebuild()

    @staticmethod
    def _path_arrays(msg):
        stamps, poses = [], []
        for ps in msg.poses:
            stamps.append(ps.header.stamp.sec * 1_000_000_000
                          + ps.header.stamp.nanosec)
            q, t = ps.pose.orientation, ps.pose.position
            poses.append(make_transform(
                rotation_from_quaternion(q.x, q.y, q.z, q.w),
                [t.x, t.y, t.z]))
        return np.array(stamps, dtype=np.int64), poses

    def desired_corrections(self):
        """Per-frame world ← odom the current graph implies, or None."""
        if self.traj_map is None or self.traj_odom is None:
            return None
        stamps, opt = self.traj_map
        stamps_o, odo = self.traj_odom
        n = min(len(opt), len(odo))
        if n == 0 or not np.array_equal(stamps[:n], stamps_o[:n]):
            return None
        corrections = [opt[k] @ invert(odo[k]) for k in range(n)]
        node_stamps = stamps[:n]
        out = []
        for frame in self.frames:
            k = int(np.argmin(np.abs(node_stamps - frame['stamp_ns'])))
            out.append(corrections[k])
        return out

    def maybe_rebuild(self):
        if not self.frames or self.volume is None:
            return
        desired = self.desired_corrections()
        if desired is None:
            return
        shift_m, shift_deg = 0.0, 0.0
        for frame, want in zip(self.frames, desired):
            delta = invert(frame['applied']) @ want
            shift_m = max(shift_m, float(np.linalg.norm(delta[:3, 3])))
            cos = (np.trace(delta[:3, :3]) - 1.0) / 2.0
            shift_deg = max(shift_deg, float(np.degrees(
                np.arccos(np.clip(cos, -1.0, 1.0)))))
        if (shift_m < self.get_parameter('rebuild_min_shift_m').value
                and shift_deg < self.get_parameter(
                    'rebuild_min_shift_deg').value):
            return
        now = time.monotonic()
        if (self.last_rebuild_at is not None
                and now - self.last_rebuild_at
                < self.get_parameter('rebuild_min_interval_s').value):
            return
        self.rebuild(desired, shift_m, shift_deg)

    def rebuild(self, desired, shift_m, shift_deg):
        """Re-integrate every remembered frame at its corrected pose."""
        entry = time.monotonic()
        o3c = self.o3d.core
        depth_max = self.get_parameter('depth_max').value
        self.volume = None
        self.ensure_volume()
        integrated = 0
        for frame, want in zip(self.frames, desired):
            f = frame['factor']
            colour = cv2.imdecode(np.frombuffer(frame['jpeg'], np.uint8),
                                  cv2.IMREAD_COLOR)
            if colour is None:
                continue
            depth = frame['depth_u16'].astype(np.float32) / 1000.0
            h, w = depth.shape
            colour = cv2.resize(colour, (w, h), interpolation=cv2.INTER_AREA)
            k = self.k_matrix.copy()
            k[:2] /= f
            depth_img = self.o3d.t.geometry.Image(
                o3c.Tensor(np.ascontiguousarray(depth))).to(self.device)
            colour_img = self.o3d.t.geometry.Image(o3c.Tensor(
                np.ascontiguousarray(
                    colour[:, :, ::-1].astype(np.float32) / 255.0))).to(
                self.device)
            intrinsic = o3c.Tensor(k, o3c.float64)
            extrinsic = o3c.Tensor(
                invert(want @ frame['t_odom_optical']), o3c.float64)
            blocks = self.volume.compute_unique_block_coordinates(
                depth_img, intrinsic, extrinsic, 1.0, depth_max)
            self.volume.integrate(blocks, depth_img, colour_img, intrinsic,
                                  intrinsic, extrinsic, 1.0, depth_max)
            frame['applied'] = want
            integrated += 1
        self.integrated = integrated
        self.rebuilds += 1
        self.last_rebuild_at = time.monotonic()
        cost = (self.last_rebuild_at - entry) * 1000.0
        self.get_logger().info(
            f'rebuilt TSDF #{self.rebuilds}: {integrated} frames at '
            f'corrected poses in {cost:.0f} ms (map moved up to '
            f'{shift_m:.3f} m / {shift_deg:.1f}°)')
        self.on_refresh()

    def extract_mesh_arrays(self, complete=True):
        """
        Extract the current surface as (vertices, triangles, colours).

        Shared by the timed refresh and ~/save; returns None while the
        volume meshes to nothing. Extraction is followed (complete=True,
        the save path) by the completion pass (mesh_fill.py): noise-flake
        components are pruned, and every *interior* boundary loop is
        closed with a patch assumed from its surroundings — while each
        component's frontier (its largest loop) stays open, because
        unseen space is never invented. The old single-radius
        `fill_holes` bound could not tell a large hole from the frontier
        and left every hole wider than 6 cm open (P0 measured the
        survivors at p50 7.3 cm). The refresh path completes *after*
        decimating instead (complete_arrays, on the worker thread).
        """
        mesh = self.volume.extract_triangle_mesh(
            weight_threshold=self.get_parameter('weight_threshold').value)
        legacy = mesh.to_legacy()
        vertices = np.asarray(legacy.vertices)
        triangles = np.asarray(legacy.triangles)
        colours = np.asarray(legacy.vertex_colors)
        if len(triangles) == 0:
            return None
        if not complete:
            return vertices, triangles, colours
        return self.complete_arrays(vertices, triangles, colours)

    def complete_arrays(self, vertices, triangles, colours):
        """Run the mesh-completion pass on arrays; None if nothing is left."""
        tint = (1.0, 0.0, 1.0) \
            if self.get_parameter('fill_debug_tint').value else None
        vertices, triangles, colours, stats = complete_mesh(
            vertices, triangles, colours,
            self.get_parameter('min_component_triangles').value,
            self.get_parameter('fill_max_hole_radius').value, tint)
        if len(triangles) == 0:
            return None
        if stats['pruned'] or stats['filled']:
            self.get_logger().info(
                f"completion: pruned {stats['pruned']} debris components, "
                f"filled {stats['filled']} interior holes",
                throttle_duration_sec=30.0)
        return vertices, triangles, colours

    def decimate_to_budget(self, vertices, triangles, colours, budget):
        """
        Quadric-decimate to the Marker budget (mesh-completion plan P1).

        Simplification, not deletion: the old even-subsampling cap
        removed every Nth triangle across the whole surface, turning an
        intact mesh into a sieve (P0 measured 37k boundary edges at a
        50% cap). Decimation keeps the surface closed at lower detail —
        vertex colours survive (verified on 0.19), ~126 ms per 57k
        input triangles, only paid when over budget.
        """
        mesh = self.o3d.geometry.TriangleMesh()
        mesh.vertices = self.o3d.utility.Vector3dVector(vertices)
        mesh.triangles = self.o3d.utility.Vector3iVector(triangles)
        mesh.vertex_colors = self.o3d.utility.Vector3dVector(colours)
        decimated = mesh.simplify_quadric_decimation(
            target_number_of_triangles=budget)
        return (np.asarray(decimated.vertices),
                np.asarray(decimated.triangles),
                np.asarray(decimated.vertex_colors))

    def on_refresh(self):
        """
        Re-mesh on the timer: extract on this thread, finish on a worker.

        Only extraction touches the volume; decimation, completion and
        the Marker build work on plain arrays and run on a background
        thread so integration keeps its cadence (measured: complete +
        decimate at 720k triangles is 12-15 s, longer than the refresh
        period — done inline it starved the mesher to a few frames a
        minute and the surface never saw the second half of a bag). A
        refresh is skipped while the previous one is still finishing.
        """
        if self.volume is None or self.integrated == 0:
            return
        if self.refresh_thread is not None and self.refresh_thread.is_alive():
            self.get_logger().info('refresh skipped — previous still busy',
                                   throttle_duration_sec=30.0)
            return
        entry = time.monotonic()
        extracted = self.extract_mesh_arrays(complete=False)
        if extracted is None:
            return
        extract_ms = (time.monotonic() - entry) * 1000.0
        self.refresh_thread = threading.Thread(
            target=self.finish_refresh, args=(*extracted, extract_ms),
            daemon=True)
        self.refresh_thread.start()

    def finish_refresh(self, vertices, triangles, colours, extract_ms):
        entry = time.monotonic()
        budget = self.get_parameter('max_triangles').value
        before = len(triangles)
        if len(triangles) > budget:
            # Decimate first, then complete: completion on the budgeted
            # mesh is ~1 s where the full-detail one is ~6 s, and the
            # marker is a view — the saved PLY still completes at full
            # detail (on_save).
            vertices, triangles, colours = self.decimate_to_budget(
                vertices, triangles, colours, budget)
        completed = self.complete_arrays(vertices, triangles, colours)
        if completed is None:
            return
        vertices, triangles, colours = completed
        # Header stamp zero = "latest TF" to RViz — the surface may live
        # in the map frame, whose transform is published on a timer.
        marker = marker_from_mesh(
            vertices, triangles, colours,
            self.get_parameter('world_frame').value, Time().to_msg())
        # Shutdown can land mid-refresh and invalidate the publisher — a
        # teardown race, not a fault worth a traceback.
        try:
            self.pub_mesh.publish(marker)
        except rclpy._rclpy_pybind11.RCLError:
            return
        cost = (time.monotonic() - entry) * 1000.0
        self.get_logger().info(
            f'refresh: {before} → {len(triangles)} triangles, extract '
            f'{extract_ms:.0f} ms + finish {cost:.0f} ms off-thread')

    def on_reset(self, request, response):
        self.volume = None
        self.integrated = 0
        self.recent_scales = []
        self.aligner = None
        self.frames = []
        response.success = True
        response.message = 'TSDF volume cleared'
        return response

    def on_save(self, request, response):
        """~/save: the surface outlives the session as a PLY."""
        if self.volume is None or self.integrated == 0:
            response.success = False
            response.message = 'nothing integrated yet — nothing to save'
            return response
        extracted = self.extract_mesh_arrays()
        if extracted is None:
            response.success = False
            response.message = 'volume meshes to zero triangles'
            return response
        vertices, triangles, colours = extracted
        save_dir = self.get_parameter('save_dir').value
        os.makedirs(save_dir, exist_ok=True)
        path = os.path.join(
            save_dir, time.strftime('live_%Y%m%d-%H%M%S.ply'))
        # Full detail on purpose: max_triangles caps the Marker (an
        # rclpy message-build cost), not the artifact.
        with open(path, 'wb') as f:
            f.write(ply_from_mesh(vertices, triangles, colours))
        response.success = True
        response.message = (f'{path}: {len(vertices)} vertices, '
                            f'{len(triangles)} triangles')
        if self.get_parameter('save_frames').value and self.frames:
            frames_path = path[:-4] + '_frames.npz'
            self.save_frames(frames_path)
            response.message += (f' + {frames_path}: {len(self.frames)} '
                                 'remembered frames')
        if self.get_parameter('save_watertight').value:
            closed_path = path[:-4] + '_closed.ply'
            n_tris = self.save_watertight(closed_path)
            response.message += (f' + {closed_path}: {n_tris} triangles '
                                 '(Poisson-closed, frontier included — '
                                 'assumed geometry)')
        self.get_logger().info(f'saved {response.message}')
        return response

    def save_frames(self, path):
        """Dump the frame memory as plain arrays (JPEG bytes packed)."""
        jpegs = [f['jpeg'] for f in self.frames]
        offsets = np.cumsum([0] + [len(j) for j in jpegs])
        np.savez_compressed(
            path,
            stamp_ns=np.array([f['stamp_ns'] for f in self.frames],
                              dtype=np.int64),
            depth_u16=np.stack([f['depth_u16'] for f in self.frames]),
            jpeg_bytes=np.frombuffer(b''.join(jpegs), dtype=np.uint8),
            jpeg_offsets=offsets,
            t_odom_optical=np.stack([f['t_odom_optical'] for f in self.frames]),
            applied=np.stack([f['applied'] for f in self.frames]),
            factor=np.array([f['factor'] for f in self.frames]),
            k_matrix=self.k_matrix)

    def save_watertight(self, path):
        """
        Write a Poisson-closed companion mesh; returns its triangle count.

        Screened Poisson over the TSDF's own point cloud (positions,
        TSDF-gradient normals, colours): the solve produces a closed
        surface by construction, extrapolating smoothly wherever the
        scan is open — including the frontier, which is exactly the
        fiction the live mesh refuses to publish. That is the tier
        split: the honest PLY for looking at what was seen, the closed
        one for tools that require watertight input. Written with
        open3d's writer (this path is open3d-bound anyway; the honest
        PLY keeps the hand-written unit-testable serialiser).
        """
        pcd = self.volume.extract_point_cloud(
            weight_threshold=self.get_parameter(
                'weight_threshold').value).to_legacy()
        if not pcd.has_normals():
            pcd.estimate_normals()
        # Octree depth 9 ≈ 1-2 cm cells at room scale — matched to the
        # 1.5 cm TSDF voxels; deeper reconstructs depth-model noise.
        mesh, _ = self.o3d.geometry.TriangleMesh.\
            create_from_point_cloud_poisson(pcd, depth=9)
        # Poisson clips its surface at the reconstruction domain's box,
        # leaving boundary loops there (measured: 223 edges on a room
        # scan). This tier's whole promise is closed, so an unbounded
        # fill pass finishes the job — no frontier honesty to protect
        # in a file already labelled assumed geometry.
        mesh = self.o3d.t.geometry.TriangleMesh.from_legacy(mesh)\
            .fill_holes(hole_size=1e6).to_legacy()
        self.o3d.io.write_triangle_mesh(path, mesh)
        return len(mesh.triangles)


def main():
    rclpy.init()
    node = TsdfMesher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
