"""
Subscribe /image_raw/compressed, publish ORB keypoints, counts + rotation.

The feature-detection primer node, extended by the world 3D plan's P0 into
a rotation-only visual odometer. The concepts, in the order they arrived:

- A classical detector (ORB) is cheap enough for the full 30 fps stream on
  CPU — the contrast with the neural depth node's ~13 fps is deliberate,
  and the dashboard exists to make that gap visible.
- Descriptors turn detection into *tracking*: each keypoint gets a 256-bit
  binary fingerprint of its neighbourhood, and Hamming-matching against
  recent fingerprints separates re-observed points (drawn green) from new
  ones (drawn yellow). Those matched pairs are the raw material of visual
  odometry — camera pose falls out of how they move.
- Display matching runs against the last `match_window` frames, not just
  the previous one. Frame-to-frame matching loses ~25% of keypoints to
  *detection churn*: rank #95 vs #105 at the feature cap is decided by
  sensor noise, so features flicker out for a frame and back. A window
  forgives the flicker — the toy version of what SLAM systems call
  matching against a local map.
- Orientation (P0): strict *consecutive-pair* matches — separate from the
  display window — are unprojected through K to unit bearing rays, and one
  SVD (Kabsch) finds the rotation mapping last frame's rays onto this
  frame's. Pure rotation is the honest scope: the essential matrix is
  degenerate under zero baseline, and rays through K need no depth at all.
  Per-frame rotations compose into a running orientation on
  /camera/orientation — a compass built from pixels; it drifts, so a
  ~/reset service re-zeros it (docs/plans/completed/world-3d-plan.md).
- One frame can fan out into different *kinds* of topics: an image for
  humans, scalars for stats, a pose for TF-to-be. The counts ride plain
  std_msgs/Int32 — a custom message would need its own rosidl ament_cmake
  package, which a few numbers do not justify.

Duplicate frames (usb_cam's 60 Hz grab timer republishes each camera frame
~twice) are skipped whole, detected by CRC of the JPEG bytes: an identical
pair carries zero motion and would only dilute the estimator with identity
votes — and skipping them lands part of the standing "reduce compute" todo.

Relocalization (the relocalization plan): consecutive-pair tracking is
memoryless, so a fast flick used to corrupt the pose permanently — the
rotation spanning the blur is simply lost (and in rgbd mode the odometry
resets to identity). This node now remembers the room: healthy frames
feed a novelty-gated KeyframeStore (descriptors + bearing rays in odom,
plus 3D landmarks when depth is available), and after `relocalize_after`
frames without pairs it switches to recognition — match the store, then
solve the *absolute* pose (Kabsch on rays for orientation; Umeyama on 3D
landmark pairs for full 6-DoF). In kp mode the snap replaces the
composed orientation; in rgbd mode it is delivered to the pose's owner
via RTAB-Map's /reset_odom_to_pose. A wrong snap is worse than none, so
recovery demands a match margin and a robust-fit gate before acting.

SLAM (the SLAM plan, P1-P2): in rgbd mode the keyframes are also the
nodes of a pose graph. Each stored keyframe becomes a node with an
odometry edge to the previous one; every `loop_query_every` frames the
current view is matched against the *older* keyframes (the recent ones
excluded — matching your own last view is not a revisit) and a
survivor of the rigid-fit inlier test becomes a *loop edge*. That is
loop-closure detection while tracking is healthy, which the
loss-triggered relocalization above never did. Each new loop edge runs
the Gauss-Newton in pose_graph.py over the whole graph, and the
correction — optimised newest node ∘ its odometry pose⁻¹ — is published
as `map → odom` (REP-105: odom stays continuous, map absorbs the
correction) with the optimised keyframe poses on /world/trajectory.
That reach-back-and-fix-the-past step is what makes it SLAM rather than
odometry plus a place memory.

Runs on the dev box against the compressed stream already in flight; only
JPEG ever crosses the Wi-Fi.
"""

from collections import deque
import os
import time
import zlib

import cv2
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
import numpy as np
from piros2_world_mesh.keyframe_store import KeyframeStore
from piros2_world_mesh.pose_graph import information_matrix, PoseGraph
from piros2_world_mesh.se3 import (BASE_FROM_OPTICAL, euler_from_rotation,
                                   invert, make_transform,
                                   quaternion_from_rotation,
                                   rigid_transform_3d,
                                   rotation_from_quaternion,
                                   transform_points)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import ColorRGBA, Int32
from std_srvs.srv import Trigger
from tf2_ros import (Buffer, TransformBroadcaster, TransformException,
                     TransformListener)
from visualization_msgs.msg import Marker

# RELIABLE for megabyte-class messages, same reasoning as piros2_vision's
# edge detector: BEST_EFFORT delivers zero frames once a message fragments
# past the socket buffer (measured — docs/info/troubleshooting.md), and a
# BEST_EFFORT publisher is invisible to the RELIABLE-by-default viewers.
# Depth 1 = always the freshest frame, never a backlog.
BIG_FRAME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

# Latched: one keyframe-map refresh serves a late-joining RViz.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)


def keyframe_marker(keyframes, stamp, frame_id='odom', axis_length=0.15):
    """
    Build one LINE_LIST Marker drawing every stored keyframe's viewpoint.

    Relocalization plan P4: the debugging view that makes "why didn't it
    relocalize" answerable — a stored keyframe is a short cyan stroke
    from its capture position along its view direction. rgbd keyframes
    carry a full pose (position + direction); kp-mode keyframes carry
    only a direction, drawn from the origin. Pure function so the
    geometry is unit-testable without ROS.
    """
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = 'keyframes'
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD if keyframes else Marker.DELETE
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.01
    marker.color.r, marker.color.g, marker.color.b = 0.1, 0.9, 0.9
    marker.color.a = 0.9
    for kf in keyframes:
        origin = (kf.pose[:3, 3] if kf.pose is not None
                  else np.zeros(3))
        # A stored view_dir is optical z (forward); as a stroke in
        # odom that is the base-axes forward direction of the camera.
        direction = (kf.pose[:3, :3] @ np.array([0.0, 0.0, 1.0])
                     if kf.pose is not None
                     else BASE_FROM_OPTICAL @ kf.view_dir)
        tip = origin + axis_length * direction
        marker.points.append(Point(x=float(origin[0]), y=float(origin[1]),
                                   z=float(origin[2])))
        marker.points.append(Point(x=float(tip[0]), y=float(tip[1]),
                                   z=float(tip[2])))
    return marker


def path_msg(poses, stamps, frame_id, header_stamp):
    """nav_msgs/Path from 4x4 poses + per-pose stamps (float s or None)."""
    path = Path()
    path.header.stamp = header_stamp
    path.header.frame_id = frame_id
    for pose, stamp in zip(poses, stamps):
        ps = PoseStamped()
        if stamp is not None:
            ps.header.stamp.sec = int(stamp)
            ps.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(pose[0, 3])
        ps.pose.position.y = float(pose[1, 3])
        ps.pose.position.z = float(pose[2, 3])
        qx, qy, qz, qw = quaternion_from_rotation(pose[:3, :3])
        ps.pose.orientation.x = float(qx)
        ps.pose.orientation.y = float(qy)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
        path.poses.append(ps)
    return path


def graph_marker(poses, edges, stamp, frame_id='map'):
    """
    LINE_LIST Marker of a pose graph: odom edges grey, loop edges magenta.

    SLAM plan P1: the picture that shows *which* keyframes the detector
    believes are the same place. Pure function, unit-testable.
    """
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = 'keyframe_graph'
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD if edges else Marker.DELETE
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.006
    marker.color.a = 1.0
    grey = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8)
    magenta = ColorRGBA(r=1.0, g=0.1, b=0.9, a=1.0)
    for edge in edges:
        if edge.i >= len(poses) or edge.j >= len(poses):
            continue
        for k in (edge.i, edge.j):
            p = poses[k][:3, 3]
            marker.points.append(Point(x=float(p[0]), y=float(p[1]),
                                       z=float(p[2])))
            marker.colors.append(magenta if edge.kind == 'loop' else grey)
    return marker


def pnp_pose(landmarks, pixels, k_matrix, reprojection_px=6.0,
             min_points=6, iterations=300):
    """
    Camera pose from 3D landmarks ↔ 2D pixels: RANSAC PnP + LM refine.

    Returns (T_world_optical, inlier_count) or None. `landmarks` are in
    some world frame (the store's odom coordinates); the pose comes back
    as that world's transform of the optical frame — the pose of the
    camera that saw `pixels`. Pure function around cv2 so it is
    testable on a synthetic scene.
    """
    obj = np.ascontiguousarray(landmarks, dtype=np.float64).reshape(-1, 3)
    img = np.ascontiguousarray(pixels, dtype=np.float64).reshape(-1, 2)
    if len(obj) < min_points or len(obj) != len(img):
        return None
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj, img, np.asarray(k_matrix, dtype=np.float64), None,
        iterationsCount=iterations, reprojectionError=reprojection_px,
        confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None or len(inliers) < min_points:
        return None
    idx = inliers.ravel()
    rvec, tvec = cv2.solvePnPRefineLM(
        obj[idx], img[idx], np.asarray(k_matrix, dtype=np.float64), None,
        rvec, tvec)
    r_optical_world, _ = cv2.Rodrigues(rvec)
    t_optical_world = make_transform(r_optical_world, tvec.ravel())
    return invert(t_optical_world), int(len(idx))


def rays_from_pixels(pixels, k_matrix):
    """
    Unproject Nx2 pixel coordinates to unit bearing rays through K.

    A bearing ray is the direction from the optical centre through a
    pixel, in the optical frame — all a rotation estimator needs; depth
    would only scale the ray without turning it.
    """
    fx, fy = k_matrix[0, 0], k_matrix[1, 1]
    cx, cy = k_matrix[0, 2], k_matrix[1, 2]
    rays = np.column_stack([
        (pixels[:, 0] - cx) / fx,
        (pixels[:, 1] - cy) / fy,
        np.ones(len(pixels))])
    return rays / np.linalg.norm(rays, axis=1, keepdims=True)


def kabsch(prev_rays, curr_rays):
    """
    Best-fit rotation R with curr ≈ R @ prev — one SVD, no iteration.

    The orthogonal Procrustes solution. The determinant guard matters:
    for degenerate (e.g. coplanar) ray bundles the raw SVD answer can be
    a reflection, which is not a rotation any camera can perform.
    """
    u, _, vt = np.linalg.svd(prev_rays.T @ curr_rays)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    return rotation


def residual_angles(rotation, prev_rays, curr_rays):
    """Per-pair angle (rad) between R @ prev and curr — the fit error."""
    cosines = np.sum(curr_rays * (prev_rays @ rotation.T), axis=1)
    return np.arccos(np.clip(cosines, -1.0, 1.0))


def estimate_rotation(prev_rays, curr_rays, min_pairs=8,
                      max_residual_rad=0.03, refit_rounds=2,
                      drop_fraction=0.2):
    """
    Robust rotation from matched ray pairs, or None if untrustworthy.

    Kabsch, then a couple of reject-worst-and-refit rounds standing in
    for RANSAC: descriptor matching leaves a few percent of false pairs,
    which land in the residual tail and get dropped. Returning None on a
    thin or inconsistent match set is the fail-loudly rule — a caller
    must never mistake garbage for a confident estimate.
    """
    prev = np.asarray(prev_rays, dtype=np.float64)
    curr = np.asarray(curr_rays, dtype=np.float64)
    if len(prev) < min_pairs:
        return None
    rotation = kabsch(prev, curr)
    for _ in range(refit_rounds):
        residuals = residual_angles(rotation, prev, curr)
        keep_count = int(np.ceil(len(prev) * (1.0 - drop_fraction)))
        keep = np.argsort(residuals)[:keep_count]
        prev, curr = prev[keep], curr[keep]
        rotation = kabsch(prev, curr)
    residuals = residual_angles(rotation, prev, curr)
    if len(prev) < min_pairs or float(residuals.mean()) > max_residual_rad:
        return None
    return rotation


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
        # How many recent frames' descriptors to match against. 1 = strict
        # frame-to-frame; longer forgives detection flicker at the feature
        # cap. Read once at startup — it sizes the deque.
        self.declare_parameter('match_window', 10)
        # Estimator gates: fewer consecutive-pair matches than this, or a
        # worse mean residual after refits, and no rotation is trusted.
        self.declare_parameter('min_matched_pairs', 8)
        self.declare_parameter('max_residual_rad', 0.03)
        # REP-105: one parent per frame. When rgbd_odometry owns
        # odom → base_link (live mesh plan P3, `odom:=rgbd`), this
        # node's compass must not fight it — the orientation topic
        # keeps publishing either way.
        self.declare_parameter('publish_tf', True)
        # Relocalization (see the module docstring). Novelty spacing and
        # cap size the room memory; the rest gate when recognition may
        # replace dead reckoning — a wrong snap is worse than waiting.
        self.declare_parameter('keyframe_novelty_deg', 18.0)
        self.declare_parameter('keyframe_cap', 100)
        # Frames without consecutive pairs before tracking counts as
        # lost and recognition starts; retries are rate-limited because
        # a full store query is tens of milliseconds.
        self.declare_parameter('relocalize_after', 10)
        self.declare_parameter('relocalize_retry', 5)
        self.declare_parameter('relocalize_min_pairs', 12)
        self.declare_parameter('relocalize_margin', 1.3)
        # rgbd mode only: how stale the cached /depth may be for 3D
        # landmark capture/recovery, and the odometry-vs-recovered
        # discrepancy below which no snap is sent (rgbd is fine).
        self.declare_parameter('depth_max_age', 1.0)
        self.declare_parameter('min_correction_m', 0.3)
        self.declare_parameter('min_correction_deg', 10.0)
        # Persistence (relocalization plan P3): where ~/save_map writes,
        # and an optional map to load at startup. A loaded map arms
        # recognition immediately — a cold-started session has no odom
        # history, so the first successful match *defines* where odom is
        # relative to the stored room (the map's frame wins).
        self.declare_parameter('map_dir', 'maps')
        self.declare_parameter('map_path', '')
        # SLAM plan P1: the second novelty axis (metres between
        # same-heading keyframes) and the loop-detection gates — how
        # often the store is queried, which keyframes are too recent to
        # count as a revisit, how many rigid-fit inliers (and how tight)
        # a closure needs, and the drift beyond which a "closure" is
        # more likely a lookalike wall than a loop.
        self.declare_parameter('keyframe_novelty_m', 0.3)
        # How long a depth frame may wait for its odometry TF before it
        # is dropped, and how old it must be before the first lookup is
        # tried (rgbd_odometry stamps TF at the image stamp, ~0.2 s late).
        self.declare_parameter('sync_min_delay_s', 0.25)
        self.declare_parameter('sync_max_delay_s', 1.5)
        self.declare_parameter('loop_query_every', 3)
        self.declare_parameter('loop_min_age_s', 5.0)
        self.declare_parameter('loop_exclude_recent', 2)
        self.declare_parameter('loop_min_inliers', 30)
        self.declare_parameter('loop_reprojection_px', 6.0)
        self.declare_parameter('loop_crosscheck_m', 0.15)
        self.declare_parameter('loop_crosscheck_deg', 8.0)
        self.declare_parameter('loop_max_drift_m', 1.0)
        self.declare_parameter('loop_max_drift_deg', 45.0)
        self.declare_parameter('loop_cooldown_s', 3.0)
        # SLAM plan P2: the backend. Whether to optimise on each loop
        # edge, the Huber width (chi units) that keeps one wrong closure
        # from folding the map, the edge sigmas (odometry: locally
        # trusted; loop: a rigid fit on ~50 landmarks) and whether this
        # node owns map → odom (never alongside RTAB-Map's SLAM node —
        # one owner per frame).
        self.declare_parameter('graph_optimize', True)
        self.declare_parameter('graph_huber', 2.0)
        self.declare_parameter('graph_odom_sigma_m', 0.02)
        self.declare_parameter('graph_odom_sigma_deg', 1.0)
        self.declare_parameter('graph_loop_sigma_m', 0.03)
        self.declare_parameter('graph_loop_sigma_deg', 2.0)
        self.declare_parameter('publish_map_tf', False)
        self.declare_parameter('map_frame', 'map')
        self.orb = cv2.ORB_create(
            nfeatures=self.get_parameter('max_features').value)
        # Brute force is fine at <=500 features; NORM_HAMMING because ORB
        # descriptors are binary strings, not float vectors. crossCheck
        # keeps only mutual best matches — one-to-one by construction,
        # which prunes most false pairings before any geometry exists.
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.descriptor_window = deque(
            maxlen=self.get_parameter('match_window').value)

        # Estimator state: strict previous-frame keypoints (pixel coords
        # aligned with descriptors — the display window keeps neither),
        # the cached K, the composed orientation, and the dup filter.
        self.prev_points = None
        self.prev_descriptors = None
        self.k_matrix = None
        self.orientation = np.eye(3)
        self.prev_frame_crc = None
        self.dup_skipped = 0

        # Relocalization state. rgbd_mode is read once: it decides which
        # infrastructure exists (TF listener, depth cache, the snap
        # service client), not just per-frame behaviour.
        self.rgbd_mode = not self.get_parameter('publish_tf').value
        self.store = KeyframeStore(
            novelty_deg=self.get_parameter('keyframe_novelty_deg').value,
            cap=self.get_parameter('keyframe_cap').value,
            novelty_m=self.get_parameter('keyframe_novelty_m').value)
        # The pose graph (SLAM plan P1/P2): node k's optimised pose lives
        # in graph.poses[k]; node_odom[k] is the odom → base_link the
        # odometry reported at capture (the frame the keyframe's
        # landmarks are stored in), node_stamp[k] the frame's header
        # stamp (so /world/trajectory can be scored against a bag or a
        # ground truth), node_wall[k] the receipt clock (loop-age gate).
        self.graph = PoseGraph()
        self.node_odom = []
        self.node_stamp = []
        self.node_wall = []
        self.loop_edges = []            # (i, j) for the marker
        self.map_odom = np.eye(4)       # the correction, T_map_odom
        self.frames_since_query = 0
        self.last_loop_wall = None
        self.current_stamp = None
        # Exact-sync geometry (SLAM plan P1): rgbd keyframes and loop
        # checks are built from a frame's *own* depth and the odometry
        # TF *at its stamp*, not the latest of each — depth lands ~80 ms
        # and the odom TF ~200 ms after the RGB frame, and at hand-pan
        # speed "latest" misplaces every stored landmark by degrees.
        # recent_frames keeps ORB output per stamp; pending_depth queues
        # depth frames until their TF exists (a 10 Hz timer drains it).
        self.recent_frames = {}          # stamp_ns -> (points, descriptors)
        self.recent_order = deque(maxlen=120)
        self.pending_depth = deque(maxlen=20)   # (stamp_ns, depth, wall_ns)
        self.tracking_healthy = False
        self.lost_frames = 0
        self.needs_relocalization = False
        self.retry_countdown = 0
        # Set once tracking has ever succeeded: from then on a frame with
        # nothing to match is a *lost* frame, not "nothing to track yet".
        # Found by the black-fill gate bag (2026-08-18): a lens covered by
        # something featureless yields no descriptors at all, so
        # `could_estimate` alone never counted the blackout and the
        # detector woke up in a reset odometry believing it was healthy.
        self.was_tracking = False
        map_path = os.path.expanduser(self.get_parameter('map_path').value)
        if map_path:
            # Fail loudly: a misspelt path silently starting an empty
            # room would defeat the whole point of loading one.
            self.load_map(map_path)
            self.needs_relocalization = True
            self.get_logger().info(
                f'loaded {len(self.store)} keyframes and a {len(self.graph)}'
                f'-node graph from {map_path} — relocalizing before '
                'trusting any pose')
        self.depth_image = None
        self.depth_received_at = None
        self.tf_buffer = None
        self.reset_pose_client = None
        self.reset_pose_type = None
        self.t_base_optical = None
        if self.rgbd_mode:
            # The pose authority is rgbd_odometry, reached two ways:
            # TF (latest-only lookups — this camera's stamps are faulted
            # by ~0.73 s, docs/info/camera.md#timestamps) for capture
            # poses, and its reset_odom_to_pose service for the snap.
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.sub_depth = self.create_subscription(
                Image, 'depth', self.on_depth, BIG_FRAME_QOS)
            self.create_timer(0.1, self.process_pending_depth)
            try:
                # Imported lazily: rtabmap_msgs is a dev-box package;
                # the Pi builds this package without it and never runs
                # this node in rgbd mode.
                from rtabmap_msgs.srv import ResetPose
                self.reset_pose_type = ResetPose
                self.reset_pose_client = self.create_client(
                    ResetPose, '/reset_odom_to_pose')
            except ImportError:
                self.get_logger().warn(
                    'rtabmap_msgs not available — relocalization will '
                    'recognise views but cannot snap the odometry')

        self.sub = self.create_subscription(
            CompressedImage, 'image_raw/compressed',
            self.on_frame, BIG_FRAME_QOS)
        # Cached once; usb_cam republishes it with every frame. An
        # all-zero K (the milestone-6 bag trap) is rejected, not cached.
        self.sub_info = self.create_subscription(
            CameraInfo, 'camera_info', self.on_camera_info, BIG_FRAME_QOS)
        # Published by hand on the conventional <topic>/compressed name so
        # stock viewers find it — image_transport is C++-only and gives a
        # Python publisher no automatic compressed variant.
        self.pub_image = self.create_publisher(
            CompressedImage, 'keypoints/compressed', BIG_FRAME_QOS)
        self.pub_count = self.create_publisher(
            Int32, 'keypoints/count', BIG_FRAME_QOS)
        self.pub_matched = self.create_publisher(
            Int32, 'keypoints/matched', BIG_FRAME_QOS)
        self.pub_keyframes = self.create_publisher(
            Int32, 'keypoints/keyframes', BIG_FRAME_QOS)
        self.pub_pose = self.create_publisher(
            PoseStamped, 'camera/orientation', BIG_FRAME_QOS)
        # A TransformBroadcaster is just a publisher on /tf; tf2 consumers
        # (RViz included) assemble the tree from everyone's contributions.
        # This node owns odom → base_link — REP-105's slot for drifting
        # local odometry — while the static base_link → camera chain stays
        # with camera.launch.py; neither fights the other.
        self.tf_broadcaster = TransformBroadcaster(self)
        # The repo's first service: re-zero the orientation without
        # restarting the node — the drift strategy, in lieu of loop
        # closure. '~/reset' resolves to /keypoint_detector/reset.
        self.create_service(Trigger, '~/reset', self.on_reset)
        # P3: the room outlives the session — plain npz, no pickle.
        self.create_service(Trigger, '~/save_map', self.on_save_map)
        # P4: RViz sees the room memory. Slow, latched — a debugging
        # view, not a stream.
        self.pub_keyframe_marker = self.create_publisher(
            Marker, 'world/keyframes', LATCHED_QOS)
        self.create_timer(2.0, self.publish_keyframe_marker)
        # SLAM plan P1/P2: the graph as RViz sees it — optimised keyframe
        # poses as a Path (in the map frame), edges as a LINE_LIST (odom
        # edges grey, loop edges magenta) — and, when this node owns it,
        # map → odom on a timer (TF wants a heartbeat; identity until
        # the first closure, so the tree exists from the start).
        self.pub_path = self.create_publisher(
            Path, 'world/trajectory', LATCHED_QOS)
        # The same nodes' odom → base_link at capture, same order and
        # stamps: a consumer (tsdf_mesher's rebuild) turns the pair into
        # a per-node correction without needing a TF history.
        self.pub_path_odom = self.create_publisher(
            Path, 'world/trajectory_odom', LATCHED_QOS)
        self.pub_graph_marker = self.create_publisher(
            Marker, 'world/keyframe_graph', LATCHED_QOS)
        if self.get_parameter('publish_map_tf').value:
            self.create_timer(0.1, self.publish_map_tf)

    def on_camera_info(self, msg: CameraInfo):
        if self.k_matrix is not None:
            return
        k = np.array(msg.k).reshape(3, 3)
        if k[0, 0] <= 0.0:
            self.get_logger().warn(
                'camera_info carries an all-zero K — ignoring it',
                throttle_duration_sec=30.0)
            return
        self.k_matrix = k
        self.get_logger().info(
            f'intrinsics cached: fx={k[0, 0]:.1f} fy={k[1, 1]:.1f} '
            f'cx={k[0, 2]:.1f} cy={k[1, 2]:.1f}')

    def on_reset(self, request, response):
        self.orientation = np.eye(3)
        # The room memory goes with it: stored geometry is expressed in
        # the odom the old orientation defined, so keeping it would mean
        # relocalizing into a frame that no longer exists.
        self.store.clear()
        self.lost_frames = 0
        self.needs_relocalization = False
        self.was_tracking = False
        self.graph = PoseGraph()
        self.node_odom, self.node_stamp, self.node_wall = [], [], []
        self.loop_edges = []
        self.map_odom = np.eye(4)
        self.publish_graph()
        response.success = True
        response.message = ('orientation reset to identity, keyframes and '
                            'pose graph cleared')
        return response

    def publish_keyframe_marker(self):
        self.pub_keyframe_marker.publish(keyframe_marker(
            self.store.keyframes, self.get_clock().now().to_msg()))

    def on_save_map(self, request, response):
        if len(self.store) == 0:
            response.success = False
            response.message = 'no keyframes stored yet — nothing to save'
            return response
        map_dir = self.get_parameter('map_dir').value
        os.makedirs(map_dir, exist_ok=True)
        path = os.path.join(map_dir, time.strftime('room_%Y%m%d-%H%M%S.npz'))
        self.save_map(path)
        response.success = True
        response.message = (f'{path}: {len(self.store)} keyframes, '
                            f'{len(self.graph)} graph nodes, '
                            f'{len(self.loop_edges)} loop edges')
        self.get_logger().info(f'saved {response.message}')
        return response

    # ------------------------------------------------------------------
    # SLAM plan P4: the graph outlives the session with the keyframes.
    # One npz of plain arrays (no pickle): the store's own columns plus
    # the graph — optimised poses, the odom pose at each node's capture,
    # stamps, and every edge with its measurement, information and kind.
    # A loaded graph is extended, not restarted: after the cold-start
    # relocalization adopts the map's frame (relocalization plan P3),
    # the next keyframe chains an odometry edge from the last stored
    # node, and loop closures against stored keyframes tie the sessions.

    def graph_arrays(self):
        n, e = len(self.graph), len(self.graph.edges)
        return {
            'graph_poses': np.array(self.graph.poses).reshape(n, 4, 4),
            'graph_node_odom': np.array(self.node_odom).reshape(n, 4, 4),
            'graph_node_stamp': np.array(
                [-1.0 if t is None else t for t in self.node_stamp]),
            'graph_edge_i': np.array([ed.i for ed in self.graph.edges],
                                     dtype=np.int64),
            'graph_edge_j': np.array([ed.j for ed in self.graph.edges],
                                     dtype=np.int64),
            'graph_edge_measurement': np.array(
                [ed.measurement for ed in self.graph.edges]).reshape(e, 4, 4),
            'graph_edge_information': np.array(
                [ed.information for ed in self.graph.edges]).reshape(e, 6, 6),
            'graph_edge_loop': np.array(
                [ed.kind == 'loop' for ed in self.graph.edges], dtype=bool),
            'map_odom': self.map_odom,
        }

    def save_map(self, path):
        np.savez_compressed(path, **self.store.to_arrays(),
                            **self.graph_arrays())

    def load_map(self, path):
        with np.load(path, allow_pickle=False) as data:
            arrays = dict(data.items())
        self.store = KeyframeStore.from_arrays(arrays)
        self.graph = PoseGraph()
        self.node_odom, self.node_stamp, self.node_wall = [], [], []
        self.loop_edges = []
        if 'graph_poses' not in arrays:
            return
        for pose, odom, stamp in zip(arrays['graph_poses'],
                                     arrays['graph_node_odom'],
                                     arrays['graph_node_stamp']):
            self.graph.add_node(pose)
            self.node_odom.append(np.array(odom))
            self.node_stamp.append(None if stamp < 0 else float(stamp))
            self.node_wall.append(0.0)      # old enough for any loop query
        for i, j, z, info, loop in zip(
                arrays['graph_edge_i'], arrays['graph_edge_j'],
                arrays['graph_edge_measurement'],
                arrays['graph_edge_information'], arrays['graph_edge_loop']):
            self.graph.add_edge(int(i), int(j), z, info,
                                'loop' if loop else 'odom')
            if loop:
                self.loop_edges.append((int(i), int(j)))
        if 'map_odom' in arrays:
            self.map_odom = np.array(arrays['map_odom'])

    def on_depth(self, msg: Image):
        self.depth_image = np.frombuffer(msg.data, np.float32).reshape(
            msg.height, msg.width)
        # Receipt clock, never header.stamp — the 0.73 s stamp fault.
        self.depth_received_at = self.get_clock().now()
        self.pending_depth.append(
            (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
             self.depth_image, self.depth_received_at.nanoseconds))

    def on_frame(self, msg: CompressedImage):
        entry = self.get_clock().now()
        self.current_stamp = msg.header.stamp

        # Byte-identical to the previous frame = usb_cam's duplicate
        # republish. Zero motion, zero information: skip before even
        # decoding.
        crc = zlib.crc32(bytes(msg.data))
        if crc == self.prev_frame_crc:
            self.dup_skipped += 1
            return
        self.prev_frame_crc = crc

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
        if self.rgbd_mode:
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 \
                + msg.header.stamp.nanosec
            if len(self.recent_order) == self.recent_order.maxlen:
                self.recent_frames.pop(self.recent_order[0], None)
            self.recent_order.append(stamp_ns)
            self.recent_frames[stamp_ns] = (
                np.array([kp.pt for kp in keypoints],
                         dtype=np.float64).reshape(-1, 2), descriptors)

        # Match against the pooled descriptors of the window, then reject
        # matches whose Hamming distance says "similar-looking corner",
        # not "same physical point". The same physical feature appears in
        # the pool once per recent frame; crossCheck still yields at most
        # one match per current keypoint, which is all "re-observed?"
        # needs.
        max_distance = self.get_parameter('match_max_distance').value
        matched_idx = set()
        if descriptors is not None and self.descriptor_window:
            train = np.vstack(self.descriptor_window)
            matches = self.matcher.match(descriptors, train)
            matched_idx = {m.queryIdx for m in matches
                           if m.distance <= max_distance}
        if descriptors is not None:
            self.descriptor_window.append(descriptors)

        matched = [kp for i, kp in enumerate(keypoints) if i in matched_idx]
        fresh = [kp for i, kp in enumerate(keypoints)
                 if i not in matched_idx]

        self.update_orientation(keypoints, descriptors, max_distance)

        # Yellow = new this frame, green = re-observed recently (drawn
        # second, so tracked points win any overlap).
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

        keyframes = Int32()
        keyframes.data = len(self.store)
        self.pub_keyframes.publish(keyframes)

        # Cost measured against our own clock only — this camera's header
        # stamps lag ~0.73 s by fault (docs/info/camera.md#timestamps) and
        # prove nothing about the pipeline.
        done = self.get_clock().now()
        self.get_logger().info(
            f'{len(keypoints)} keypoints ({len(matched)} matched in window), '
            f'{self.dup_skipped} dups skipped, '
            f'{(done - entry).nanoseconds / 1e6:.1f} ms/frame',
            throttle_duration_sec=5.0)

    def update_orientation(self, keypoints, descriptors, max_distance):
        """
        Estimate this frame's rotation and publish the composed pose.

        Strict consecutive-pair matching against the previous frame only
        — the display's window matching answers "seen recently?", which
        is the wrong question for motion between exactly two frames.
        """
        points = np.array([kp.pt for kp in keypoints],
                          dtype=np.float64).reshape(-1, 2)
        rotation = None
        if self.k_matrix is None:
            self.get_logger().warn(
                'no intrinsics yet — detecting only, no orientation',
                throttle_duration_sec=10.0)
        elif descriptors is not None and self.prev_descriptors is not None:
            pairs = [m for m in
                     self.matcher.match(descriptors, self.prev_descriptors)
                     if m.distance <= max_distance]
            if pairs:
                curr_px = points[[m.queryIdx for m in pairs]]
                prev_px = self.prev_points[[m.trainIdx for m in pairs]]
                rotation = estimate_rotation(
                    rays_from_pixels(prev_px, self.k_matrix),
                    rays_from_pixels(curr_px, self.k_matrix),
                    min_pairs=self.get_parameter('min_matched_pairs').value,
                    max_residual_rad=self.get_parameter(
                        'max_residual_rad').value)
        # Whether this frame *could* have estimated (both frames carried
        # descriptors and K exists) — the difference between "tracking
        # lost" and "nothing to track yet". Read before prev is replaced.
        could_estimate = (self.k_matrix is not None
                          and descriptors is not None
                          and self.prev_descriptors is not None)
        self.prev_points = points
        self.prev_descriptors = descriptors

        if rotation is not None:
            # estimate_rotation maps prev rays onto curr rays; the camera
            # itself rotated by the inverse, so the world-side composition
            # multiplies by R transposed.
            self.orientation = self.orientation @ rotation.T
        if self.k_matrix is None:
            return

        self.track_room_memory(rotation is not None, could_estimate,
                               points, descriptors)

        # Conjugate optical-axes orientation into base_link axes and
        # publish. Our own stamp: the source stamps lag ~0.73 s by fault,
        # and this pose is a now-estimate anyway.
        base = BASE_FROM_OPTICAL @ self.orientation @ BASE_FROM_OPTICAL.T
        x, y, z, w = quaternion_from_rotation(base)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.orientation.x = float(x)
        pose.pose.orientation.y = float(y)
        pose.pose.orientation.z = float(z)
        pose.pose.orientation.w = float(w)
        self.pub_pose.publish(pose)

        # The same rotation again as TF (translation stays zero — this
        # odometer measures orientation only). One estimate, two
        # representations: the topic for programs, the transform for the
        # tf2 tree RViz renders.
        if not self.get_parameter('publish_tf').value:
            return
        tf = TransformStamped()
        tf.header.stamp = pose.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def track_room_memory(self, rotation_ok, could_estimate, points,
                          descriptors):
        """Feed the store while healthy; switch to recognition when lost."""
        if rotation_ok:
            self.lost_frames = 0
            self.was_tracking = True
        elif could_estimate or self.was_tracking:
            self.lost_frames += 1
            if (self.lost_frames
                    == self.get_parameter('relocalize_after').value):
                self.needs_relocalization = True
                self.retry_countdown = 0
                self.get_logger().warn(
                    f'tracking lost for {self.lost_frames} frames — '
                    f'watching for a known view '
                    f'({len(self.store)} keyframes stored)')
        if self.needs_relocalization:
            # Full store queries cost tens of ms — retry on a cadence,
            # not per frame; between retries the compass composes from
            # its (possibly corrupt) baseline and the snap will replace
            # it absolutely anyway.
            if self.retry_countdown > 0:
                self.retry_countdown -= 1
            elif descriptors is not None and len(points):
                self.retry_countdown = self.get_parameter(
                    'relocalize_retry').value
                if self.attempt_relocalization(points, descriptors):
                    self.needs_relocalization = False
                    self.lost_frames = 0
        self.tracking_healthy = rotation_ok and not self.needs_relocalization
        if self.tracking_healthy and not self.rgbd_mode:
            # kp mode stores straight from the frame (rays need no depth
            # and the compass is this node's own, so nothing is late).
            # rgbd keyframes wait for their exact depth + TF instead
            # (process_pending_depth).
            self.maybe_store_keyframe(points, descriptors)

    def maybe_store_keyframe(self, points, descriptors, exact=None):
        """Offer the frame to the store; the novelty gate decides."""
        if descriptors is None or len(points) == 0:
            return
        if self.rgbd_mode:
            # Geometry and pose come from the odometry's own authority.
            # `exact` = (depth, T_odom_optical, T_odom_base, stamp) for
            # this very frame (process_pending_depth); without it fall
            # back to latest depth + latest TF. No depth or no odom yet →
            # no keyframe, no fallback guessing.
            if exact is not None:
                depth, transform, t_odom_base, stamp = exact
            else:
                depth, t_odom_base, stamp = None, None, None
                transform = self._lookup('odom', 'camera_optical_frame')
            if transform is None:
                return
            pts_cam, valid = self._depth_points(points, depth)
            min_pairs = self.get_parameter('relocalize_min_pairs').value
            if pts_cam is None or valid.sum() < 2 * min_pairs:
                return
            slot = self.store.maybe_add(
                descriptors[valid], transform[:3, 2],
                points=transform_points(transform, pts_cam[valid]),
                pose=transform)
            view_dir = transform[:3, 2]
            if slot is not None:
                self.add_graph_node(slot, t_odom_base, stamp)
        else:
            # The compass's own frame: rays rotated into odom-optical by
            # the composed orientation (R maps camera-frame vectors into
            # the odom the compass started in).
            rays_cam = rays_from_pixels(points, self.k_matrix)
            slot = self.store.maybe_add(
                descriptors, self.orientation[:, 2],
                rays=rays_cam @ self.orientation.T)
            view_dir = BASE_FROM_OPTICAL @ self.orientation[:, 2]
        if slot is not None:
            yaw = np.degrees(np.arctan2(view_dir[1], view_dir[0]))
            self.get_logger().info(
                f'keyframe {slot} stored (yaw {yaw:.0f}°, '
                f'store {len(self.store)}/'
                f'{self.get_parameter("keyframe_cap").value})')

    # ------------------------------------------------------------------
    # SLAM plan P1/P2: the keyframe graph, loop detection, the backend.

    def add_graph_node(self, slot, t_odom_base=None, stamp_ns=None):
        """Give a just-stored rgbd keyframe a pose-graph node (+ odom edge)."""
        if t_odom_base is None:
            t_odom_base = self._lookup('odom', 'base_link')
        if t_odom_base is None:
            return None
        # Initial estimate in the map frame: the current correction
        # applied to the odometry pose, so a new node lands consistent
        # with the already-optimised graph rather than in raw odom.
        node = self.graph.add_node(self.map_odom @ t_odom_base)
        self.node_odom.append(t_odom_base)
        if stamp_ns is None and self.current_stamp is not None:
            stamp_ns = self.current_stamp.sec * 1_000_000_000 \
                + self.current_stamp.nanosec
        self.node_stamp.append(
            None if stamp_ns is None else stamp_ns * 1e-9)
        self.node_wall.append(self.get_clock().now().nanoseconds * 1e-9)
        self.store.keyframes[slot].node_id = node
        if node > 0:
            # The odometry edge: what rgbd_odometry says the motion
            # between the two captures was. Odom is continuous, so the
            # relative pose is valid even after map → odom has moved.
            z = invert(self.node_odom[node - 1]) @ t_odom_base
            self.graph.add_edge(
                node - 1, node, z, self._odom_information(), kind='odom')
        self.publish_graph()
        return node

    def _odom_information(self):
        return information_matrix(
            self.get_parameter('graph_odom_sigma_m').value,
            np.radians(self.get_parameter('graph_odom_sigma_deg').value))

    def _loop_information(self):
        return information_matrix(
            self.get_parameter('graph_loop_sigma_m').value,
            np.radians(self.get_parameter('graph_loop_sigma_deg').value))

    def process_pending_depth(self):
        """
        Drain the depth queue: exact (frame, depth, TF-at-stamp) triples.

        Each depth frame waits at least sync_min_delay_s (its odometry
        TF is stamped at the image stamp and arrives ~0.2 s later), then
        is paired with the ORB output of the frame with the same stamp
        and the odom → base_link / camera_optical_frame transforms *at
        that stamp* (tf2 interpolates between odometry samples). Only
        then are keyframes stored and loops checked — landmark geometry
        built this way is exact instead of ~200 ms stale.
        """
        if not self.pending_depth:
            return
        now_ns = self.get_clock().now().nanoseconds
        min_wait = self.get_parameter('sync_min_delay_s').value * 1e9
        max_wait = self.get_parameter('sync_max_delay_s').value * 1e9
        while self.pending_depth:
            stamp_ns, depth, received_ns = self.pending_depth[0]
            age = now_ns - received_ns
            if age < min_wait:
                break
            frame = self.recent_frames.get(stamp_ns)
            if frame is None:
                self.pending_depth.popleft()      # no ORB for this stamp
                continue
            t_odom_base = self._lookup_at('odom', 'base_link', stamp_ns)
            if t_odom_base is None:
                if age < max_wait:
                    break                          # TF not there yet
                self.pending_depth.popleft()
                continue
            t_odom_optical = self._lookup_at('odom', 'camera_optical_frame',
                                             stamp_ns)
            self.pending_depth.popleft()
            if t_odom_optical is None:
                continue
            points, descriptors = frame
            if not self.tracking_healthy or descriptors is None:
                continue
            exact = (depth, t_odom_optical, t_odom_base, stamp_ns)
            self.maybe_store_keyframe(points, descriptors, exact)
            self.maybe_detect_loop(points, descriptors, exact)

    def maybe_detect_loop(self, points, descriptors, exact):
        """
        Loop-closure detection while tracking is healthy (rgbd mode).

        Every `loop_query_every` frames: match the live view against the
        keyframes old enough to be a revisit, verify the winner with a
        rigid fit of live 3D landmarks onto the keyframe's stored ones
        (an inlier count, not just a match count — descriptors lie,
        geometry less so), reject a "closure" whose implied drift is
        absurd, and otherwise store the frame as a keyframe node with a
        loop edge to the recognised one. Then the backend runs.
        """
        if not self.rgbd_mode or descriptors is None or len(points) == 0:
            return
        depth, t_odom_optical_now, t_odom_base_now, stamp_ns = exact
        self.frames_since_query += 1
        if self.frames_since_query < self.get_parameter(
                'loop_query_every').value:
            return
        self.frames_since_query = 0
        now = self.get_clock().now().nanoseconds * 1e-9
        cooldown = self.get_parameter('loop_cooldown_s').value
        if self.last_loop_wall is not None and now - self.last_loop_wall \
                < cooldown:
            return
        n_nodes = len(self.graph)
        recent = self.get_parameter('loop_exclude_recent').value
        min_age = self.get_parameter('loop_min_age_s').value
        exclude = set()
        for i, kf in enumerate(self.store.keyframes):
            if kf.node_id < 0 or kf.points is None:
                exclude.add(i)
            elif kf.node_id >= n_nodes - recent:
                exclude.add(i)
            elif now - self.node_wall[kf.node_id] < min_age:
                exclude.add(i)
        if len(exclude) >= len(self.store.keyframes):
            return
        result = self.store.match(
            descriptors,
            max_distance=self.get_parameter('match_max_distance').value,
            min_pairs=self.get_parameter('relocalize_min_pairs').value,
            margin=self.get_parameter('relocalize_margin').value,
            exclude=exclude)
        if result is None:
            return
        best, kf_idx, query_idx = result
        keyframe = self.store.keyframes[best]
        # Geometric verification by PnP: the keyframe's stored 3D
        # landmarks (odom coordinates at its capture) against the live
        # frame's 2D pixels. One frame's depth instead of two (monocular
        # depth is the noisy input here), RANSAC instead of drop-worst
        # refits, and LM refinement on the inliers. The result is the
        # live camera's pose *as the keyframe's landmarks see it* — in
        # the odom coordinates those landmarks were stored in; relative
        # to the keyframe's own node that is a drift-free kf → now.
        pnp = pnp_pose(keyframe.points[kf_idx], points[query_idx],
                       self.k_matrix,
                       reprojection_px=self.get_parameter(
                           'loop_reprojection_px').value)
        if pnp is None:
            return
        t_odom_optical_implied, inliers = pnp
        if inliers < self.get_parameter('loop_min_inliers').value:
            self.get_logger().info(
                f'loop candidate kf {best} rejected: {inliers} PnP inliers',
                throttle_duration_sec=5.0)
            return
        # Cross-check with the 3D-3D rigid fit (live depth vs stored
        # landmarks): two independent geometries that disagree mean one
        # of the depths lied — refuse rather than guess.
        pts_cam, valid = self._depth_points(points[query_idx], depth)
        if pts_cam is not None and valid.sum() > 0:
            fit = rigid_transform_3d(
                pts_cam[valid], keyframe.points[kf_idx][valid],
                min_pairs=self.get_parameter('relocalize_min_pairs').value)
            if fit is not None:
                gap = invert(make_transform(*fit)) @ t_odom_optical_implied
                gap_m = float(np.linalg.norm(gap[:3, 3]))
                gap_deg = self._angle_between_deg(np.eye(3), gap[:3, :3])
                if (gap_m > self.get_parameter('loop_crosscheck_m').value
                        or gap_deg > self.get_parameter(
                            'loop_crosscheck_deg').value):
                    self.get_logger().warn(
                        f'loop candidate kf {best} rejected: PnP and 3D fit '
                        f'disagree by {gap_m:.3f} m / {gap_deg:.1f}°')
                    return
        if self.t_base_optical is None:
            self.t_base_optical = self._lookup('base_link',
                                               'camera_optical_frame')
            if self.t_base_optical is None:
                return
        t_odom_base_implied = t_odom_optical_implied @ invert(
            self.t_base_optical)
        t_kf = self.node_odom[keyframe.node_id]
        z_loop = invert(t_kf) @ t_odom_base_implied
        z_odom = invert(t_kf) @ t_odom_base_now
        drift = invert(z_odom) @ z_loop
        drift_m = float(np.linalg.norm(drift[:3, 3]))
        drift_deg = self._angle_between_deg(np.eye(3), drift[:3, :3])
        if (drift_m > self.get_parameter('loop_max_drift_m').value
                or drift_deg > self.get_parameter('loop_max_drift_deg').value):
            self.get_logger().warn(
                f'loop candidate kf {best} rejected: implied drift '
                f'{drift_m:.2f} m / {drift_deg:.1f}° is not a loop')
            return
        # Store the live frame as a keyframe *node* (novelty gate
        # bypassed — the graph needs a node exactly here) and connect it.
        all_pts, all_valid = self._depth_points(points, depth)
        if all_pts is None or all_valid.sum() < 2 * self.get_parameter(
                'relocalize_min_pairs').value:
            return
        slot = self.store.maybe_add(
            descriptors[all_valid], t_odom_optical_now[:3, 2],
            points=transform_points(t_odom_optical_now, all_pts[all_valid]),
            pose=t_odom_optical_now, force=True)
        node = self.add_graph_node(slot, t_odom_base_now, stamp_ns)
        if node is None:
            return
        self.graph.add_edge(keyframe.node_id, node, z_loop,
                            self._loop_information(), kind='loop')
        self.loop_edges.append((keyframe.node_id, node))
        self.last_loop_wall = now
        self.get_logger().info(
            f'loop closure kf {slot} -> kf {best}: {inliers} inliers, '
            f'drift {drift_m:.3f} m / {drift_deg:.1f}°')
        if self.get_parameter('graph_optimize').value:
            self.optimize_graph()

    def optimize_graph(self):
        """P2: run the backend, then move the correction to map → odom."""
        stats = self.graph.optimize(
            fixed=(0,), huber=self.get_parameter('graph_huber').value)
        newest = len(self.graph) - 1
        self.map_odom = self.graph.poses[newest] @ invert(
            self.node_odom[newest])
        corr_m = float(np.linalg.norm(self.map_odom[:3, 3]))
        corr_deg = self._angle_between_deg(np.eye(3), self.map_odom[:3, :3])
        self.get_logger().info(
            f'graph optimised: {len(self.graph)} nodes, '
            f'{len(self.loop_edges)} loops, chi2 {stats["chi2_before"]:.1f}'
            f' -> {stats["chi2_after"]:.1f} in {stats["iterations"]} it, '
            f'max shift {stats["max_shift_m"]:.3f} m / '
            f'{stats["max_shift_deg"]:.1f}°; map->odom now '
            f'{corr_m:.3f} m / {corr_deg:.1f}°')
        self.publish_graph()
        return stats

    def publish_map_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.get_parameter('map_frame').value
        tf.child_frame_id = 'odom'
        x, y, z = (float(v) for v in self.map_odom[:3, 3])
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        qx, qy, qz, qw = quaternion_from_rotation(self.map_odom[:3, :3])
        tf.transform.rotation.x = float(qx)
        tf.transform.rotation.y = float(qy)
        tf.transform.rotation.z = float(qz)
        tf.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(tf)

    def publish_graph(self):
        """Optimised keyframe poses as a Path, edges as a marker."""
        frame = self.get_parameter('map_frame').value
        # Header stamp zero = "latest TF" to RViz: these live in the map
        # frame, whose transform is published on a timer, and a marker
        # stamped 'now' would wait on a TF that lands 100 ms later.
        zero = Time().to_msg()
        self.pub_path.publish(path_msg(self.graph.poses, self.node_stamp,
                                       frame, zero))
        self.pub_path_odom.publish(path_msg(self.node_odom, self.node_stamp,
                                            'odom', zero))
        self.pub_graph_marker.publish(graph_marker(
            self.graph.poses, self.graph.edges, zero, frame))

    def attempt_relocalization(self, points, descriptors):
        """One recognition attempt; True clears the lost state."""
        result = self.store.match(
            descriptors,
            max_distance=self.get_parameter('match_max_distance').value,
            min_pairs=self.get_parameter('relocalize_min_pairs').value,
            margin=self.get_parameter('relocalize_margin').value)
        if result is None:
            return False
        best, kf_idx, query_idx = result
        keyframe = self.store.keyframes[best]
        if self.rgbd_mode:
            return self._snap_rgbd(keyframe, best, kf_idx, query_idx,
                                   points)
        return self._snap_orientation(keyframe, best, kf_idx, query_idx,
                                      points)

    def _snap_orientation(self, keyframe, best, kf_idx, query_idx, points):
        """Replace the composed orientation absolutely (kp mode)."""
        if keyframe.rays is None:
            return False
        rotation = estimate_rotation(
            keyframe.rays[kf_idx],
            rays_from_pixels(points[query_idx], self.k_matrix),
            min_pairs=self.get_parameter('relocalize_min_pairs').value,
            max_residual_rad=self.get_parameter('max_residual_rad').value)
        if rotation is None:
            return False
        # estimate_rotation gave R mapping stored odom rays onto current
        # camera rays — R_cam_odom; the camera's orientation in odom is
        # its transpose. Not composed: *replaced*.
        recovered = rotation.T
        correction = self._angle_between_deg(self.orientation, recovered)
        self.orientation = recovered
        self.get_logger().info(
            f'relocalized against keyframe {best}: orientation snapped, '
            f'correction {correction:.1f}°')
        return True

    def _snap_rgbd(self, keyframe, best, kf_idx, query_idx, points):
        """Recover 6-DoF and hand it to the pose's owner (rgbd mode)."""
        if keyframe.points is None or self.reset_pose_type is None:
            return False
        pts_cam, valid = self._depth_points(points[query_idx])
        if pts_cam is None or valid.sum() == 0:
            return False
        fit = rigid_transform_3d(
            pts_cam[valid], keyframe.points[kf_idx][valid],
            min_pairs=self.get_parameter('relocalize_min_pairs').value)
        if fit is None:
            return False
        # dst points live in odom, src in the current optical frame, so
        # the fit *is* T_odom_optical for the current camera.
        t_odom_optical = make_transform(*fit)
        if self.t_base_optical is None:
            self.t_base_optical = self._lookup('base_link',
                                               'camera_optical_frame')
            if self.t_base_optical is None:
                return False
        t_odom_base = t_odom_optical @ invert(self.t_base_optical)

        current = self._lookup('odom', 'base_link')
        delta_m, delta_deg = float('inf'), float('inf')
        if current is not None:
            delta_m = float(np.linalg.norm(
                t_odom_base[:3, 3] - current[:3, 3]))
            delta_deg = self._angle_between_deg(current[:3, :3],
                                                t_odom_base[:3, :3])
            if (delta_m < self.get_parameter('min_correction_m').value
                    and delta_deg < self.get_parameter(
                        'min_correction_deg').value):
                self.get_logger().info(
                    f'view recognised (keyframe {best}); odometry already '
                    f'consistent (Δ {delta_m:.2f} m, {delta_deg:.1f}°)')
                return True
        if (self.reset_pose_client is None
                or not self.reset_pose_client.service_is_ready()):
            self.get_logger().warn(
                'recovered a pose but /reset_odom_to_pose is not ready',
                throttle_duration_sec=5.0)
            return False
        roll, pitch, yaw = euler_from_rotation(t_odom_base[:3, :3])
        request = self.reset_pose_type.Request()
        request.x, request.y, request.z = (
            float(v) for v in t_odom_base[:3, 3])
        request.roll, request.pitch = float(roll), float(pitch)
        request.yaw = float(yaw)
        self.reset_pose_client.call_async(request)
        if current is None:
            # Cold start (P3): no odometry yet, so nothing to be "off"
            # from — the map's frame is being adopted, not corrected.
            self.get_logger().info(
                f'relocalized against keyframe {best}: adopting the '
                f"map's frame (no odometry yet)")
        else:
            self.get_logger().info(
                f'relocalized against keyframe {best}: snapping odometry '
                f'(Δ {delta_m:.2f} m, {delta_deg:.1f}°)')
        return True

    def _depth_points(self, pixels, depth=None):
        """Nx2 pixels → (Nx3 optical-frame points, valid mask), or None."""
        if depth is None:
            max_age = self.get_parameter('depth_max_age').value
            if (self.depth_image is None
                    or (self.get_clock().now()
                        - self.depth_received_at).nanoseconds
                    > max_age * 1e9):
                return None, None
            depth = self.depth_image
        height, width = depth.shape
        u = np.clip(pixels[:, 0].astype(int), 0, width - 1)
        v = np.clip(pixels[:, 1].astype(int), 0, height - 1)
        z = depth[v, u].astype(np.float64)
        valid = (z > 0.05) & (z < 20.0)
        fx, fy = self.k_matrix[0, 0], self.k_matrix[1, 1]
        cx, cy = self.k_matrix[0, 2], self.k_matrix[1, 2]
        return np.column_stack([(pixels[:, 0] - cx) * z / fx,
                                (pixels[:, 1] - cy) * z / fy, z]), valid

    def _lookup_at(self, target, source, stamp_ns):
        """TF at an exact stamp (interpolated) as a 4x4, or None."""
        return self._lookup(target, source, Time(nanoseconds=int(stamp_ns)))

    def _lookup(self, target, source, when=None):
        """TF lookup (latest by default) as a 4x4, or None if unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, Time() if when is None else when)
        except TransformException:
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        return make_transform(
            rotation_from_quaternion(q.x, q.y, q.z, q.w),
            np.array([t.x, t.y, t.z]))

    @staticmethod
    def _angle_between_deg(r_a, r_b):
        cosine = (np.trace(r_a.T @ r_b) - 1.0) / 2.0
        return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


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
