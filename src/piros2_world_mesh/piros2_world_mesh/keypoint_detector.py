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

Runs on the dev box against the compressed stream already in flight; only
JPEG ever crosses the Wi-Fi.
"""

from collections import deque
import zlib

import cv2
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from piros2_world_mesh.keyframe_store import KeyframeStore
from piros2_world_mesh.se3 import (BASE_FROM_OPTICAL, euler_from_rotation,
                                   invert, make_transform,
                                   quaternion_from_rotation,
                                   rigid_transform_3d,
                                   rotation_from_quaternion,
                                   transform_points)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from tf2_ros import (Buffer, TransformBroadcaster, TransformException,
                     TransformListener)

# RELIABLE for megabyte-class messages, same reasoning as piros2_vision's
# edge detector: BEST_EFFORT delivers zero frames once a message fragments
# past the socket buffer (measured — docs/info/troubleshooting.md), and a
# BEST_EFFORT publisher is invisible to the RELIABLE-by-default viewers.
# Depth 1 = always the freshest frame, never a backlog.
BIG_FRAME_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)


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
            cap=self.get_parameter('keyframe_cap').value)
        self.lost_frames = 0
        self.needs_relocalization = False
        self.retry_countdown = 0
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
        response.success = True
        response.message = 'orientation reset to identity, keyframes cleared'
        return response

    def on_depth(self, msg: Image):
        self.depth_image = np.frombuffer(msg.data, np.float32).reshape(
            msg.height, msg.width)
        # Receipt clock, never header.stamp — the 0.73 s stamp fault.
        self.depth_received_at = self.get_clock().now()

    def on_frame(self, msg: CompressedImage):
        entry = self.get_clock().now()

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
        elif could_estimate:
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
        elif rotation_ok:
            self.maybe_store_keyframe(points, descriptors)

    def maybe_store_keyframe(self, points, descriptors):
        """Offer the frame to the store; the novelty gate decides."""
        if descriptors is None or len(points) == 0:
            return
        if self.rgbd_mode:
            # Geometry and pose come from the odometry's own authority:
            # a latest-only TF lookup and per-keypoint depth. No fresh
            # depth or no odom yet → no keyframe, no fallback guessing.
            transform = self._lookup('odom', 'camera_optical_frame')
            if transform is None:
                return
            pts_cam, valid = self._depth_points(points)
            min_pairs = self.get_parameter('relocalize_min_pairs').value
            if pts_cam is None or valid.sum() < 2 * min_pairs:
                return
            slot = self.store.maybe_add(
                descriptors[valid], transform[:3, 2],
                points=transform_points(transform, pts_cam[valid]),
                pose=transform)
            view_dir = transform[:3, 2]
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
        self.get_logger().info(
            f'relocalized against keyframe {best}: snapping odometry '
            f'(Δ {delta_m:.2f} m, {delta_deg:.1f}°)')
        return True

    def _depth_points(self, pixels):
        """Nx2 pixels → (Nx3 optical-frame points, valid mask), or None."""
        max_age = self.get_parameter('depth_max_age').value
        if (self.depth_image is None
                or (self.get_clock().now()
                    - self.depth_received_at).nanoseconds > max_age * 1e9):
            return None, None
        height, width = self.depth_image.shape
        u = np.clip(pixels[:, 0].astype(int), 0, width - 1)
        v = np.clip(pixels[:, 1].astype(int), 0, height - 1)
        z = self.depth_image[v, u].astype(np.float64)
        valid = (z > 0.05) & (z < 20.0)
        fx, fy = self.k_matrix[0, 0], self.k_matrix[1, 1]
        cx, cy = self.k_matrix[0, 2], self.k_matrix[1, 2]
        return np.column_stack([(pixels[:, 0] - cx) * z / fx,
                                (pixels[:, 1] - cy) * z / fy, z]), valid

    def _lookup(self, target, source):
        """Latest-only TF lookup as a 4x4, or None while it doesn't exist."""
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
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
