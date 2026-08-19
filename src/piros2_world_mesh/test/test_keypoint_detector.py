# Copyright 2026 Benedict Thekkel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Unit test for the keypoint detector: no camera, no graph, no discovery.

Same technique as the piros2_vision test — a callback is just a method, and
capturing publishers keeps DDS out entirely. A chessboard pattern is the
synthetic input of choice for a corner detector: ORB cannot *not* find
features on one.
"""

import cv2
import numpy as np
from piros2_world_mesh.keyframe_store import Keyframe
from piros2_world_mesh.keypoint_detector import (
    estimate_rotation,
    kabsch,
    keyframe_marker,
    KeypointDetector,
    rays_from_pixels,
)
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import Trigger


class CapturingPublisher:
    """Stands in for rclpy's Publisher; keeps what the node publishes."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class CapturingBroadcaster:
    """Stands in for tf2's TransformBroadcaster; keeps sent transforms."""

    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


@pytest.fixture
def node():
    rclpy.init()
    node = KeypointDetector()
    node.pub_image = CapturingPublisher()
    node.pub_count = CapturingPublisher()
    node.pub_matched = CapturingPublisher()
    node.pub_pose = CapturingPublisher()
    node.tf_broadcaster = CapturingBroadcaster()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def make_frame(squares=8, square_px=20) -> CompressedImage:
    """Build a JPEG chessboard frame — corners everywhere, by construction."""
    tile = np.zeros((2 * square_px, 2 * square_px), dtype=np.uint8)
    tile[:square_px, :square_px] = 255
    tile[square_px:, square_px:] = 255
    board = np.tile(tile, (squares // 2, squares // 2))
    img = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = cv2.imencode('.jpg', img)[1].tobytes()
    msg.header.frame_id = 'camera_optical_frame'
    msg.header.stamp.sec = 12345
    return msg


def make_textured_frame(seed=7, tweak=0) -> CompressedImage:
    """
    Build a JPEG of seeded noise — every patch unique, by construction.

    The chessboard is the right input for *detection* but the wrong one for
    *matching*: its corners are deliberate lookalikes, and the matcher's
    cross-check throws lookalikes away. Matching tests need texture where
    each keypoint's neighbourhood is distinctive. Greyscale noise, like
    the chessboard, so the overlay colours remain unmistakable — colour
    noise would contain green-ish and yellow-ish pixels of its own.

    tweak nudges one pixel before encoding: visually the same frame, but
    different JPEG bytes — needed since the node skips byte-identical
    frames as usb_cam duplicates.
    """
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 256, (240, 320), dtype=np.uint8)
    if tweak:
        img[0, 0] = (int(img[0, 0]) + tweak) % 256
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = cv2.imencode(
        '.jpg', cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))[1].tobytes()
    return msg


def greenish(img):
    """Mask of pixels only the matched-keypoint overlay can produce."""
    return (img[:, :, 1].astype(int) - img[:, :, 2].astype(int)) > 100


def yellowish(img):
    """Mask of pixels only the new-keypoint overlay can produce."""
    blue = img[:, :, 0].astype(int)
    return ((img[:, :, 1].astype(int) - blue > 100)
            & (img[:, :, 2].astype(int) - blue > 100))


def decode(msg):
    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)


def test_keypoints_found_and_counted(node):
    node.on_frame(make_frame())

    assert len(node.pub_count.messages) == 1
    count = node.pub_count.messages[0].data
    assert count > 0, 'ORB found nothing on a chessboard'
    # The cap is a real bound, not a target — but the count must respect it.
    assert count <= node.get_parameter('max_features').value


def test_annotated_image_round_trips(node):
    frame = make_frame()
    node.on_frame(frame)

    assert len(node.pub_image.messages) == 1
    out = node.pub_image.messages[0]
    assert out.format == 'jpeg'
    # JPEG magic bytes — real encoded data, not an empty buffer.
    assert bytes(out.data[:2]) == b'\xff\xd8'

    original = cv2.imdecode(np.frombuffer(frame.data, np.uint8),
                            cv2.IMREAD_COLOR)
    annotated = cv2.imdecode(np.frombuffer(out.data, np.uint8),
                             cv2.IMREAD_COLOR)
    # Same geometry as the input, and visibly annotated: with no previous
    # frame every keypoint is new, so the overlay is pure yellow — a colour
    # a grey chessboard cannot produce even through JPEG artefacts.
    assert annotated.shape == original.shape
    assert yellowish(annotated).any(), 'no keypoint overlay in the output'


def test_count_matches_detection(node):
    """The published count is the detector's own answer for that frame."""
    frame = make_frame()
    node.on_frame(frame)

    img = cv2.imdecode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # detectAndCompute, like the node: compute drops keypoints whose
    # descriptor patch falls off the edge, so detect() would over-count.
    expected, _ = node.orb.detectAndCompute(gray, None)
    assert node.pub_count.messages[0].data == len(expected)


def test_header_is_preserved(node):
    """Downstream consumers need the frame's identity, not our publish time."""
    node.on_frame(make_frame())
    out = node.pub_image.messages[0]
    assert out.header.frame_id == 'camera_optical_frame'
    assert out.header.stamp.sec == 12345


def test_first_frame_has_nothing_to_match(node):
    """No previous frame means zero matches and an all-yellow overlay."""
    node.on_frame(make_textured_frame())

    assert node.pub_matched.messages[0].data == 0
    annotated = decode(node.pub_image.messages[0])
    assert yellowish(annotated).any()
    assert not greenish(annotated).any()


def test_near_identical_frame_matches_and_draws_green(node):
    """A repeated scene re-observes its keypoints: matched, drawn green."""
    node.on_frame(make_textured_frame())
    # Byte-identical would be skipped as a usb_cam duplicate — one nudged
    # pixel makes it a distinct frame of the same scene.
    node.on_frame(make_textured_frame(tweak=1))

    total = node.pub_count.messages[1].data
    matched = node.pub_matched.messages[1].data
    # Near-identical descriptors match at tiny Hamming distance; on
    # distinctive texture the cross-check keeps nearly all of them.
    assert matched > total * 0.8
    assert greenish(decode(node.pub_image.messages[1])).any()


def test_window_recovers_a_feature_that_skipped_a_frame(node):
    """
    A scene that skips a frame is still matched on return.

    The reason the window exists: strict frame-to-frame matching would
    find nothing here (the middle frame shares no texture), but the pool
    still holds the first frame's descriptors when its scene returns.
    """
    scene = make_textured_frame(seed=7)
    node.on_frame(scene)
    node.on_frame(make_textured_frame(seed=8))
    node.on_frame(scene)

    total = node.pub_count.messages[2].data
    matched = node.pub_matched.messages[2].data
    assert matched > total * 0.8


def test_unrelated_frame_matches_little(node):
    """Different texture = different fingerprints: matching collapses."""
    node.on_frame(make_textured_frame(seed=7))
    node.on_frame(make_textured_frame(seed=8))

    total = node.pub_count.messages[1].data
    matched = node.pub_matched.messages[1].data
    assert matched < total * 0.2


def test_undecodable_frame_is_skipped(node):
    bad = CompressedImage()
    bad.format = 'jpeg'
    bad.data = b'not a jpeg'
    node.on_frame(bad)

    assert node.pub_image.messages == []
    assert node.pub_count.messages == []
    assert node.pub_matched.messages == []


# --- rotation geometry (world 3D plan P0): pure functions, no ROS -------

def unit_rays(n=60, seed=3):
    """Build a seeded bundle of unit rays, all in front of the camera."""
    rng = np.random.default_rng(seed)
    rays = rng.normal(size=(n, 3))
    rays[:, 2] = np.abs(rays[:, 2]) + 1.0
    return rays / np.linalg.norm(rays, axis=1, keepdims=True)


def rotation_matrix(axis, angle_rad):
    """Ground-truth rotation via cv2.Rodrigues — an independent source."""
    return cv2.Rodrigues(np.asarray(axis, np.float64) * angle_rad)[0]


def test_rays_from_pixels_geometry():
    k = np.array([[900., 0., 320.], [0., 900., 240.], [0., 0., 1.]])
    rays = rays_from_pixels(np.array([[320., 240.], [1220., 240.]]), k)
    # Principal point looks straight ahead; 900 px right of centre at
    # fx=900 is exactly 45 degrees off-axis.
    assert np.allclose(rays[0], [0., 0., 1.])
    assert np.allclose(rays[1], [np.sqrt(0.5), 0., np.sqrt(0.5)])


def test_estimator_recovers_a_known_rotation():
    prev = unit_rays()
    truth = rotation_matrix([0., 1., 0.], np.deg2rad(5))
    estimate = estimate_rotation(prev, prev @ truth.T)
    assert estimate is not None
    assert np.allclose(estimate, truth, atol=1e-9)


def test_estimator_survives_false_matches():
    """10% garbage pairs must land in the residual tail and get dropped."""
    prev = unit_rays(80)
    truth = rotation_matrix([1., 0., 0.], np.deg2rad(4))
    curr = prev @ truth.T
    curr[:8] = unit_rays(8, seed=9)
    estimate = estimate_rotation(prev, curr)
    assert estimate is not None
    error = np.arccos(np.clip((np.trace(estimate @ truth.T) - 1) / 2,
                              -1.0, 1.0))
    assert error < np.deg2rad(0.5)


def test_estimator_refuses_thin_or_inconsistent_input():
    """None on bad input is the contract — never a confident-looking R."""
    rays = unit_rays(20)
    assert estimate_rotation(rays[:5], rays[:5]) is None
    assert estimate_rotation(rays, unit_rays(20, seed=11)) is None


def test_kabsch_reflection_guard_on_coplanar_rays():
    """Coplanar bundles are where raw SVD can return a det=-1 reflection."""
    rng = np.random.default_rng(5)
    prev = np.column_stack([rng.normal(size=40), np.zeros(40), np.ones(40)])
    prev /= np.linalg.norm(prev, axis=1, keepdims=True)
    truth = rotation_matrix([0., 0., 1.], np.deg2rad(10))
    estimate = kabsch(prev, prev @ truth.T)
    assert np.isclose(np.linalg.det(estimate), 1.0)
    assert np.allclose(prev @ truth.T, prev @ estimate.T, atol=1e-9)


# --- the node's P0 behaviour --------------------------------------------

def make_camera_info(fx=900.0, cx=320.0, cy=240.0) -> CameraInfo:
    info = CameraInfo()
    info.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
    return info


def make_scene(seed=21, size=(480, 640)):
    """Band-limited texture: blurred noise survives warping, raw doesn't."""
    rng = np.random.default_rng(seed)
    noise = rng.integers(0, 256, size, dtype=np.uint8)
    smooth = cv2.GaussianBlur(noise, (0, 0), 3)
    return cv2.normalize(smooth, None, 0, 255, cv2.NORM_MINMAX)


def encode(img) -> CompressedImage:
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = cv2.imencode('.jpg', img)[1].tobytes()
    return msg


def test_duplicate_frame_is_skipped_whole(node):
    """Byte-identical means usb_cam's re-publish: no detection, no pose."""
    frame = make_textured_frame()
    node.on_frame(frame)
    node.on_frame(frame)

    assert node.dup_skipped == 1
    assert len(node.pub_count.messages) == 1
    assert len(node.pub_image.messages) == 1


def test_no_intrinsics_means_no_pose(node):
    node.on_frame(make_textured_frame())
    node.on_frame(make_textured_frame(tweak=1))
    assert node.pub_pose.messages == []


def test_zero_k_camera_info_is_rejected(node):
    """The milestone-6 bag trap: camera_info present, K all zeros."""
    info = make_camera_info()
    info.k = [0.0] * 9
    node.on_camera_info(info)
    assert node.k_matrix is None


def test_in_plane_rotation_is_estimated(node):
    """
    End-to-end wiring check with known geometry.

    Rotating the image about the principal point is a pure roll about
    the optical axis — which the conjugation must turn into base_link
    roll about x. The estimated angle must match the warp's.
    """
    node.on_camera_info(make_camera_info())
    scene = make_scene()
    node.on_frame(encode(scene))
    warp = cv2.getRotationMatrix2D((320.0, 240.0), 4.0, 1.0)
    node.on_frame(encode(cv2.warpAffine(scene, warp, (640, 480))))

    assert len(node.pub_pose.messages) == 2
    first = node.pub_pose.messages[0].pose.orientation
    assert np.isclose(first.w, 1.0)   # starts at identity
    q = node.pub_pose.messages[1].pose.orientation
    angle = 2 * np.arccos(np.clip(abs(q.w), -1.0, 1.0))
    assert np.deg2rad(3) < angle < np.deg2rad(5)
    axis = np.array([q.x, q.y, q.z])
    assert abs(axis[0]) > 0.9 * np.linalg.norm(axis)
    assert node.pub_pose.messages[1].header.frame_id == 'odom'


def test_orientation_is_also_broadcast_as_tf(node):
    """The odom → base_link TF mirrors the pose: one estimate, twice."""
    node.on_camera_info(make_camera_info())
    scene = make_scene()
    node.on_frame(encode(scene))
    warp = cv2.getRotationMatrix2D((320.0, 240.0), 4.0, 1.0)
    node.on_frame(encode(cv2.warpAffine(scene, warp, (640, 480))))

    assert len(node.tf_broadcaster.transforms) == 2
    tf = node.tf_broadcaster.transforms[1]
    assert tf.header.frame_id == 'odom'
    assert tf.child_frame_id == 'base_link'
    # Rotation-only odometry: translation must stay exactly zero.
    assert (tf.transform.translation.x, tf.transform.translation.y,
            tf.transform.translation.z) == (0.0, 0.0, 0.0)
    assert tf.transform.rotation == node.pub_pose.messages[1].pose.orientation


def test_publish_tf_false_yields_the_frame(node):
    """
    publish_tf: false silences TF.

    REP-105: with rgbd odometry owning odom → base_link, the compass
    must stop broadcasting — one parent per frame. The fixture built
    this node in kp mode, so its pose publisher exists and still
    fires; whether the topic is advertised at all is decided once, at
    construction — test_rgbd_mode_advertises_no_orientation_topic.
    """
    node.set_parameters([rclpy.parameter.Parameter('publish_tf',
                                                   value=False)])
    node.on_camera_info(make_camera_info())
    scene = make_scene()
    node.on_frame(encode(scene))
    warp = cv2.getRotationMatrix2D((320.0, 240.0), 4.0, 1.0)
    node.on_frame(encode(cv2.warpAffine(scene, warp, (640, 480))))

    assert node.tf_broadcaster.transforms == []
    assert len(node.pub_pose.messages) == 2


def test_rgbd_mode_advertises_no_orientation_topic():
    """
    No /camera/orientation topic is advertised in rgbd mode.

    Nothing in the session subscribed it, and a second "orientation in
    odom" published while RTAB-Map owns that frame would contradict its
    real owner. No publisher means no topic in the graph — a node built
    for rgbd mode has none. (Constructed through global CLI overrides:
    the parameter is read once, so it must be set before __init__.)
    """
    rclpy.init(args=['--ros-args', '-p', 'publish_tf:=false'])
    try:
        node = KeypointDetector()
        assert node.rgbd_mode
        assert node.pub_pose is None
        # The frame still estimates — it just keeps the answer to
        # itself and to the graph.
        node.on_camera_info(make_camera_info())
        scene = make_scene()
        node.on_frame(encode(scene))
        warp = cv2.getRotationMatrix2D((320.0, 240.0), 4.0, 1.0)
        node.on_frame(encode(cv2.warpAffine(scene, warp, (640, 480))))
        assert not np.allclose(node.orientation, np.eye(3))
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_reset_service_rezeros_orientation(node):
    node.orientation = rotation_matrix([0., 0., 1.], 1.0)
    response = node.on_reset(None, Trigger.Response())
    assert response.success
    assert np.allclose(node.orientation, np.eye(3))


# --- relocalization (relocalization plan) ---------------------------------

K_TEST = np.array([[500.0, 0.0, 320.0],
                   [0.0, 500.0, 240.0],
                   [0.0, 0.0, 1.0]])


def synthetic_view(orientation, seed=3, count=60):
    """
    Fake "the camera looks at the same wall again", pure geometry.

    Returns descriptors, the pixels a fixed landmark field projects to
    at the given orientation (R_odom_cam), and the landmark directions
    themselves (odom frame).
    """
    rng = np.random.default_rng(seed)
    descriptors = rng.integers(0, 256, size=(count, 32), dtype=np.uint8)
    spread = rng.uniform(-0.25, 0.25, size=(count, 2))
    dirs = np.column_stack([spread, np.ones(count)])
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    rays_cam = dirs @ orientation  # R.T @ d, row layout
    pixels = np.column_stack([
        K_TEST[0, 0] * rays_cam[:, 0] / rays_cam[:, 2] + K_TEST[0, 2],
        K_TEST[1, 1] * rays_cam[:, 1] / rays_cam[:, 2] + K_TEST[1, 2]])
    return descriptors, pixels, dirs


def rotation_about_y(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), 0.0, np.sin(a)],
                     [0.0, 1.0, 0.0],
                     [-np.sin(a), 0.0, np.cos(a)]])


def test_keyframe_capture_stores_rays_in_odom(node):
    node.k_matrix = K_TEST
    true_orientation = rotation_about_y(20.0)
    node.orientation = true_orientation
    descriptors, pixels, dirs = synthetic_view(true_orientation)
    node.maybe_store_keyframe(pixels, descriptors)

    assert len(node.store) == 1
    # Stored rays must be the landmark directions in odom — independent
    # of the orientation the camera happened to have at capture.
    assert np.allclose(node.store.keyframes[0].rays, dirs, atol=1e-9)


def test_relocalization_recovers_a_corrupted_orientation(node):
    node.k_matrix = K_TEST
    true_orientation = rotation_about_y(20.0)
    node.orientation = true_orientation
    descriptors, pixels, _ = synthetic_view(true_orientation)
    node.maybe_store_keyframe(pixels, descriptors)

    # The flick: composition lost the motion, the compass is wrong.
    node.orientation = np.eye(3)
    assert node.attempt_relocalization(pixels, descriptors)
    assert node._angle_between_deg(
        node.orientation, true_orientation) < 0.01


def test_relocalization_refuses_an_unknown_view(node):
    node.k_matrix = K_TEST
    node.orientation = np.eye(3)
    descriptors, pixels, _ = synthetic_view(np.eye(3), seed=3)
    node.maybe_store_keyframe(pixels, descriptors)

    stranger_desc, stranger_px, _ = synthetic_view(np.eye(3), seed=99)
    before = node.orientation.copy()
    assert not node.attempt_relocalization(stranger_px, stranger_desc)
    assert np.allclose(node.orientation, before)


def test_lost_tracking_arms_relocalization(node):
    node.k_matrix = K_TEST
    descriptors, pixels, _ = synthetic_view(np.eye(3))
    for _ in range(node.get_parameter('relocalize_after').value):
        node.track_room_memory(rotation_ok=False, could_estimate=True,
                               points=pixels, descriptors=descriptors)
    assert node.needs_relocalization


def test_blackout_after_tracking_counts_as_lost(node):
    # A featureless view (lens covered, black frames) yields no
    # descriptors: could_estimate is False, yet it is a loss if we were
    # tracking before — the black-fill gate bag proved the counter
    # otherwise never moved and the pose came back wrong by 19°.
    node.k_matrix = K_TEST
    descriptors, pixels, _ = synthetic_view(np.eye(3))
    node.track_room_memory(rotation_ok=True, could_estimate=True,
                           points=pixels, descriptors=descriptors)
    for _ in range(node.get_parameter('relocalize_after').value):
        node.track_room_memory(rotation_ok=False, could_estimate=False,
                               points=np.zeros((0, 2)), descriptors=None)
    assert node.needs_relocalization


def test_blackout_before_any_tracking_is_not_a_loss(node):
    # Startup in the dark is "nothing to track yet", not a loss.
    node.k_matrix = K_TEST
    for _ in range(node.get_parameter('relocalize_after').value + 1):
        node.track_room_memory(rotation_ok=False, could_estimate=False,
                               points=np.zeros((0, 2)), descriptors=None)
    assert not node.needs_relocalization


def test_reset_clears_the_room_memory(node):
    node.k_matrix = K_TEST
    descriptors, pixels, _ = synthetic_view(np.eye(3))
    node.orientation = np.eye(3)
    node.maybe_store_keyframe(pixels, descriptors)
    assert len(node.store) == 1

    node.on_reset(Trigger.Request(), Trigger.Response())
    assert len(node.store) == 0


def test_save_map_writes_npz_that_reloads(node, tmp_path):
    node.k_matrix = K_TEST
    node.orientation = np.eye(3)
    descriptors, pixels, _ = synthetic_view(np.eye(3))
    node.maybe_store_keyframe(pixels, descriptors)
    node.set_parameters([rclpy.parameter.Parameter(
        'map_dir', rclpy.parameter.Parameter.Type.STRING, str(tmp_path))])

    response = node.on_save_map(Trigger.Request(), Trigger.Response())
    assert response.success
    saved = list(tmp_path.glob('room_*.npz'))
    assert len(saved) == 1
    from piros2_world_mesh.keyframe_store import KeyframeStore
    assert len(KeyframeStore.load(saved[0])) == 1


def test_save_map_refuses_an_empty_store(node, tmp_path):
    node.set_parameters([rclpy.parameter.Parameter(
        'map_dir', rclpy.parameter.Parameter.Type.STRING, str(tmp_path))])
    response = node.on_save_map(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert list(tmp_path.glob('*.npz')) == []


# --- keyframe marker (P4) ---------------------------------------------------

def test_keyframe_marker_draws_one_stroke_per_keyframe():
    from builtin_interfaces.msg import Time as TimeMsg
    pose = np.eye(4)
    pose[:3, 3] = [1.0, 2.0, 0.5]
    keyframes = [
        Keyframe(descriptors=np.zeros((1, 32), np.uint8),
                 view_dir=np.array([0.0, 0.0, 1.0]), pose=pose),
        Keyframe(descriptors=np.zeros((1, 32), np.uint8),
                 view_dir=np.array([0.0, 0.0, 1.0])),  # kp mode: no pose
    ]
    marker = keyframe_marker(keyframes, TimeMsg(), axis_length=0.5)
    assert marker.header.frame_id == 'odom'
    assert len(marker.points) == 4  # LINE_LIST: two points per stroke
    # rgbd keyframe: stroke starts at the stored position, points along
    # the pose's optical-z axis (identity pose → +z).
    assert (marker.points[0].x, marker.points[0].y,
            marker.points[0].z) == (1.0, 2.0, 0.5)
    assert np.isclose(marker.points[1].z, 1.0)
    # kp keyframe: from the origin, optical forward → base +x.
    assert (marker.points[2].x, marker.points[2].y,
            marker.points[2].z) == (0.0, 0.0, 0.0)
    assert np.isclose(marker.points[3].x, 0.5)


def test_keyframe_marker_deletes_when_store_is_empty():
    from builtin_interfaces.msg import Time as TimeMsg
    marker = keyframe_marker([], TimeMsg())
    assert marker.action == marker.DELETE
    assert marker.points == []


# ---------------------------------------------------------------- SLAM P1

def test_pnp_recovers_a_known_camera_pose():
    from piros2_world_mesh.keypoint_detector import pnp_pose
    from piros2_world_mesh.se3 import invert as inv, make_transform as mk
    rng = np.random.default_rng(5)
    k = np.array([[900.0, 0.0, 320.0], [0.0, 900.0, 240.0], [0.0, 0.0, 1.0]])
    # A cloud of landmarks 1-3 m ahead of a camera posed in "world".
    r_wc = rotation_matrix(np.array([0.0, 1.0, 0.0]), np.radians(12.0))
    t_wc = np.array([0.3, -0.1, 0.05])
    t_world_cam = mk(r_wc, t_wc)
    pts_cam = np.column_stack([rng.uniform(-1, 1, 80), rng.uniform(-0.7, 0.7, 80),
                               rng.uniform(1.0, 3.0, 80)])
    pts_world = pts_cam @ r_wc.T + t_wc
    proj = pts_cam @ k.T
    pixels = proj[:, :2] / proj[:, 2:3] + rng.normal(0, 0.3, (80, 2))
    # A few wrong correspondences — RANSAC's job.
    pixels[:6] = rng.uniform(0, 600, (6, 2))
    result = pnp_pose(pts_world, pixels, k)
    assert result is not None
    pose, inliers = result
    assert inliers >= 60
    err = inv(t_world_cam) @ pose
    assert np.linalg.norm(err[:3, 3]) < 0.01
    assert np.degrees(np.arccos((np.trace(err[:3, :3]) - 1) / 2)) < 0.3


def test_pnp_refuses_thin_input():
    from piros2_world_mesh.keypoint_detector import pnp_pose
    k = np.array([[900.0, 0.0, 320.0], [0.0, 900.0, 240.0], [0.0, 0.0, 1.0]])
    assert pnp_pose(np.zeros((3, 3)), np.zeros((3, 2)), k) is None


def test_graph_marker_colours_loop_edges():
    from builtin_interfaces.msg import Time as TimeMsg
    from piros2_world_mesh.keypoint_detector import graph_marker
    from piros2_world_mesh.pose_graph import PoseGraph
    graph = PoseGraph()
    for x in range(4):
        pose = np.eye(4)
        pose[0, 3] = float(x)
        graph.add_node(pose)
    for i in range(3):
        graph.add_odometry_edge(i, i + 1)
    graph.add_edge(3, 0, np.eye(4), kind='loop')
    marker = graph_marker(graph.poses, graph.edges, TimeMsg(), 'map')
    assert marker.header.frame_id == 'map'
    assert len(marker.points) == 8 and len(marker.colors) == 8
    # Odom edges grey, the loop edge magenta (last two vertices).
    assert marker.colors[0].r == marker.colors[0].g == marker.colors[0].b
    assert marker.colors[-1].r == 1.0 and marker.colors[-1].g < 0.5
    empty = graph_marker([], [], TimeMsg(), 'map')
    assert empty.action == empty.DELETE


# ---------------------------------------------------------------- SLAM P4

def test_save_map_carries_the_graph_and_load_restores_it(node, tmp_path):
    from piros2_world_mesh.pose_graph import information_matrix
    # Two keyframes with a graph: two nodes, an odom edge, a loop edge,
    # a non-identity correction.
    rng = np.random.default_rng(1)
    node.store.maybe_add(rng.integers(0, 256, (40, 32), dtype=np.uint8),
                         np.array([0.0, 0.0, 1.0]))
    node.store.maybe_add(rng.integers(0, 256, (40, 32), dtype=np.uint8),
                         np.array([0.0, 1.0, 0.0]))
    p0, p1 = np.eye(4), np.eye(4)
    p1[0, 3] = 0.5
    node.graph.add_node(p0)
    node.graph.add_node(p1)
    node.node_odom = [np.eye(4), p1 @ np.diag([1.0, 1.0, 1.0, 1.0])]
    node.node_stamp = [12.5, 13.5]
    node.node_wall = [0.0, 1.0]
    node.graph.add_odometry_edge(0, 1, information_matrix(0.02, 0.02))
    node.graph.add_edge(1, 0, np.eye(4), information_matrix(0.03, 0.03),
                        kind='loop')
    node.loop_edges = [(1, 0)]
    node.map_odom = np.eye(4)
    node.map_odom[1, 3] = 0.07
    node.store.keyframes[0].node_id = 0
    node.store.keyframes[1].node_id = 1
    path = tmp_path / 'room.npz'
    node.save_map(str(path))

    node.graph = type(node.graph)()
    node.node_odom, node.node_stamp = [], []
    node.map_odom = np.eye(4)
    node.load_map(str(path))
    assert len(node.store) == 2 and len(node.graph) == 2
    assert [kf.node_id for kf in node.store.keyframes] == [0, 1]
    assert np.allclose(node.graph.poses[1], p1)
    assert node.node_stamp == [12.5, 13.5]
    assert len(node.graph.edges) == 2
    assert node.graph.edges[1].kind == 'loop' and node.loop_edges == [(1, 0)]
    assert np.isclose(node.map_odom[1, 3], 0.07)
    # Loaded nodes are eligible for loop queries straight away.
    assert node.node_wall == [0.0, 0.0]
