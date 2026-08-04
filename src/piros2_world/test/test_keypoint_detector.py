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
from piros2_world.keypoint_detector import (
    BASE_FROM_OPTICAL,
    estimate_rotation,
    kabsch,
    KeypointDetector,
    quaternion_from_rotation,
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


@pytest.fixture
def node():
    rclpy.init()
    node = KeypointDetector()
    node.pub_image = CapturingPublisher()
    node.pub_count = CapturingPublisher()
    node.pub_matched = CapturingPublisher()
    node.pub_pose = CapturingPublisher()
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


def test_quaternion_from_rotation():
    assert np.allclose(quaternion_from_rotation(np.eye(3)), [0, 0, 0, 1])
    quarter = quaternion_from_rotation(
        rotation_matrix([0., 0., 1.], np.pi / 2))
    assert np.allclose(quarter,
                       [0., 0., np.sin(np.pi / 4), np.cos(np.pi / 4)])


def test_optical_pan_conjugates_to_base_yaw():
    """A pan about optical y (down) must become base yaw about z (up)."""
    pan = rotation_matrix([0., 1., 0.], np.deg2rad(30))
    base = BASE_FROM_OPTICAL @ pan @ BASE_FROM_OPTICAL.T
    assert np.allclose(base, rotation_matrix([0., 0., -1.], np.deg2rad(30)),
                       atol=1e-12)


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


def test_reset_service_rezeros_orientation(node):
    node.orientation = rotation_matrix([0., 0., 1.], 1.0)
    response = node.on_reset(None, Trigger.Response())
    assert response.success
    assert np.allclose(node.orientation, np.eye(3))
