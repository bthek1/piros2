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
Unit tests for the shared SE(3) module — geometry only, no ROS graph.

Ground-truth rotations come from cv2.Rodrigues, an implementation
independent of everything under test. The quaternion round trip is
driven through all four branches of the conversion on purpose: the
trace>0 fast path never touches the other three, which only fire near
180-degree rotations — exactly where a converter bug would hide.
"""

import cv2
import numpy as np
from piros2_world_mesh.se3 import (
    BASE_FROM_OPTICAL,
    euler_from_rotation,
    invert,
    make_transform,
    quaternion_from_rotation,
    rigid_transform_3d,
    rotation_from_quaternion,
    transform_points,
)


def rotation_matrix(axis, angle_rad):
    """Ground-truth rotation via cv2.Rodrigues — an independent source."""
    return cv2.Rodrigues(np.asarray(axis, np.float64) * angle_rad)[0]


# --- quaternions --------------------------------------------------------

def test_quaternion_from_rotation():
    assert np.allclose(quaternion_from_rotation(np.eye(3)), [0, 0, 0, 1])
    quarter = quaternion_from_rotation(
        rotation_matrix([0., 0., 1.], np.pi / 2))
    assert np.allclose(quarter,
                       [0., 0., np.sin(np.pi / 4), np.cos(np.pi / 4)])


def test_quaternion_round_trip_all_branches():
    """One rotation per branch of the conversion, recovered exactly."""
    cases = [
        rotation_matrix([0.3, -0.5, 0.8], 0.9),   # trace > 0
        rotation_matrix([1., 0., 0.], np.pi),      # r00 dominant, w ~ 0
        rotation_matrix([0., 1., 0.], np.pi),      # r11 dominant
        rotation_matrix([0., 0., 1.], np.pi),      # r22 dominant
    ]
    for truth in cases:
        back = rotation_from_quaternion(*quaternion_from_rotation(truth))
        assert np.allclose(back, truth, atol=1e-12)


# --- homogeneous transforms ---------------------------------------------

def test_invert_undoes_a_transform():
    t_ab = make_transform(rotation_matrix([0.2, 0.9, -0.4], 1.1),
                          [1.0, -2.0, 0.5])
    assert np.allclose(invert(t_ab) @ t_ab, np.eye(4), atol=1e-12)
    assert np.allclose(t_ab @ invert(t_ab), np.eye(4), atol=1e-12)


def test_composition_reads_right_to_left():
    """
    T_ac = T_ab @ T_bc, worked by hand.

    Frame b sits at [1,0,0] in a, yawed +90 deg; frame c sits at [0,1,0]
    in b. Frame c's origin in a: rotate [0,1,0] by +90 deg about z
    (-> [-1,0,0]) and add [1,0,0] -> the a-frame origin exactly.
    """
    t_ab = make_transform(rotation_matrix([0., 0., 1.], np.pi / 2),
                          [1., 0., 0.])
    t_bc = make_transform(np.eye(3), [0., 1., 0.])
    origin_of_c_in_a = transform_points(t_ab @ t_bc, [[0., 0., 0.]])
    assert np.allclose(origin_of_c_in_a, [[0., 0., 0.]], atol=1e-12)


def test_transform_points_rotates_then_translates():
    t_ab = make_transform(rotation_matrix([0., 0., 1.], np.pi / 2),
                          [10., 0., 0.])
    moved = transform_points(t_ab, [[1., 0., 0.], [0., 2., 0.]])
    assert np.allclose(moved, [[10., 1., 0.], [8., 0., 0.]], atol=1e-12)


# --- the optical-frame convention ---------------------------------------

def test_optical_axes_map_to_base_axes():
    """Optical +z (forward) -> base +x; +x -> -y; +y (down) -> -z."""
    assert np.allclose(BASE_FROM_OPTICAL @ [0., 0., 1.], [1., 0., 0.])
    assert np.allclose(BASE_FROM_OPTICAL @ [1., 0., 0.], [0., -1., 0.])
    assert np.allclose(BASE_FROM_OPTICAL @ [0., 1., 0.], [0., 0., -1.])
    assert np.isclose(np.linalg.det(BASE_FROM_OPTICAL), 1.0)


def test_optical_pan_conjugates_to_base_yaw():
    """A pan about optical y (down) must become base yaw about z (up)."""
    pan = rotation_matrix([0., 1., 0.], np.deg2rad(30))
    base = BASE_FROM_OPTICAL @ pan @ BASE_FROM_OPTICAL.T
    assert np.allclose(base, rotation_matrix([0., 0., -1.], np.deg2rad(30)),
                       atol=1e-12)


# --- euler and rigid 3D fits (relocalization plan) ----------------------

def test_euler_round_trips_through_zyx_composition():
    roll, pitch, yaw = 0.3, -0.4, 1.2
    rotation = (rotation_matrix([0., 0., 1.], yaw)
                @ rotation_matrix([0., 1., 0.], pitch)
                @ rotation_matrix([1., 0., 0.], roll))
    assert np.allclose(euler_from_rotation(rotation), (roll, pitch, yaw))


def test_rigid_transform_recovers_a_known_pose():
    rng = np.random.default_rng(11)
    src = rng.uniform(-2.0, 2.0, size=(40, 3))
    rotation = rotation_matrix([0.3, -0.5, 0.8], 0.9)
    translation = np.array([1.0, -2.0, 0.5])
    dst = src @ rotation.T + translation
    fit = rigid_transform_3d(src, dst)
    assert fit is not None
    assert np.allclose(fit[0], rotation, atol=1e-9)
    assert np.allclose(fit[1], translation, atol=1e-9)


def test_rigid_transform_survives_noise_and_outliers():
    rng = np.random.default_rng(12)
    src = rng.uniform(-2.0, 2.0, size=(60, 3))
    rotation = rotation_matrix([0., 1., 0.], 0.4)
    translation = np.array([0.2, 0.0, -0.7])
    dst = src @ rotation.T + translation
    dst += rng.normal(0.0, 0.005, size=dst.shape)
    dst[:6] += 3.0  # a few gross outliers, as bad matches produce
    fit = rigid_transform_3d(src, dst)
    assert fit is not None
    # The reject-worst refits must shrug the outliers off.
    assert np.allclose(fit[0], rotation, atol=0.01)
    assert np.allclose(fit[1], translation, atol=0.02)


def test_rigid_transform_refuses_thin_or_inconsistent_input():
    rng = np.random.default_rng(13)
    src = rng.uniform(-1.0, 1.0, size=(5, 3))
    assert rigid_transform_3d(src, src) is None  # too few pairs
    src = rng.uniform(-1.0, 1.0, size=(40, 3))
    garbage = rng.uniform(-1.0, 1.0, size=(40, 3))
    assert rigid_transform_3d(src, garbage) is None  # no rigid fit exists
