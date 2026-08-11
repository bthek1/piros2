"""
Shared SE(3) pure functions — the repo's transform conventions, named.

World fusion plan P0. This module exists to make one convention explicit
everywhere instead of implicit in two nodes:

- A rotation matrix R_ab maps *b-frame vectors into a-frame*; a 4x4
  homogeneous transform T_ab does the same for points (rotate, then
  translate by frame b's origin expressed in a). Read the subscripts
  right-to-left and composition is mechanical: T_ac = T_ab @ T_bc, the
  inner frames cancelling. tf2's lookup_transform('a', 'b', ...) returns
  exactly T_ab — the repo had been using the convention all along; now
  the argument names say so.
- The inverse never needs a matrix solve: T_ba = [R.T | -R.T @ t]. That
  identity is worth internalising — half the bugs in reconstruction
  pipelines are inverted transforms (info.md), and writing T_wc in the
  variable name is the cheap defence.
- Quaternions ride ROS messages as (x, y, z, w); both conversions live
  here, side by side, because they are each other's inverse and used to
  live in different files (detector encoding, mapper decoding) where the
  pairing was invisible.

Pure numpy, no ROS imports — the tests exercise geometry, not a graph.
"""

import numpy as np

# The canonical optical rotation (the -90/0/-90 that camera.launch.py
# publishes statically): optical +z (forward) -> base +x, optical +x
# (right) -> base -y, optical +y (down) -> base -z. In this module's
# notation it is R_base_optical: it maps optical-frame vectors into
# base_link-frame vectors, and conjugating by it re-expresses an
# optical-axes rotation in base axes (a pan becomes yaw-about-z-up).
BASE_FROM_OPTICAL = np.array([[0., 0., 1.],
                              [-1., 0., 0.],
                              [0., -1., 0.]])


def quaternion_from_rotation(rotation):
    """Rotation matrix -> (x, y, z, w), the message field order."""
    r = rotation
    trace = np.trace(r)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        return (0.25 * s, (r[0, 1] + r[1, 0]) / s,
                (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    if r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        return ((r[0, 1] + r[1, 0]) / s, 0.25 * s,
                (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
    return ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s,
            0.25 * s, (r[1, 0] - r[0, 1]) / s)


def rotation_from_quaternion(x, y, z, w):
    """(x, y, z, w) -> rotation matrix; quaternion_from_rotation inverted."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def make_transform(r_ab, t_ab):
    """
    Assemble T_ab from R_ab and frame b's origin in a-frame coordinates.

    The 4x4 homogeneous form [[R, t], [0, 1]] — matrix multiplication
    then composes rotation and translation in one operation, which is
    the entire reason the representation exists.
    """
    t_mat = np.eye(4)
    t_mat[:3, :3] = r_ab
    t_mat[:3, 3] = t_ab
    return t_mat


def invert(t_ab):
    """
    T_ab -> T_ba without a matrix solve: [R.T | -R.T @ t].

    Follows from undoing the operations in reverse order: subtract the
    translation, then rotate back.
    """
    r_ba = t_ab[:3, :3].T
    t_ba = np.eye(4)
    t_ba[:3, :3] = r_ba
    t_ba[:3, 3] = -r_ba @ t_ab[:3, 3]
    return t_ba


def transform_points(t_ab, points_b):
    """
    Map Nx3 b-frame points into frame a: R_ab @ p + t, vectorised.

    Row-vector layout means the rotation applies as `points @ R.T` —
    transposing instead of looping is the numpy idiom for point clouds.
    """
    points_b = np.asarray(points_b)
    return points_b @ t_ab[:3, :3].T + t_ab[:3, 3]
