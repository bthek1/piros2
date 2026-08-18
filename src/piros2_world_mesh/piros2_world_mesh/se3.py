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


def euler_from_rotation(rotation):
    """
    R -> (roll, pitch, yaw) with R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

    The ZYX convention every ROS rpy interface uses — including
    RTAB-Map's reset_odom_to_pose service, which is why this exists.
    """
    r = rotation
    pitch = -np.arcsin(np.clip(r[2, 0], -1.0, 1.0))
    roll = np.arctan2(r[2, 1], r[2, 2])
    yaw = np.arctan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


def rigid_transform_3d(src_points, dst_points, min_pairs=8,
                       max_residual_m=0.08, refit_rounds=2,
                       drop_fraction=0.2):
    """
    Best-fit (R, t) with dst ≈ R @ src + t, or None if untrustworthy.

    Umeyama without the scale term — the 3D sibling of the detector's
    Kabsch-on-rays: subtract centroids, one SVD on the covariance, and
    the translation falls out of the rotated centroids. The same
    reject-worst-and-refit rounds stand in for RANSAC (descriptor
    matches carry a few percent of false pairs), and the same
    fail-loudly rule applies: a thin or inconsistent pair set returns
    None, never a confident-looking guess.
    """
    src = np.asarray(src_points, dtype=np.float64)
    dst = np.asarray(dst_points, dtype=np.float64)
    if len(src) < min_pairs:
        return None

    def fit(s, d):
        cs, cd = s.mean(axis=0), d.mean(axis=0)
        u, _, vt = np.linalg.svd((s - cs).T @ (d - cd))
        rot = vt.T @ u.T
        if np.linalg.det(rot) < 0:
            vt[-1] *= -1
            rot = vt.T @ u.T
        return rot, cd - rot @ cs

    rot, t = fit(src, dst)
    for _ in range(refit_rounds):
        residuals = np.linalg.norm(src @ rot.T + t - dst, axis=1)
        keep_count = int(np.ceil(len(src) * (1.0 - drop_fraction)))
        keep = np.argsort(residuals)[:keep_count]
        src, dst = src[keep], dst[keep]
        rot, t = fit(src, dst)
    residuals = np.linalg.norm(src @ rot.T + t - dst, axis=1)
    if len(src) < min_pairs or float(residuals.mean()) > max_residual_m:
        return None
    return rot, t


# ----------------------------------------------------------------------
# Lie group / algebra (SLAM plan P2): the pose-graph optimiser lives on
# the manifold. A rigid motion has 6 degrees of freedom but 12 numbers
# in its 4x4 form; the twist (rho, phi) — 3 of translation-ish, 3 of
# rotation — is the minimal local coordinate, and exp/log convert
# between "a small motion as 6 numbers" and "a transform". Optimising in
# twist space is what lets a Gauss-Newton step be an ordinary 6-vector
# and the update T <- T @ exp(delta) stay a valid rotation for free.
# Twist order [rho, phi] = [translation, rotation], the g2o/Sophus
# convention.

def hat(phi):
    """Return the skew-symmetric matrix of a 3-vector (so it acts as a cross product)."""
    x, y, z = phi
    return np.array([[0.0, -z, y],
                     [z, 0.0, -x],
                     [-y, x, 0.0]])


def so3_exp(phi):
    """Rotation vector (axis * angle) -> rotation matrix (Rodrigues)."""
    phi = np.asarray(phi, dtype=np.float64)
    angle = np.linalg.norm(phi)
    k = hat(phi)
    if angle < 1e-9:
        return np.eye(3) + k
    return (np.eye(3) + np.sin(angle) / angle * k
            + (1.0 - np.cos(angle)) / angle ** 2 * (k @ k))


def so3_log(rotation):
    """Rotation matrix -> rotation vector; so3_exp inverted."""
    r = np.asarray(rotation, dtype=np.float64)
    cos = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos)
    if angle < 1e-9:
        return np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0],
                         r[1, 0] - r[0, 1]]) / 2.0
    if np.pi - angle < 1e-6:
        # Near pi the sine formula degenerates; read the axis off the
        # symmetric part instead.
        sym = (r + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(sym), 0.0, None))
        # Fix signs from the off-diagonals.
        i = int(np.argmax(axis))
        for j in range(3):
            if j != i and axis[j] > 0 and sym[i, j] < 0:
                axis[j] = -axis[j]
        return axis / np.linalg.norm(axis) * angle
    return angle / (2.0 * np.sin(angle)) * np.array(
        [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


def _left_jacobian_so3(phi):
    angle = np.linalg.norm(phi)
    k = hat(phi)
    if angle < 1e-9:
        return np.eye(3) + 0.5 * k
    return (np.eye(3) + (1.0 - np.cos(angle)) / angle ** 2 * k
            + (angle - np.sin(angle)) / angle ** 3 * (k @ k))


def se3_exp(twist):
    """Twist [rho, phi] -> 4x4 transform."""
    twist = np.asarray(twist, dtype=np.float64)
    rho, phi = twist[:3], twist[3:]
    return make_transform(so3_exp(phi), _left_jacobian_so3(phi) @ rho)


def se3_log(t_mat):
    """4x4 transform -> twist [rho, phi]; se3_exp inverted."""
    phi = so3_log(t_mat[:3, :3])
    rho = np.linalg.solve(_left_jacobian_so3(phi), t_mat[:3, 3])
    return np.concatenate([rho, phi])


def adjoint(t_mat):
    """
    Return the adjoint of T, which moves a twist between frames: Ad(T) @ xi.

    T @ exp(xi) == exp(Ad(T) @ xi) @ T — the identity that lets a
    perturbation applied on one side of a transform be expressed on the
    other, which is exactly what the pose-graph Jacobians need.
    """
    r = t_mat[:3, :3]
    t = t_mat[:3, 3]
    ad = np.zeros((6, 6))
    ad[:3, :3] = r
    ad[3:, 3:] = r
    ad[:3, 3:] = hat(t) @ r
    return ad
