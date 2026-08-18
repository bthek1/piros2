"""
A pose graph and the Gauss-Newton that optimises it — the SLAM backend.

SLAM plan P2. The front-end (keypoint_detector) produces two kinds of
constraint between keyframe poses: *odometry edges* (consecutive
keyframes, from rgbd_odometry — locally accurate, drifting globally) and
*loop edges* (a recognised revisit, from a rigid fit of the current
frame's 3D landmarks against a stored keyframe's — globally anchoring,
occasionally wrong). Given both, the backend answers one question: what
set of poses is most consistent with *all* the constraints at once? That
is a nonlinear least-squares problem, and solving it is what turns
"odometry plus a place memory" into SLAM: a loop closure stops being a
snap of the present and becomes a correction spread over the past.

What is here, and why hand-written rather than g2o/GTSAM bindings:

- Nodes are SE(3) poses (4x4), edges carry a relative measurement
  Z_ij ≈ T_i⁻¹ T_j and a 6x6 information matrix. The residual is
  e = log(Z_ij⁻¹ T_i⁻¹ T_j) — a twist, six numbers, zero when the graph
  agrees with the edge.
- Optimisation is on the manifold: each Gauss-Newton step solves for a
  6-vector per node and applies it as T <- T exp(δ) (right
  perturbation), so poses never leave SE(3). The Jacobians follow from
  T_i exp(δ_i) → e ≈ e − J Ad(T_j⁻¹ T_i) δ_i + J δ_j with J = J_r⁻¹(e)
  ≈ I + ½ ad(e) — the standard first-order forms.
- A Huber kernel re-weights edges whose residual exceeds `huber` (in
  chi units), so one wrong loop closure bends the graph a little rather
  than breaking it — the robust-cost line of the syllabus, in ~10 lines.
- Dense linear algebra: a room is ≤ a few hundred nodes, so the 6N×6N
  system is at most ~1000², which numpy solves in milliseconds; sparse
  Cholesky is what g2o adds at city scale, and the .g2o read/write
  below lets the installed `g2o` binary check every answer this code
  gives (test_pose_graph.py).

Pure numpy, no ROS: the graph is data, the optimiser is a function.
"""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .se3 import (adjoint, hat, invert, make_transform,
                  quaternion_from_rotation, rotation_from_quaternion, se3_exp,
                  se3_log)


@dataclass
class Edge:
    i: int
    j: int
    measurement: np.ndarray            # 4x4, Z_ij ≈ T_i⁻¹ T_j
    information: np.ndarray            # 6x6
    kind: str = 'odom'                 # 'odom' | 'loop'


def _ad_algebra(xi):
    """ad(ξ) for ξ = [rho, phi]: the algebra adjoint (6x6)."""
    rho, phi = xi[:3], xi[3:]
    out = np.zeros((6, 6))
    out[:3, :3] = hat(phi)
    out[:3, 3:] = hat(rho)
    out[3:, 3:] = hat(phi)
    return out


def information_matrix(sigma_t, sigma_r):
    """
    Diagonal 6x6 information from translation and rotation sigmas.

    Standard deviations in metres and radians, [rho, phi] ordering.
    """
    return np.diag([1.0 / sigma_t ** 2] * 3 + [1.0 / sigma_r ** 2] * 3)


@dataclass
class PoseGraph:
    poses: list = field(default_factory=list)      # 4x4 each
    edges: list = field(default_factory=list)

    # ------------------------------------------------------------ build

    def add_node(self, pose):
        self.poses.append(np.array(pose, dtype=np.float64))
        return len(self.poses) - 1

    def add_edge(self, i, j, measurement, information=None, kind='odom'):
        if information is None:
            information = np.eye(6)
        self.edges.append(Edge(i, j, np.array(measurement, dtype=np.float64),
                               np.array(information, dtype=np.float64), kind))
        return len(self.edges) - 1

    def add_odometry_edge(self, i, j, information=None):
        """
        Edge whose measurement is the current relative pose T_i⁻¹ T_j.

        What a chain of odometry-initialised nodes starts with.
        """
        z = invert(self.poses[i]) @ self.poses[j]
        return self.add_edge(i, j, z, information, 'odom')

    def __len__(self):
        return len(self.poses)

    # ------------------------------------------------------------ cost

    def residual(self, edge):
        return se3_log(invert(edge.measurement)
                       @ invert(self.poses[edge.i]) @ self.poses[edge.j])

    def chi2(self, huber=None):
        total = 0.0
        for edge in self.edges:
            e = self.residual(edge)
            r2 = float(e @ edge.information @ e)
            if huber is not None and edge.kind == 'loop':
                r = np.sqrt(r2)
                if r > huber:
                    r2 = 2 * huber * r - huber ** 2
            total += r2
        return total

    # ------------------------------------------------------------ solve

    def optimize(self, fixed=(0,), max_iters=20, huber=None, tol=1e-6,
                 damping=1e-4):
        """
        Gauss-Newton (with a little Levenberg damping) on the manifold.

        `fixed` nodes are held (gauge freedom: a pose graph is defined
        up to one rigid motion, so something must pin it). `huber` — in
        chi units, i.e. residual measured by the information matrix —
        down-weights loop edges beyond it. Returns a stats dict: the
        iteration count, chi² before/after, and the largest node shift
        in metres and degrees, which is what a caller uses to decide
        whether the map moved enough to be worth re-integrating.
        """
        n = len(self.poses)
        if n == 0 or not self.edges:
            return {'iterations': 0, 'chi2_before': 0.0, 'chi2_after': 0.0,
                    'max_shift_m': 0.0, 'max_shift_deg': 0.0}
        before = [p.copy() for p in self.poses]
        chi2_before = self.chi2(huber)
        free = [k for k in range(n) if k not in set(fixed)]
        col = {k: c for c, k in enumerate(free)}
        m = 6 * len(free)
        iterations = 0
        lam = damping
        chi2_prev = chi2_before
        for iterations in range(1, max_iters + 1):
            h_mat = np.zeros((m, m))
            b_vec = np.zeros(m)
            for edge in self.edges:
                e = self.residual(edge)
                info = edge.information
                if huber is not None and edge.kind == 'loop':
                    r = float(np.sqrt(e @ info @ e))
                    if r > huber:
                        info = info * (huber / r)
                j_inv = np.eye(6) + 0.5 * _ad_algebra(e)
                a_ji = adjoint(invert(self.poses[edge.j]) @ self.poses[edge.i])
                jac_i = -j_inv @ a_ji
                jac_j = j_inv
                blocks = []
                if edge.i in col:
                    blocks.append((col[edge.i], jac_i))
                if edge.j in col:
                    blocks.append((col[edge.j], jac_j))
                for ca, ja in blocks:
                    ra = slice(6 * ca, 6 * ca + 6)
                    b_vec[ra] += ja.T @ info @ e
                    for cb, jb in blocks:
                        rb = slice(6 * cb, 6 * cb + 6)
                        h_mat[ra, rb] += ja.T @ info @ jb
            if m == 0:
                break
            h_damped = h_mat + lam * np.diag(np.diag(h_mat) + 1e-9)
            try:
                delta = np.linalg.solve(h_damped, -b_vec)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(h_damped, -b_vec, rcond=None)[0]
            trial = [p.copy() for p in self.poses]
            for k in free:
                d = delta[6 * col[k]:6 * col[k] + 6]
                trial[k] = self.poses[k] @ se3_exp(d)
            saved = self.poses
            self.poses = trial
            chi2_now = self.chi2(huber)
            if chi2_now > chi2_prev:
                # Step made it worse: back off, damp harder, retry.
                self.poses = saved
                lam *= 10.0
                if lam > 1e3:
                    break
                continue
            lam = max(lam / 3.0, 1e-9)
            step = float(np.max(np.abs(delta))) if len(delta) else 0.0
            converged = (chi2_prev - chi2_now) < tol * max(chi2_prev, 1e-12) \
                or step < 1e-8
            chi2_prev = chi2_now
            if converged:
                break
        shifts_m, shifts_deg = [0.0], [0.0]
        for a, b in zip(before, self.poses):
            shifts_m.append(float(np.linalg.norm(a[:3, 3] - b[:3, 3])))
            cos = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1.0) / 2.0
            shifts_deg.append(float(np.degrees(np.arccos(np.clip(cos, -1, 1)))))
        return {'iterations': iterations, 'chi2_before': chi2_before,
                'chi2_after': chi2_prev, 'max_shift_m': max(shifts_m),
                'max_shift_deg': max(shifts_deg)}

    # ------------------------------------------------------------ g2o I/O

    def to_g2o(self, path, fixed=(0,)):
        """Write VERTEX_SE3:QUAT / EDGE_SE3:QUAT lines g2o reads."""
        lines = []
        for k, pose in enumerate(self.poses):
            q = quaternion_from_rotation(pose[:3, :3])
            t = pose[:3, 3]
            lines.append('VERTEX_SE3:QUAT %d %.9f %.9f %.9f %.9f %.9f %.9f %.9f'
                         % (k, t[0], t[1], t[2], q[0], q[1], q[2], q[3]))
        for k in fixed:
            lines.append(f'FIX {k}')
        for edge in self.edges:
            q = quaternion_from_rotation(edge.measurement[:3, :3])
            t = edge.measurement[:3, 3]
            upper = [edge.information[r, c] for r in range(6)
                     for c in range(r, 6)]
            lines.append('EDGE_SE3:QUAT %d %d %.9f %.9f %.9f %.9f %.9f %.9f %.9f '
                         % (edge.i, edge.j, t[0], t[1], t[2],
                            q[0], q[1], q[2], q[3])
                         + ' '.join('%.9f' % v for v in upper))
        Path(path).write_text('\n'.join(lines) + '\n')

    @classmethod
    def from_g2o(cls, path):
        graph = cls()
        vertices = {}
        for line in Path(path).read_text().splitlines():
            f = line.split()
            if not f:
                continue
            if f[0] == 'VERTEX_SE3:QUAT':
                vid = int(f[1])
                t = np.array([float(v) for v in f[2:5]])
                q = [float(v) for v in f[5:9]]
                vertices[vid] = make_transform(rotation_from_quaternion(*q), t)
            elif f[0] == 'EDGE_SE3:QUAT':
                i, j = int(f[1]), int(f[2])
                t = np.array([float(v) for v in f[3:6]])
                q = [float(v) for v in f[6:10]]
                upper = [float(v) for v in f[10:31]]
                info = np.zeros((6, 6))
                k = 0
                for r in range(6):
                    for c in range(r, 6):
                        info[r, c] = info[c, r] = upper[k]
                        k += 1
                graph.edges.append(Edge(
                    i, j, make_transform(rotation_from_quaternion(*q), t),
                    info, 'odom' if j == i + 1 else 'loop'))
        for vid in sorted(vertices):
            graph.poses.append(vertices[vid])
        return graph
