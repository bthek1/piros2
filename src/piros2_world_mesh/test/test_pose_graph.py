"""
Pose-graph optimiser tests (SLAM plan P2) — pure functions, no ROS.

Three kinds of evidence:

- Lie-group arithmetic round-trips (exp/log on SO(3) and SE(3), the
  adjoint identity) — the optimiser is only as right as these.
- Synthetic graphs with a known truth: a drifted loop that a single
  correct closure must pull back onto the circle; a planted *wrong*
  closure that Huber must neutralise.
- The installed `g2o` binary as an oracle: the same graph written to
  .g2o and optimised by g2o lands where our Gauss-Newton lands. Skipped
  cleanly when g2o is not on PATH.
"""

import os
from pathlib import Path
import shutil
import subprocess

import numpy as np
from piros2_world_mesh.pose_graph import (Edge, information_matrix,
                                          PoseGraph)
from piros2_world_mesh.se3 import (adjoint, invert, make_transform, se3_exp,
                                   se3_log, so3_exp, so3_log)
import pytest


def rotz(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def angle_deg(r_a, r_b):
    cos = (np.trace(r_a.T @ r_b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


# ---------------------------------------------------------------- Lie

@pytest.mark.parametrize('phi', [
    [0.0, 0.0, 0.0], [1e-7, 0.0, 0.0], [0.3, -0.2, 0.5],
    [0.0, 0.0, 3.0], [np.pi - 1e-4, 0.0, 0.0]])
def test_so3_exp_log_round_trip(phi):
    phi = np.array(phi)
    back = so3_log(so3_exp(phi))
    assert np.allclose(back, phi, atol=1e-6)


def test_so3_exp_is_a_rotation():
    r = so3_exp([0.4, 0.5, -0.6])
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(r), 1.0)


@pytest.mark.parametrize('xi', [
    np.zeros(6), [0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
    [0.5, -0.1, 0.2, 0.3, -0.4, 0.1], [1.0, 2.0, 3.0, 0.0, 0.0, 2.5]])
def test_se3_exp_log_round_trip(xi):
    xi = np.array(xi, dtype=float)
    assert np.allclose(se3_log(se3_exp(xi)), xi, atol=1e-8)


def test_se3_exp_of_pure_translation_is_a_translation():
    t = se3_exp([1.0, 2.0, 3.0, 0.0, 0.0, 0.0])
    assert np.allclose(t[:3, :3], np.eye(3))
    assert np.allclose(t[:3, 3], [1.0, 2.0, 3.0])


def test_adjoint_moves_a_twist_across_a_transform():
    t_mat = make_transform(so3_exp([0.2, -0.3, 0.4]), [1.0, -2.0, 0.5])
    xi = np.array([0.05, -0.02, 0.03, 0.01, 0.04, -0.02])
    lhs = t_mat @ se3_exp(xi)
    rhs = se3_exp(adjoint(t_mat) @ xi) @ t_mat
    assert np.allclose(lhs, rhs, atol=1e-9)


# ---------------------------------------------------------------- graphs

def circle_truth(n=24, radius=1.0):
    """Ground-truth poses walking a circle, heading tangent to it."""
    poses = []
    for k in range(n):
        ang = 2 * np.pi * k / n
        heading = np.degrees(ang) + 90.0
        poses.append(make_transform(
            rotz(heading), [radius * np.cos(ang), radius * np.sin(ang), 0.0]))
    return poses


def drifted_graph(n=24, yaw_bias_deg=0.6, seed=0, info=None):
    """
    Odometry with a systematic yaw bias, integrated into node estimates.

    Returns the graph (no loop edge yet) and the truth to compare against.
    """
    rng = np.random.default_rng(seed)
    truth = circle_truth(n)
    graph = PoseGraph()
    graph.add_node(truth[0])
    measurements = []
    for k in range(1, n):
        z_true = invert(truth[k - 1]) @ truth[k]
        noise = se3_exp(np.concatenate([
            rng.normal(0, 0.003, 3),
            [0.0, 0.0, np.radians(yaw_bias_deg)] + rng.normal(0, 0.001, 3)]))
        z = z_true @ noise
        measurements.append(z)
        graph.add_node(graph.poses[-1] @ z)
    for k, z in enumerate(measurements):
        graph.add_edge(k, k + 1, z, info)
    return graph, truth


def end_error(graph, truth):
    return (float(np.linalg.norm(graph.poses[-1][:3, 3] - truth[-1][:3, 3])),
            angle_deg(graph.poses[-1][:3, :3], truth[-1][:3, :3]))


def test_drift_accumulates_without_a_loop_edge():
    graph, truth = drifted_graph()
    d, a = end_error(graph, truth)
    assert d > 0.1 and a > 5.0      # the yaw bias walked the end away


def test_one_correct_loop_closure_pulls_the_loop_shut():
    graph, truth = drifted_graph()
    n = len(graph)
    d0, a0 = end_error(graph, truth)
    # A loop edge from the last node back to the first, measured
    # exactly (a place recognition that got it right).
    z_loop = invert(truth[n - 1]) @ truth[0]
    graph.add_edge(n - 1, 0, z_loop,
                   information_matrix(0.01, np.radians(0.5)), kind='loop')
    stats = graph.optimize(fixed=(0,))
    d1, a1 = end_error(graph, truth)
    assert stats['chi2_after'] < stats['chi2_before'] * 0.05
    assert d1 < d0 * 0.2 and a1 < a0 * 0.2
    # And the correction is spread over the whole loop, not dumped on
    # the last node: every node ends within a few cm of truth.
    worst = max(np.linalg.norm(p[:3, 3] - t[:3, 3])
                for p, t in zip(graph.poses, truth))
    assert worst < 0.06
    assert np.allclose(graph.poses[0], truth[0])     # fixed stays fixed


def test_optimised_poses_stay_on_se3():
    graph, truth = drifted_graph()
    n = len(graph)
    graph.add_edge(n - 1, 0, invert(truth[n - 1]) @ truth[0], kind='loop')
    graph.optimize()
    for p in graph.poses:
        r = p[:3, :3]
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)
        assert np.allclose(p[3], [0, 0, 0, 1])


def test_huber_neutralises_a_wrong_loop_closure():
    # A planted wrong closure: "node 12 is where node 0 is" (relative
    # pose identity) — a lookalike wall on the far side of the room.
    # Without a robust kernel the graph folds toward it; with Huber the
    # odometry chain wins and the graph barely moves.
    bogus = np.eye(4)
    info = information_matrix(0.01, np.radians(0.5))   # same trust both

    naive, truth = drifted_graph(info=info)
    before = [p.copy() for p in naive.poses]
    naive.add_edge(12, 0, bogus, info, kind='loop')
    naive.optimize()
    moved_naive = max(np.linalg.norm(a[:3, 3] - b[:3, 3])
                      for a, b in zip(before, naive.poses))

    robust, _ = drifted_graph(info=info)
    robust.add_edge(12, 0, bogus, info, kind='loop')
    robust.optimize(huber=1.0)
    moved_robust = max(np.linalg.norm(a[:3, 3] - b[:3, 3])
                       for a, b in zip(before, robust.poses))
    assert moved_naive > 0.3
    assert moved_robust < moved_naive * 0.25


def test_empty_and_trivial_graphs_are_no_ops():
    graph = PoseGraph()
    stats = graph.optimize()
    assert stats['iterations'] == 0
    graph.add_node(np.eye(4))
    stats = graph.optimize()
    assert stats['iterations'] == 0 and stats['max_shift_m'] == 0.0


def test_g2o_round_trip_preserves_the_graph(tmp_path):
    graph, truth = drifted_graph(n=8)
    graph.add_edge(7, 0, invert(truth[7]) @ truth[0],
                   information_matrix(0.02, 0.01), kind='loop')
    path = tmp_path / 'g.g2o'
    graph.to_g2o(path)
    back = PoseGraph.from_g2o(path)
    assert len(back) == len(graph) and len(back.edges) == len(graph.edges)
    for a, b in zip(graph.poses, back.poses):
        assert np.allclose(a, b, atol=1e-8)
    for ea, eb in zip(graph.edges, back.edges):
        assert (ea.i, ea.j) == (eb.i, eb.j)
        assert np.allclose(ea.measurement, eb.measurement, atol=1e-8)
        assert np.allclose(ea.information, eb.information)
    assert back.edges[-1].kind == 'loop'


G2O = shutil.which('g2o') or (
    '/opt/ros/jazzy/bin/g2o' if os.path.exists('/opt/ros/jazzy/bin/g2o')
    else None)


@pytest.mark.skipif(G2O is None, reason='g2o binary not installed')
def test_matches_g2o_on_the_same_graph(tmp_path):
    graph, truth = drifted_graph(n=24, info=information_matrix(0.01, 0.01))
    n = len(graph)
    graph.add_edge(n - 1, 0, invert(truth[n - 1]) @ truth[0],
                   information_matrix(0.005, 0.005), kind='loop')
    # g2o's EDGE_SE3:QUAT residual is [translation, quaternion xyz]:
    # the rotation part is sin(θ/2)·axis ≈ φ/2, half of our log-map
    # residual φ. Scale its rotation information by 4 so both optimise
    # the same quadratic cost — otherwise the two answers differ by a
    # real (if small) amount, not by convergence.
    twin = PoseGraph([p.copy() for p in graph.poses],
                     [Edge(e.i, e.j, e.measurement.copy(),
                           e.information.copy(), e.kind)
                      for e in graph.edges])
    for e in twin.edges:
        e.information[3:, 3:] *= 4.0
    src = tmp_path / 'in.g2o'
    dst = tmp_path / 'out.g2o'
    twin.to_g2o(src, fixed=(0,))
    env = dict(os.environ)
    lib = str(Path(G2O).resolve().parent.parent / 'lib')
    env['LD_LIBRARY_PATH'] = lib + ':' + env.get('LD_LIBRARY_PATH', '')
    result = subprocess.run(
        [G2O, '-solver', 'lm_dense', '-i', '50', '-o', str(dst), str(src)],
        capture_output=True, text=True, env=env, timeout=60)
    assert result.returncode == 0, result.stderr[-500:]
    oracle = PoseGraph.from_g2o(dst)
    graph.optimize(fixed=(0,), max_iters=50)
    worst_m = max(np.linalg.norm(a[:3, 3] - b[:3, 3])
                  for a, b in zip(graph.poses, oracle.poses))
    worst_deg = max(angle_deg(a[:3, :3], b[:3, :3])
                    for a, b in zip(graph.poses, oracle.poses))
    assert worst_m < 1e-3, worst_m
    assert worst_deg < 0.2, worst_deg
    # The decisive check: under OUR cost, our optimum is no worse than
    # g2o's (the two sit in the same flat valley — measured 0.07° apart
    # at identical chi² to six decimals).
    ours = graph.chi2()
    graph.poses = [p.copy() for p in oracle.poses]
    theirs = graph.chi2()
    assert ours <= theirs * (1 + 1e-6) + 1e-9, (ours, theirs)
