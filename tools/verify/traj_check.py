"""
Trajectory arithmetic for the SLAM gates — pure numpy, no ROS.

Two verdicts live here (SLAM plan P0), both read files `traj_record.py`
wrote while a session ran headless:

    traj_check.py loop <gate_bag> <record_dir> [--corrected NAME]
        The loop gate. `make_gate_bag.py loop` plays a sweep out and then
        back, so every return-leg (BACK) frame has its outbound (OUT)
        twin's pose as reference — the same source frame, seen twice.
        For a trajectory, the BACK-vs-OUT error over the return leg is
        the drift the run accumulated; over the last `--tail` seconds it
        is the loop-closure gap. Raw odometry (odom → base_link) is
        scored, then the corrected trajectory (a SLAM node's optimised
        path, or map → odom composed with odometry), and PASS means the
        correction closed the loop tighter than the odometry alone: a
        smaller tail translation error and no worse an angle. FAIL when
        nothing corrected anything — no closure is a fail, not a skip.

    traj_check.py ate <estimate.txt> <groundtruth.txt> [--align se3|sim3]
        Absolute trajectory error against a ground truth in TUM form
        (stamp x y z qx qy qz qw) — the TUM RGB-D sequences the player
        replays carry one. Stamps are associated within --tol, the
        estimate is aligned to the truth by Umeyama (SE(3), or Sim(3)
        when scale is not trusted), and the RMSE / median / max of the
        translation residual is printed. --max-rmse turns it into a
        pass/fail.

Both write a PNG beside the report — the picture instead of a window.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

NS = 1_000_000_000


# ------------------------------------------------------------ geometry

def quat_to_rotation(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n == 0:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def rotation_to_quat(rot):
    """Rotation matrix → (x, y, z, w), Shepperd's branches."""
    m = rot
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, \
            (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, \
            (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, \
            0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, \
            (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def angle_between_deg(rot_a, rot_b):
    cos = (np.trace(rot_a.T @ rot_b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def slerp(q0, q1, alpha):
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = np.arccos(dot)
    return (np.sin((1 - alpha) * theta) * q0 + np.sin(alpha * theta) * q1) \
        / np.sin(theta)


def umeyama(src, dst, with_scale=False):
    """Similarity/rigid fit dst ≈ s·R·src + t (Umeyama 1991)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    u, d, vt = np.linalg.svd(cov)
    sgn = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        sgn[2, 2] = -1
    rot = u @ sgn @ vt
    if with_scale:
        var_s = (xs ** 2).sum() / len(src)
        scale = float(np.trace(np.diag(d) @ sgn) / var_s)
    else:
        scale = 1.0
    trans = mu_d - scale * rot @ mu_s
    return scale, rot, trans


# ------------------------------------------------------------ files

class Trajectory:
    """Stamps (float s), positions (N,3), rotations (N,3,3)."""

    def __init__(self, stamps, positions, rotations):
        self.t = np.asarray(stamps, dtype=np.float64)
        self.p = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
        self.r = np.asarray(rotations, dtype=np.float64).reshape(-1, 3, 3)

    def __len__(self):
        return len(self.t)

    @classmethod
    def read_tum(cls, path):
        stamps, pos, rots = [], [], []
        for line in Path(path).read_text().splitlines():
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            f = line.split()
            if len(f) < 8:
                continue
            stamps.append(float(f[0]))
            pos.append([float(f[1]), float(f[2]), float(f[3])])
            rots.append(quat_to_rotation(*map(float, f[4:8])))
        return cls(stamps, pos, rots)

    def write_tum(self, path, header=''):
        with open(path, 'w') as fh:
            if header:
                fh.write(f'# {header}\n')
            fh.write('# stamp x y z qx qy qz qw\n')
            for t, p, r in zip(self.t, self.p, self.r):
                q = rotation_to_quat(r)
                fh.write(f'{t:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} '
                         f'{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n')

    def with_stamps(self, stamps):
        return Trajectory(stamps, self.p, self.r)

    def subset(self, mask):
        return Trajectory(self.t[mask], self.p[mask], self.r[mask])

    def sorted(self):
        order = np.argsort(self.t)
        return self.subset(order)

    def interpolate(self, when, max_gap):
        """Pose at `when`: linear translation, slerp rotation. None if the
        bracketing samples are further apart than max_gap or `when` lies
        outside the trajectory."""
        if len(self) == 0 or when < self.t[0] or when > self.t[-1]:
            return None
        i = int(np.searchsorted(self.t, when))
        if i < len(self) and self.t[i] == when:
            return self.p[i], self.r[i]
        if i == 0 or i >= len(self):
            return None
        t0, t1 = self.t[i - 1], self.t[i]
        if t1 - t0 > max_gap:
            return None
        a = (when - t0) / (t1 - t0)
        p = (1 - a) * self.p[i - 1] + a * self.p[i]
        q = slerp(rotation_to_quat(self.r[i - 1]), rotation_to_quat(self.r[i]),
                  a)
        return p, quat_to_rotation(*q)


def read_frames(path):
    """frames.txt: header_ns wall_ns per replayed frame."""
    rows = [tuple(int(v) for v in ln.split()) for ln in
            Path(path).read_text().splitlines()
            if ln.strip() and not ln.startswith('#')]
    return np.array(rows, dtype=np.int64).reshape(-1, 2)


def to_bag_time(traj_stamps_ns, traj_wall_ns, frames):
    """Put pose stamps on the bag's receive-time clock (seconds from the
    first replayed frame) — the clock gate.json's segments are written in.
    A stamp within a second of a seen image header is image-stamped
    (rgbd_odometry, rtabmap) and maps through the header timeline; any
    other is wall-stamped (the kp compass) and maps through wall time."""
    headers = frames[:, 0]
    header0, wall0 = frames[0]
    out = np.empty(len(traj_stamps_ns))
    for k, (stamp, wall) in enumerate(zip(traj_stamps_ns, traj_wall_ns)):
        i = int(np.searchsorted(headers, stamp))
        near = min(abs(int(headers[j]) - int(stamp))
                   for j in (i - 1, i) if 0 <= j < len(headers))
        out[k] = ((stamp - header0) / NS if near < NS
                  else (wall - wall0) / NS)
    return out


def read_recorded(record_dir, name):
    """A trajectory `traj_record.py` wrote: TUM columns + a wall_ns column
    (nanosecond stamps kept exact in a side file)."""
    d = Path(record_dir)
    txt = d / f'{name}.txt'
    if not txt.exists():
        return None, None
    traj = Trajectory.read_tum(txt)
    ns_file = d / f'{name}.ns'
    if ns_file.exists() and len(traj) > 0:
        cols = np.loadtxt(ns_file, dtype=np.int64, ndmin=2).reshape(-1, 2)
        stamps_ns, wall_ns = cols[:, 0], cols[:, 1]
    else:
        stamps_ns = (traj.t * NS).astype(np.int64)
        wall_ns = stamps_ns
    return traj, (stamps_ns, wall_ns)


# ------------------------------------------------------------ loop gate

def loop_errors(traj_bag, gate, tail, max_gap):
    """BACK-vs-OUT errors for a trajectory on bag time.

    Returns (rows, tail_rows) where a row is (bag_t, angle_deg, dist_m)."""
    segs = {s['name']: s for s in gate['segments']}
    out_seg, back_seg = segs[gate['compare']['reference']], \
        segs[gate['compare']['trial']]
    in_out = (traj_bag.t >= out_seg['out_t0']) & (traj_bag.t < out_seg['out_t1'])
    in_back = (traj_bag.t >= back_seg['out_t0']) & \
        (traj_bag.t < back_seg['out_t1'])
    # OUT on the source clock, sorted for interpolation.
    outbound = traj_bag.subset(in_out)
    outbound = outbound.with_stamps(
        outbound.t - out_seg['out_t0'] + out_seg['src_t0']).sorted()
    rows = []
    for t, p, r in zip(traj_bag.t[in_back], traj_bag.p[in_back],
                       traj_bag.r[in_back]):
        if back_seg['kind'] == 'rev':
            src_t = back_seg['src_t1'] - (t - back_seg['out_t0'])
        else:
            src_t = t - back_seg['out_t0'] + back_seg['src_t0']
        ref = outbound.interpolate(src_t, max_gap)
        if ref is None:
            continue
        p_ref, r_ref = ref
        rows.append((float(t), angle_between_deg(r_ref, r),
                     float(np.linalg.norm(p - p_ref))))
    tail_rows = [row for row in rows if row[0] >= back_seg['out_t1'] - tail]
    return rows, tail_rows


def stats(rows, k):
    if not rows:
        return None
    vals = np.array([r[k] for r in rows])
    return {'median': float(np.median(vals)),
            'p90': float(np.percentile(vals, 90)),
            'max': float(vals.max()), 'n': int(len(vals))}


def compose_corrected(odom, odom_ns, map_odom, map_odom_ns):
    """map → base_link by composing the latest map → odom at each odometry
    stamp with odom → base_link — the live-corrected trajectory (what a
    TF listener would see), as opposed to the optimised *past*."""
    if map_odom is None or len(map_odom) == 0:
        return None
    order = np.argsort(map_odom_ns[1])       # by wall time of arrival
    mo_wall = map_odom_ns[1][order]
    mo_p, mo_r = map_odom.p[order], map_odom.r[order]
    ps, rs = [], []
    for wall, p, r in zip(odom_ns[1], odom.p, odom.r):
        i = int(np.searchsorted(mo_wall, wall, side='right')) - 1
        if i < 0:
            ps.append(p)
            rs.append(r)
            continue
        ps.append(mo_r[i] @ p + mo_p[i])
        rs.append(mo_r[i] @ r)
    return Trajectory(odom.t, ps, rs)


def dense_from_graph(odom_dense, graph_opt, max_gap=1.5):
    """Lift a sparse optimised graph onto the dense odometry.

    A pose graph optimises keyframe poses only; between them the odometry
    is still the best local estimate. For every dense odometry pose take
    the nearest graph node in time and apply that node's correction
    (optimised ∘ odometry⁻¹) — the composition a SLAM node publishes as
    map → odom, applied per node to the past instead of only the latest
    to the present. The odometry pose at each node is interpolated from
    the recorded odom → base_link — never taken from the SLAM node's own
    per-node odometry record: RTAB-Map re-bases that on every odometry
    reset (new map session), so it is not the TF the correction applies
    to (found on the first loop-gate run, 2026-08-18)."""
    if graph_opt is None or len(graph_opt) == 0:
        return None
    graph_opt = graph_opt.sorted()
    corrections = []      # (stamp, R_c, t_c)
    for t, p, r in zip(graph_opt.t, graph_opt.p, graph_opt.r):
        ref = odom_dense.interpolate(t, max_gap)
        if ref is None:
            continue
        p_o, r_o = ref
        r_c = r @ r_o.T
        t_c = p - r_c @ p_o
        corrections.append((t, r_c, t_c))
    if not corrections:
        return None
    ct = np.array([c[0] for c in corrections])
    ps, rs = [], []
    for t, p, r in zip(odom_dense.t, odom_dense.p, odom_dense.r):
        k = int(np.argmin(np.abs(ct - t)))
        _, r_c, t_c = corrections[k]
        ps.append(r_c @ p + t_c)
        rs.append(r_c @ r)
    return Trajectory(odom_dense.t, ps, rs), len(corrections)


def run_loop(args):
    gate_dir, rec = Path(args.gate), Path(args.record)
    gate = json.loads((gate_dir / 'gate.json').read_text())
    frames = read_frames(rec / 'frames.txt')
    report = {'gate': str(gate_dir), 'record': str(rec),
              'frames': int(len(frames)), 'tail_s': args.tail}
    odom, odom_ns = read_recorded(rec, 'odom')
    if odom is None or len(odom) == 0 or len(frames) == 0:
        report['verdict'] = 'NO DATA'
        print('=== gate loop: NO DATA (no odometry recorded) ===')
        (rec / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        return 2

    tracks = {}
    odom_bag = odom.with_stamps(to_bag_time(*odom_ns, frames))
    tracks['odom'] = odom_bag
    # Corrected candidates, best first: a stamped optimised path
    # (path_*.txt — the fork's /world/trajectory), a sparse optimised
    # graph lifted onto the odometry (graph_<name>_slam.txt from
    # rtabmap-report, or any TUM file of optimised keyframe poses), and
    # last the live map → odom composition (corrects the present only).
    candidates = []
    sparse = []      # (name, Trajectory of optimised keyframe poses)
    for path_file in sorted(rec.glob('path_*.txt')):
        traj, ns = read_recorded(rec, path_file.stem)
        if traj is not None and len(traj) > 1:
            sparse.append((path_file.stem, traj))
    for slam_file in sorted(rec.glob('graph_*_slam.txt')):
        sparse.append((slam_file.stem[:-5], Trajectory.read_tum(slam_file)))
    # Optimised keyframe poses are sparse (a node every few seconds);
    # score the dense trajectory they imply by lifting the per-node
    # correction onto the odometry — see dense_from_graph.
    for base, graph_opt in sparse:
        lifted = dense_from_graph(odom.sorted(), graph_opt, args.max_gap)
        if lifted is None:
            continue
        dense, n_nodes = lifted
        order = np.argsort(odom.t)       # dense shares odom's stamps
        tracks[base] = dense.with_stamps(
            to_bag_time(odom_ns[0][order], odom_ns[1][order], frames))
        report[f'{base}_nodes'] = n_nodes
        candidates.append(base)
    map_odom, mo_ns = read_recorded(rec, 'map_odom')
    live = compose_corrected(odom, odom_ns, map_odom, mo_ns)
    if live is not None:
        tracks['live_corrected'] = live.with_stamps(
            to_bag_time(*odom_ns, frames))
        candidates.append('live_corrected')
    corrected_name = args.corrected or (candidates[0] if candidates else None)
    report['map_odom_updates'] = 0 if map_odom is None else int(len(map_odom))
    report['corrected'] = corrected_name

    results = {}
    for name, traj in tracks.items():
        rows, tail_rows = loop_errors(traj, gate, args.tail, args.max_gap)
        results[name] = {
            'poses': int(len(traj)), 'paired': len(rows),
            'return_leg': {'angle_deg': stats(rows, 1),
                           'translation_m': stats(rows, 2)},
            'tail': {'angle_deg': stats(tail_rows, 1),
                     'translation_m': stats(tail_rows, 2)},
            '_rows': rows,
        }
    report['tracks'] = {k: {kk: vv for kk, vv in v.items()
                            if not kk.startswith('_')}
                        for k, v in results.items()}

    raw = results['odom']
    verdict = 'FAIL'
    reason = ''
    if raw['tail']['translation_m'] is None:
        reason = 'odometry has no poses in the return-leg tail'
    elif corrected_name is None:
        reason = 'nothing published a corrected trajectory — no loop closed'
    elif results[corrected_name]['tail']['translation_m'] is None:
        reason = f'{corrected_name} has no poses in the return-leg tail'
    else:
        cor = results[corrected_name]
        d_raw = raw['tail']['translation_m']['median']
        d_cor = cor['tail']['translation_m']['median']
        a_raw = raw['tail']['angle_deg']['median']
        a_cor = cor['tail']['angle_deg']['median']
        closer = d_cor < d_raw
        no_worse_angle = a_cor <= a_raw + args.angle_slack
        verdict = 'PASS' if closer and no_worse_angle else 'FAIL'
        reason = (f'tail translation {d_raw:.3f} → {d_cor:.3f} m, '
                  f'angle {a_raw:.2f} → {a_cor:.2f}°')
    report['verdict'] = verdict
    report['reason'] = reason
    (rec / 'report.json').write_text(json.dumps(report, indent=2) + '\n')

    try:
        plot_loop(gate, tracks, results, rec / 'poses.png')
    except Exception as exc:  # the plot is a courtesy, not the verdict
        print(f'plot skipped: {exc}', file=sys.stderr)

    print(f'\n=== gate loop: {verdict} — {reason} ===')
    print(f'frames {len(frames)}, map→odom updates '
          f'{report["map_odom_updates"]}, corrected = {corrected_name}')
    for name, res in results.items():
        print(f'  {name}: {res["poses"]} poses, {res["paired"]} paired '
              f'BACK-vs-OUT')
        for scope in ('return_leg', 'tail'):
            a, d = res[scope]['angle_deg'], res[scope]['translation_m']
            if a is None:
                print(f'    {scope:<10} no poses')
                continue
            print(f'    {scope:<10} translation median {d["median"]:.3f} m '
                  f'p90 {d["p90"]:.3f} max {d["max"]:.3f} | angle median '
                  f'{a["median"]:.2f}° p90 {a["p90"]:.2f} max {a["max"]:.2f} '
                  f'(n={a["n"]})')
    print(f'  report: {rec}/report.json  plot: {rec}/poses.png')
    return 0 if verdict == 'PASS' else 1


def plot_loop(gate, tracks, results, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    segs = {s['name']: s for s in gate['segments']}
    out_seg = segs[gate['compare']['reference']]
    for name, traj in tracks.items():
        in_out = traj.t < out_seg['out_t1']
        axes[0].plot(traj.p[in_out, 0], traj.p[in_out, 1], '-', lw=1,
                     label=f'{name} OUT')
        axes[0].plot(traj.p[~in_out, 0], traj.p[~in_out, 1], '--', lw=1,
                     label=f'{name} BACK')
    axes[0].set_aspect('equal', 'datalim')
    axes[0].set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    axes[0].set_title('top view — out solid, back dashed')
    axes[0].legend(fontsize=7)
    for name, res in results.items():
        rows = res['_rows']
        if not rows:
            continue
        ts = [r[0] for r in rows]
        axes[1].plot(ts, [r[2] * 100 for r in rows], '.', ms=2, label=name)
        axes[2].plot(ts, [r[1] for r in rows], '.', ms=2, label=name)
    axes[1].set_ylabel('BACK vs OUT translation (cm)')
    axes[2].set_ylabel('BACK vs OUT angle (°)')
    for ax in axes[1:]:
        ax.set_xlabel('bag time (s)')
        ax.legend(fontsize=7)
        ax.axvspan(out_seg['out_t1'] - 0.01, out_seg['out_t1'] + 0.01,
                   color='k')
    fig.suptitle(f'loop gate — {Path(gate["source"]).name} out and back')
    fig.tight_layout()
    fig.savefig(out, dpi=110)


# ------------------------------------------------------------ ATE

def associate(a, b, tol):
    """Index pairs (i, j) with |a[i] - b[j]| <= tol, nearest, one-to-one
    greedy by a's order."""
    j_all = np.searchsorted(b, a)
    pairs, used = [], set()
    for i, j in enumerate(j_all):
        best, best_d = None, tol + 1
        for jj in (j - 1, j):
            if 0 <= jj < len(b) and jj not in used:
                d = abs(a[i] - b[jj])
                if d < best_d:
                    best, best_d = jj, d
        if best is not None and best_d <= tol:
            pairs.append((i, best))
            used.add(best)
    return pairs


def ate(est, gt, tol=0.02, align='se3'):
    pairs = associate(est.t, gt.t, tol)
    if len(pairs) < 3:
        return None
    ei = np.array([p[0] for p in pairs])
    gi = np.array([p[1] for p in pairs])
    src, dst = est.p[ei], gt.p[gi]
    if align == 'none':
        s, rot, trans = 1.0, np.eye(3), np.zeros(3)
    else:
        s, rot, trans = umeyama(src, dst, with_scale=(align == 'sim3'))
    aligned = (s * (rot @ src.T)).T + trans
    resid = np.linalg.norm(aligned - dst, axis=1)
    return {'pairs': int(len(pairs)), 'scale': float(s),
            'rmse_m': float(np.sqrt((resid ** 2).mean())),
            'median_m': float(np.median(resid)),
            'max_m': float(resid.max()),
            'span_s': float(est.t[ei[-1]] - est.t[ei[0]]),
            '_aligned': aligned, '_gt': dst, '_stamps': est.t[ei],
            '_resid': resid}


def run_ate(args):
    est = Trajectory.read_tum(args.estimate).sorted()
    gt = Trajectory.read_tum(args.groundtruth).sorted()
    label = Path(args.estimate).stem
    if args.lift:
        # The estimate is a sparse optimised graph: score the dense
        # trajectory it implies (see dense_from_graph).
        dense = Trajectory.read_tum(args.lift).sorted()
        lifted = dense_from_graph(dense, est, args.max_gap)
        if lifted is None:
            print(f'=== ate {label}: NO DATA (graph could not be lifted onto '
                  f'{args.lift})')
            return 2
        est, n_nodes = lifted
        label += '_lifted'
        print(f'lifted {n_nodes} graph nodes onto {len(est)} odometry poses')
    res = ate(est, gt, args.tol, args.align)
    if res is None:
        print(f'=== ate {label}: NO DATA ({len(est)} estimate poses, '
              f'{len(gt)} truth poses, none associated within {args.tol} s)')
        return 2
    verdict = 'PASS' if (args.max_rmse is None or
                         res['rmse_m'] <= args.max_rmse) else 'FAIL'
    print(f'=== ate {label}: {verdict} — RMSE {res["rmse_m"]:.3f} m over '
          f'{res["pairs"]} pairs ({res["span_s"]:.1f} s), median '
          f'{res["median_m"]:.3f} max {res["max_m"]:.3f}, {args.align} '
          f'alignment scale {res["scale"]:.3f}'
          + (f', limit {args.max_rmse} m' if args.max_rmse else '') + ' ===')
    if args.out:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        clean = {k: v for k, v in res.items() if not k.startswith('_')}
        clean.update({'estimate': args.estimate, 'groundtruth':
                      args.groundtruth, 'verdict': verdict,
                      'align': args.align})
        (out / f'ate_{label}.json').write_text(
            json.dumps(clean, indent=2) + '\n')
        try:
            plot_ate(res, gt, label, out / f'ate_{label}.png')
        except Exception as exc:
            print(f'plot skipped: {exc}', file=sys.stderr)
        print(f'  report: {out}/ate_{label}.json')
    return 0 if verdict == 'PASS' else 1


def plot_ate(res, gt, label, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(gt.p[:, 0], gt.p[:, 1], '-', lw=0.8, color='0.6',
                 label='ground truth')
    axes[0].plot(res['_aligned'][:, 0], res['_aligned'][:, 1], '.', ms=2,
                 label=f'{label} (aligned)')
    axes[0].set_aspect('equal', 'datalim')
    axes[0].set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    axes[0].legend(fontsize=8)
    t0 = res['_stamps'][0]
    axes[1].plot(res['_stamps'] - t0, res['_resid'] * 100, '.', ms=2)
    axes[1].set_xlabel('time (s)')
    axes[1].set_ylabel('translation error (cm)')
    axes[1].set_title(f'ATE RMSE {res["rmse_m"] * 100:.1f} cm')
    fig.tight_layout()
    fig.savefig(out, dpi=110)


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    lp = sub.add_parser('loop', help='BACK-vs-OUT verdict on a loop gate')
    lp.add_argument('gate', help='gate bag dir (gate.json inside)')
    lp.add_argument('record', help='traj_record.py output dir')
    lp.add_argument('--corrected',
                    help='track name to judge as the correction (default: '
                         'the first of path_*, graph_*, live_corrected)')
    lp.add_argument('--tail', type=float, default=5.0,
                    help='seconds at the end of BACK scored as the loop gap')
    lp.add_argument('--max-gap', type=float, default=1.5,
                    help='largest OUT sample gap to interpolate across (s)')
    lp.add_argument('--angle-slack', type=float, default=0.5,
                    help='degrees the corrected angle may exceed the raw')
    ap = sub.add_parser('ate', help='ATE against a TUM ground truth')
    ap.add_argument('estimate')
    ap.add_argument('groundtruth')
    ap.add_argument('--tol', type=float, default=0.02)
    ap.add_argument('--align', choices=('se3', 'sim3', 'none'),
                    default='se3')
    ap.add_argument('--max-rmse', type=float)
    ap.add_argument('--lift', metavar='ODOM_TXT',
                    help='treat the estimate as a sparse optimised graph and '
                         'lift it onto this dense odometry file first')
    ap.add_argument('--max-gap', type=float, default=1.5)
    ap.add_argument('--out', help='directory for the json + png')
    args = p.parse_args()
    sys.exit(run_loop(args) if args.cmd == 'loop' else run_ate(args))


if __name__ == '__main__':
    main()
