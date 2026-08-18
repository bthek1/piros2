"""
Watch a gate bag play through the pipeline and decide the gate — no human.

A gate bag (`make_gate_bag.py`) replays a window A of a real sweep, then
something that breaks tracking, then part of A again (A2). Whatever pose
the pipeline reported for a source frame during A is the reference for
that same frame during A2 — so "did the camera come back to where it
was?" is a number, not a judgement: the rotation angle (and, in rgbd
mode, the translation) between the two passes' `odom → base_link`.

This node records `/tf` and the replayed image topic while the bag
plays, maps every transform onto bag time, pairs A2 poses with their A
reference by source time, and reports median/p90 error over A2's tail
(after `--settle` seconds, the time the recovery is allowed to take).
PASS needs the tail under `gate.json`'s thresholds *and* every expected
log line present in the launch log (`tracking lost`, `relocalized
against keyframe`, `snapping odometry` for rgbd) — the numbers say the
pose is right, the log says it is right *for the reason the plan built*.

Two clocks, handled explicitly (the camera's header stamps lag receive
by ~0.73 s and the two odometry sources stamp their TF differently):
rgbd_odometry stamps TF with the image header stamp, the kp detector
with its own wall clock. A TF stamp that lands within a second of a
seen header stamp is mapped through the header timeline, anything else
through wall time since the first frame arrived. Both land on the
receive-time bag clock the gate's segments are written in.

Writes report.json, poses.csv and poses.png (yaw — and x/y for rgbd —
against bag time with the segments shaded, plus the A2-vs-A error) into
--out; the png is the picture to look at instead of an RViz window. Exit
0 = PASS, 1 = FAIL, 2 = no usable data (pipeline never published).

Run through `just gate` (ROS environment; /usr/bin/python3).
"""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from tf2_msgs.msg import TFMessage

NS = 1_000_000_000


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


def yaw_deg(rot):
    return float(np.degrees(np.arctan2(rot[1, 0], rot[0, 0])))


def angle_between_deg(rot_a, rot_b):
    cos = (np.trace(rot_a.T @ rot_b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


class GateRecorder(Node):

    def __init__(self, image_topic):
        super().__init__('gate_check')
        self.images = []      # (header_ns, wall_ns)
        self.poses = []       # (stamp_ns, wall_ns, R, t)
        reliable = QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CompressedImage, image_topic,
                                 self.on_image, reliable)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.last_image_wall = None

    def on_image(self, msg):
        now = time.time_ns()
        self.images.append(
            (msg.header.stamp.sec * NS + msg.header.stamp.nanosec, now))
        self.last_image_wall = now

    def on_tf(self, msg):
        now = time.time_ns()
        for tf in msg.transforms:
            if tf.header.frame_id != 'odom' or tf.child_frame_id != 'base_link':
                continue
            q, t = tf.transform.rotation, tf.transform.translation
            self.poses.append((
                tf.header.stamp.sec * NS + tf.header.stamp.nanosec, now,
                quat_to_rotation(q.x, q.y, q.z, q.w),
                np.array([t.x, t.y, t.z])))


def record(gate, args):
    rclpy.init()
    node = GateRecorder(gate['image_topic'])
    node.get_logger().info(
        f'waiting for {gate["image_topic"]} — start the bag now')
    t_start = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.last_image_wall is None:
                if time.time() - t_start > args.wait:
                    node.get_logger().error(
                        f'no frames within {args.wait:.0f} s')
                    break
                continue
            idle = (time.time_ns() - node.last_image_wall) / NS
            if idle > args.idle:
                node.get_logger().info(
                    f'no frames for {idle:.1f} s — bag finished, evaluating')
                break
            if time.time() - t_start > args.timeout:
                node.get_logger().warn('timeout reached, evaluating')
                break
    finally:
        images, poses = node.images, node.poses
        node.destroy_node()
        rclpy.shutdown()
    return images, poses


def to_bag_time(images, poses):
    """Put every pose on the bag's receive-time clock (seconds)."""
    headers = np.array([h for h, _ in images], dtype=np.int64)
    header0, wall0 = images[0]
    out = []
    for stamp, wall, rot, trans in poses:
        i = int(np.searchsorted(headers, stamp))
        near = min(abs(headers[j] - stamp)
                   for j in (i - 1, i) if 0 <= j < len(headers))
        if near < NS:
            t = (stamp - header0) / NS          # image-stamped (rgbd)
        else:
            t = (wall - wall0) / NS             # wall-stamped (kp)
        out.append((t, rot, trans))
    return out


def evaluate(gate, images, poses, log_text, args):
    report = {'mode': gate['mode'], 'odom': gate['odom'],
              'frames_seen': len(images), 'poses_seen': len(poses)}
    if not images or not poses:
        report['verdict'] = 'NO DATA'
        return report, []
    timeline = to_bag_time(images, poses)
    segs = {s['name']: s for s in gate['segments']}
    ref, trial = (segs[gate['compare']['reference']],
                  segs[gate['compare']['trial']])

    def in_seg(seg, t):
        return seg['out_t0'] <= t < seg['out_t1']

    ref_poses = [(t - ref['out_t0'] + ref['src_t0'], r, tr)
                 for t, r, tr in timeline if in_seg(ref, t)]
    trial_poses = [(t, t - trial['out_t0'] + trial['src_t0'], r, tr)
                   for t, r, tr in timeline if in_seg(trial, t)]
    report['reference_poses'] = len(ref_poses)
    report['trial_poses'] = len(trial_poses)
    errors = []   # (bag_t, angle_deg, translation_m)
    if ref_poses:
        ref_times = np.array([p[0] for p in ref_poses])
        for bag_t, src_t, rot, trans in trial_poses:
            i = int(np.argmin(np.abs(ref_times - src_t)))
            if abs(ref_times[i] - src_t) > args.pair_tolerance:
                continue
            _, r_ref, t_ref = ref_poses[i]
            errors.append((bag_t, angle_between_deg(r_ref, rot),
                           float(np.linalg.norm(trans - t_ref))))
    report['paired'] = len(errors)
    settled = [e for e in errors if e[0] >= trial['out_t0'] + args.settle]
    early = [e for e in errors if e[0] < trial['out_t0'] + args.settle]

    def stats(rows, k):
        vals = np.array([r[k] for r in rows])
        return {'median': float(np.median(vals)),
                'p90': float(np.percentile(vals, 90)),
                'max': float(vals.max()), 'n': int(len(vals))}

    thresholds = gate['thresholds']
    verdicts = []
    if settled:
        report['tail'] = {'angle_deg': stats(settled, 1),
                          'translation_m': stats(settled, 2)}
        if early:
            report['before_settle'] = {'angle_deg': stats(early, 1),
                                       'translation_m': stats(early, 2)}
        for key, limit in thresholds.items():
            ok = report['tail'][key]['median'] <= limit
            verdicts.append(ok)
            report[f'{key}_ok'] = ok
    else:
        report['tail'] = None
        verdicts.append(False)

    found = {line: (line in log_text) for line in gate['expect_log']}
    report['log_lines'] = found
    report['log_hits'] = [ln.strip() for ln in log_text.splitlines()
                          if any(k in ln for k in gate['expect_log'])][-12:]
    verdicts.append(all(found.values()) if log_text else False)
    if not log_text:
        report['log_note'] = 'no launch log given — log lines unchecked'
    report['verdict'] = 'PASS' if all(verdicts) else 'FAIL'
    return report, (timeline, errors)


def plot(gate, timeline, errors, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rgbd = gate['odom'] == 'rgbd'
    rows = 3 if rgbd else 2
    fig, axes = plt.subplots(rows, 1, figsize=(10, 2 + 2 * rows), sharex=True)
    ts = np.array([t for t, _, _ in timeline])
    yaws = np.array([yaw_deg(r) for _, r, _ in timeline])
    axes[0].plot(ts, yaws, '.', ms=2)
    axes[0].set_ylabel('yaw odom→base_link (°)')
    row = 1
    if rgbd:
        xy = np.array([tr[:2] for _, _, tr in timeline])
        axes[1].plot(ts, xy[:, 0], '.', ms=2, label='x')
        axes[1].plot(ts, xy[:, 1], '.', ms=2, label='y')
        axes[1].set_ylabel('position (m)')
        axes[1].legend(loc='upper left')
        row = 2
    if errors:
        et = np.array([e[0] for e in errors])
        axes[row].plot(et, [e[1] for e in errors], '.', ms=2,
                       label='angle vs A (°)')
        if rgbd:
            axes[row].plot(et, [e[2] * 100 for e in errors], '.', ms=2,
                           label='translation vs A (cm)')
        axes[row].axhline(gate['thresholds']['angle_deg'], color='r',
                          lw=0.8, ls='--')
        axes[row].legend(loc='upper left')
    axes[row].set_ylabel('A2 error vs A')
    axes[row].set_xlabel('bag time (s)')
    for ax in axes:
        for s in gate['segments']:
            colour = ('#ffdddd' if s['kind'] == 'fill'
                      else '#ddffdd' if s['name'] in gate['compare'].values()
                      else '#dddddd')
            ax.axvspan(s['out_t0'], s['out_t1'], color=colour, alpha=0.5)
            ax.text(s['out_t0'] + 0.1, ax.get_ylim()[1], s['name'],
                    va='top', fontsize=8)
    fig.suptitle(f'gate {gate["mode"]} ({gate["odom"]} odom) — '
                 f'{Path(gate["source"]).name}')
    fig.tight_layout()
    fig.savefig(out / 'poses.png', dpi=110)


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('gate', help='path to the gate bag (dir with gate.json)')
    p.add_argument('--out', required=True, help='report directory')
    p.add_argument('--log', help='launch log to grep for the expected lines')
    p.add_argument('--settle', type=float, default=3.0,
                   help='seconds of A2 the recovery may take (default 3)')
    p.add_argument('--pair-tolerance', type=float, default=0.1)
    p.add_argument('--wait', type=float, default=60.0,
                   help='give up if no frame arrives in this long')
    p.add_argument('--idle', type=float, default=4.0,
                   help='no frames for this long = bag finished')
    p.add_argument('--timeout', type=float, default=180.0)
    args = p.parse_args()

    gate = json.loads((Path(args.gate) / 'gate.json').read_text())
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    images, poses = record(gate, args)
    log_text = Path(args.log).read_text(errors='replace') if args.log else ''
    report, data = evaluate(gate, images, poses, log_text, args)
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    if data:
        timeline, errors = data
        with open(out / 'poses.csv', 'w') as f:
            f.write('bag_t,yaw_deg,x,y,z\n')
            for t, r, tr in timeline:
                f.write(f'{t:.3f},{yaw_deg(r):.3f},{tr[0]:.4f},'
                        f'{tr[1]:.4f},{tr[2]:.4f}\n')
        try:
            plot(gate, timeline, errors, out)
        except Exception as exc:  # the plot is a courtesy, not the verdict
            print(f'plot skipped: {exc}', file=sys.stderr)

    print(f'\n=== gate {gate["mode"]} ({gate["odom"]}): {report["verdict"]} ===')
    print(f'frames {report["frames_seen"]}, poses {report["poses_seen"]}, '
          f'paired A2-vs-A {report.get("paired", 0)}')
    if report.get('tail'):
        for key, limit in gate['thresholds'].items():
            s = report['tail'][key]
            unit = '°' if key == 'angle_deg' else ' m'
            before = report.get('before_settle', {}).get(key)
            pre = (f' (before settle: median {before["median"]:.2f}{unit})'
                   if before else '')
            print(f'  {key}: median {s["median"]:.2f}{unit} p90 '
                  f'{s["p90"]:.2f}{unit} max {s["max"]:.2f}{unit} '
                  f'over {s["n"]} poses — limit {limit}{unit} '
                  f'{"ok" if report[f"{key}_ok"] else "EXCEEDED"}{pre}')
    else:
        print('  no A2 poses after settle — the pipeline published nothing '
              'there')
    for line, ok in report['log_lines'].items():
        print(f'  log {"✓" if ok else "✗"} {line!r}')
    for hit in report['log_hits']:
        print(f'    | {hit[-140:]}')
    print(f'  report: {out}/report.json  plot: {out}/poses.png')
    sys.exit(0 if report['verdict'] == 'PASS' else
             2 if report['verdict'] == 'NO DATA' else 1)


if __name__ == '__main__':
    main()
