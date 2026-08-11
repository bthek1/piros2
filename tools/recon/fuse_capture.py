"""
Fuse an exported capture (captures/<name>) into a TSDF mesh.

World fusion plan P4 — two fusions of the same capture, differing only
in the trajectory file, which is the entire lesson:

    just fuse-capture static1                      # groundtruth.txt
    just fuse-capture sweep2 --trajectory rtabmap.txt   # 6-DoF poses

With export_capture.py's rotation-only poses the result is the
**panorama TSDF** — the denoised ceiling of the live pipeline. Swapping
in RTAB-Map's optimised trajectory (same file format) re-fuses the same
pixels into a rigid room; nothing else changes.

Capture-specific facts (the rest is tsdf.py, shared with fuse_tum.py):
depth PNGs are 1000 units per metre (millimetres), K comes from the
capture's K.txt, and rgb/depth/pose lines align by construction — the
exporter wrote them from the same keyframes, so no association pass.

Defaults differ from the TUM script on purpose: monocular depth is far
noisier than a Kinect, so 2 cm voxels (not 8 mm — finer would model
model error) and a 6 m range (the mapper's max_range reasoning).
"""

import argparse
from pathlib import Path

import numpy as np
from piros2_world.se3 import (
    BASE_FROM_OPTICAL,
    make_transform,
    rotation_from_quaternion,
)
import tsdf

# The static chain camera.launch.py publishes: base_link -> camera_link
# (5 cm up) -> camera_optical_frame (the canonical -90/0/-90). RTAB-Map
# reports base_link poses; right-multiplying by this turns T_w_base into
# the T_w_optical fusion needs.
T_BASE_OPTICAL = make_transform(BASE_FROM_OPTICAL, [0.0, 0.0, 0.05])


def read_trajectory(path):
    """TUM-form `t tx ty tz qx qy qz qw [...]` lines -> [(stamp, T_wc)].

    Extra columns are ignored: rtabmap-report appends a node `id` after
    the quaternion.
    """
    poses = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        t, tx, ty, tz, qx, qy, qz, qw = (
            float(v) for v in line.split()[:8])
        poses.append((t, make_transform(
            rotation_from_quaternion(qx, qy, qz, qw), [tx, ty, tz])))
    return poses


def frame_list(capture, trajectory_file, every, tolerance,
               poses_frame='camera'):
    """(rgb, depth, T_wc) triples; poses matched to keyframes by stamp.

    Nearest-stamp within a tolerance, not exact equality — and after
    removing the *constant offset* between the two clocks: capture
    stamps are the bag's original receive times, while poses from a
    replayed bag are stamped in replay wall time, a shift of days. Both
    series cover the same physical sweep at roughly uniform rates, so
    the difference of medians recovers the shift; per-keyframe jitter
    then falls under the tolerance or the keyframe is skipped aloud —
    odometry can also simply lose frames, and skipping (not
    interpolating) keeps the fusion honest about what was posed.
    """
    poses = read_trajectory(Path(trajectory_file))
    if poses_frame == 'base':
        poses = [(t, t_wb @ T_BASE_OPTICAL) for t, t_wb in poses]
    stamps = np.array([t for t, _ in poses])
    key_stamps = [float(line.split()[0]) for line in
                  (capture / 'rgb.txt').read_text().splitlines()
                  if line.strip() and not line.startswith('#')]
    offset = float(np.median(stamps) - np.median(key_stamps))
    if abs(offset) > 1.0:
        print(f'clock offset between capture and trajectory: '
              f'{offset:.3f} s (removed)')

    frames = []
    skipped = 0
    for line in (capture / 'rgb.txt').read_text().splitlines():
        if line.startswith('#') or not line.strip():
            continue
        stamp, rgb_file = line.split()
        depth_file = rgb_file.replace('rgb/', 'depth/')
        i = int(np.argmin(np.abs(stamps - offset - float(stamp))))
        if abs(stamps[i] - offset - float(stamp)) > tolerance:
            skipped += 1
            continue
        frames.append((capture / rgb_file, capture / depth_file,
                       poses[i][1]))
    if skipped:
        print(f'{skipped}/{skipped + len(frames)} keyframes have no '
              f'pose within {tolerance} s — skipped')
    return frames[::every]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('capture', type=Path,
                        help='capture dir (captures/<name>)')
    parser.add_argument('--trajectory', default='groundtruth.txt',
                        help='TUM-form pose file: a path, or a filename '
                             'inside the capture dir (default: the '
                             'exporter\'s rotation-only groundtruth.txt)')
    parser.add_argument('--poses-frame', choices=('camera', 'base'),
                        default='camera',
                        help='frame the trajectory poses: camera = '
                             'optical (the exporter\'s), base = '
                             'base_link (RTAB-Map\'s rtabmap-report '
                             'output) — converted through the static '
                             'chain')
    parser.add_argument('--tolerance', type=float, default=0.10,
                        help='max stamp mismatch (s) after constant-'
                             'offset removal before a keyframe is '
                             'dropped as unposed')
    parser.add_argument('--voxel-size', type=float, default=0.02)
    parser.add_argument('--trunc-voxels', type=float, default=4.0)
    parser.add_argument('--depth-max', type=float, default=6.0)
    parser.add_argument('--every', type=int, default=1)
    parser.add_argument('--weight-threshold', type=float, default=3.0)
    parser.add_argument('--align', action='store_true',
                        help='per-frame depth scale alignment against '
                             'the running TSDF — the ±4%% wobble fix '
                             '(live mesh plan P2)')
    parser.add_argument('--out', type=Path, default=None)
    args = parser.parse_args()

    k_matrix = np.loadtxt(args.capture / 'K.txt')
    device = tsdf.pick_device()
    trajectory = (Path(args.trajectory) if Path(args.trajectory).exists()
                  else args.capture / args.trajectory)
    frames = frame_list(args.capture, trajectory, args.every,
                        args.tolerance, args.poses_frame)
    if not frames:
        raise SystemExit('no posed frames — empty capture or an empty '
                         'trajectory file')
    vbg, count, elapsed = tsdf.integrate(
        frames, k_matrix, 1000.0, args.voxel_size, args.trunc_voxels,
        args.depth_max, device, align=args.align)
    mesh = tsdf.extract_mesh(vbg, args.weight_threshold)

    label = trajectory.stem + ('_aligned' if args.align else '')
    out = args.out or Path('meshes') / (
        f'{args.capture.name}_{label}_{args.voxel_size * 1000:g}mm.ply')
    tsdf.write_mesh(mesh, out, count, elapsed, vbg.hashmap().size())


if __name__ == '__main__':
    main()
