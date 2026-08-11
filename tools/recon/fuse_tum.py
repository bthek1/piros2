"""
Fuse a TUM RGB-D sequence into a TSDF and extract a mesh.

World fusion plan P1 — the Open3D tutorial, made ours. Runs under the
perception venv (open3d is PyPI-only, same escape hatch as onnxruntime)
with the workspace overlay sourced, so `piros2_world.se3` is importable:

    just fuse-tum [sequence] [-- extra args]

The fusion itself lives in tsdf.py (shared with fuse_capture.py since
P4); what is TUM-specific here:

- The freiburg1 intrinsics and the 5000-units-per-metre depth PNGs.
- Association: rgb, depth and groundtruth run on three unsynchronised
  clocks, paired by nearest timestamp within a tolerance — their own
  recipe.
- groundtruth.txt stores T_wc (camera pose in world); Open3D wants the
  inverse, which tsdf.integrate takes care of via se3.invert.
"""

import argparse
from pathlib import Path

import numpy as np
from piros2_world.se3 import make_transform, rotation_from_quaternion
import tsdf

# Camera intrinsics for the freiburg1 sequences, from the TUM benchmark
# page — a calibrated K, the luxury our own captures don't have yet.
FR1_K = np.array([[517.3, 0.0, 318.6],
                  [0.0, 516.5, 255.3],
                  [0.0, 0.0, 1.0]])
TUM_DEPTH_SCALE = 5000.0  # depth PNG units per metre


def read_stamped_list(path):
    """Parse a TUM list file: lines of `timestamp value...`, # comments."""
    entries = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        fields = line.split()
        entries.append((float(fields[0]), fields[1:]))
    return entries


def associate(reference, candidates, tolerance):
    """
    For each (t, data) in reference, the nearest candidate within tol.

    The TUM association recipe: three sensors, three clocks, nearest
    neighbour in time. Returns list of (ref_data, cand_data) pairs;
    reference entries with no candidate close enough are dropped.
    """
    times = np.array([t for t, _ in candidates])
    pairs = []
    for t, data in reference:
        i = int(np.argmin(np.abs(times - t)))
        if abs(times[i] - t) <= tolerance:
            pairs.append((data, candidates[i][1]))
    return pairs


def pose_from_groundtruth(fields):
    """TUM groundtruth `tx ty tz qx qy qz qw` -> T_wc as a 4x4."""
    tx, ty, tz, qx, qy, qz, qw = (float(f) for f in fields)
    return make_transform(rotation_from_quaternion(qx, qy, qz, qw),
                          [tx, ty, tz])


def frame_list(sequence, every):
    """Associated (rgb_path, depth_path, T_wc) triples, every Nth."""
    rgb_list = read_stamped_list(sequence / 'rgb.txt')
    depth_list = read_stamped_list(sequence / 'depth.txt')
    gt_list = read_stamped_list(sequence / 'groundtruth.txt')

    rgb_depth = associate(rgb_list, depth_list, tolerance=0.02)
    rgb_stamp = {tuple(files): t for t, files in rgb_list}
    frames = []
    for rgb_file, depth_file in rgb_depth:
        t = rgb_stamp[tuple(rgb_file)]
        gt = associate([(t, rgb_file)], gt_list, tolerance=0.02)
        if gt:
            frames.append((sequence / rgb_file[0], sequence / depth_file[0],
                           pose_from_groundtruth(gt[0][1])))
    return frames[::every]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('sequence', type=Path,
                        help='TUM sequence dir (has rgb.txt, depth.txt, '
                             'groundtruth.txt)')
    parser.add_argument('--voxel-size', type=float, default=0.008,
                        help='metres; 4-8 mm is the room range, below 4 '
                             'models sensor noise (default 0.008)')
    parser.add_argument('--trunc-voxels', type=float, default=4.0,
                        help='truncation band as a multiple of voxel size '
                             '(default 4)')
    parser.add_argument('--depth-max', type=float, default=3.0,
                        help='metres; depth beyond this is ignored')
    parser.add_argument('--every', type=int, default=1,
                        help='keep every Nth frame (default 1 = all)')
    parser.add_argument('--weight-threshold', type=float, default=3.0,
                        help='min observations for a voxel to mesh')
    parser.add_argument('--out', type=Path, default=None,
                        help='output mesh path (default '
                             'meshes/<sequence>_<voxel>mm.ply)')
    args = parser.parse_args()

    device = tsdf.pick_device()
    frames = frame_list(args.sequence, args.every)
    vbg, count, elapsed = tsdf.integrate(
        frames, FR1_K, TUM_DEPTH_SCALE, args.voxel_size, args.trunc_voxels,
        args.depth_max, device)
    mesh = tsdf.extract_mesh(vbg, args.weight_threshold)

    out = args.out or Path('meshes') / (
        f'{args.sequence.name}_{args.voxel_size * 1000:g}mm.ply')
    tsdf.write_mesh(mesh, out, count, elapsed, vbg.hashmap().size())


if __name__ == '__main__':
    main()
