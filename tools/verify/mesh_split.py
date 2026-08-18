"""
Does the surface follow the graph? Score it from the mesher's frame memory.

SLAM plan P3's gate. A loop bag (`make_gate_bag.py loop`) plays a sweep
out and back, so its second half re-observes exactly what its first half
saw. Integrate the two halves into *separate* TSDF volumes and the two
surfaces should coincide — and they only can if the poses agree. This
script does that twice from `live_<stamp>_frames.npz` (tsdf_mesher's
`~/save` with `save_frames`): once at the raw odometry poses each frame
was captured with, once at the graph-corrected poses the mesher's
rebuild applied — and measures how far the BACK surface sits from the
OUT surface (median / p90 nearest-neighbour distance of BACK vertices to
OUT vertices, both ways). PASS when the corrected pair sits closer than
the odometry pair: the doubled wall pulled together, as a number
instead of a plane RANSAC that this close-range scene defeats.

Perception venv (open3d, CUDA if present); run through `just gate-mesh`.
"""

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np


def load_frames(path):
    with np.load(path, allow_pickle=False) as d:
        data = {k: d[k] for k in d.files}
    offsets = data['jpeg_offsets']
    jpegs = [data['jpeg_bytes'][offsets[i]:offsets[i + 1]]
             for i in range(len(offsets) - 1)]
    return data, jpegs


def integrate(o3d, frames, poses, k_matrix, factor, voxel, depth_max,
              device):
    vbg = o3d.t.geometry.VoxelBlockGrid(
        attr_names=('tsdf', 'weight'),
        attr_dtypes=(o3d.core.float32, o3d.core.float32),
        attr_channels=((1), (1)), voxel_size=voxel, block_resolution=8,
        block_count=40000, device=device)
    k = k_matrix.copy()
    k[:2] /= factor
    intrinsic = o3d.core.Tensor(k, o3d.core.float64)
    for depth_u16, pose in zip(frames, poses):
        depth = o3d.t.geometry.Image(o3d.core.Tensor(
            np.ascontiguousarray(depth_u16.astype(np.float32) / 1000.0))
        ).to(device)
        extrinsic = o3d.core.Tensor(np.linalg.inv(pose), o3d.core.float64)
        blocks = vbg.compute_unique_block_coordinates(
            depth, intrinsic, extrinsic, 1.0, depth_max)
        vbg.integrate(blocks, depth, intrinsic, extrinsic, 1.0, depth_max)
    cloud = vbg.extract_point_cloud(weight_threshold=2.0).to_legacy()
    return np.asarray(cloud.points)


def surface_gap(o3d, a, b):
    """Nearest-neighbour distances a→b and b→a (median, p90)."""
    if len(a) == 0 or len(b) == 0:
        return None
    pa, pb = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
    pa.points = o3d.utility.Vector3dVector(a)
    pb.points = o3d.utility.Vector3dVector(b)
    ab = np.asarray(pa.compute_point_cloud_distance(pb))
    ba = np.asarray(pb.compute_point_cloud_distance(pa))
    both = np.concatenate([ab, ba])
    return {'median_m': float(np.median(both)),
            'p90_m': float(np.percentile(both, 90)),
            'mean_m': float(both.mean()),
            'points_a': int(len(a)), 'points_b': int(len(b))}


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('frames', help='live_<stamp>_frames.npz from ~/save')
    p.add_argument('--split', type=float, default=0.5,
                   help='fraction of the stamp span that ends OUT (0.5 = '
                        'a palindrome bag)')
    p.add_argument('--voxel', type=float, default=0.02)
    p.add_argument('--depth-max', type=float, default=6.0)
    p.add_argument('--out', help='directory for the json')
    args = p.parse_args()

    import open3d as o3d  # slow import, keep --help fast
    device = o3d.core.Device(
        'CUDA:0' if o3d.core.cuda.is_available() else 'CPU:0')
    data, _ = load_frames(args.frames)
    stamps = data['stamp_ns']
    n = len(stamps)
    if n < 4:
        print(f'=== mesh split: NO DATA ({n} frames)')
        return 2
    cut = stamps[0] + args.split * (stamps[-1] - stamps[0])
    out_idx = np.where(stamps <= cut)[0]
    back_idx = np.where(stamps > cut)[0]
    factor = int(data['factor'][0])
    depth = data['depth_u16']
    odom = data['t_odom_optical']
    corrected = np.array([a @ t for a, t in
                          zip(data['applied'], data['t_odom_optical'])])
    results = {'frames': int(n), 'out_frames': int(len(out_idx)),
               'back_frames': int(len(back_idx))}
    for label, poses in (('odom', odom), ('corrected', corrected)):
        surf_out = integrate(o3d, depth[out_idx], poses[out_idx],
                             data['k_matrix'], factor, args.voxel,
                             args.depth_max, device)
        surf_back = integrate(o3d, depth[back_idx], poses[back_idx],
                              data['k_matrix'], factor, args.voxel,
                              args.depth_max, device)
        gap = surface_gap(o3d, surf_out, surf_back)
        results[label] = gap
        if gap is None:
            print(f'{label}: a half meshed to nothing')
            continue
        print(f'{label:<10} OUT {gap["points_a"]} pts vs BACK '
              f'{gap["points_b"]} pts: gap median {gap["median_m"] * 100:.2f} '
              f'cm  p90 {gap["p90_m"] * 100:.2f} cm  mean '
              f'{gap["mean_m"] * 100:.2f} cm')
    if results['odom'] is None or results['corrected'] is None:
        print('=== mesh split: NO DATA')
        return 2
    o, c = results['odom'], results['corrected']
    verdict = ('PASS' if c['median_m'] < o['median_m']
               and c['p90_m'] < o['p90_m'] else 'FAIL')
    results['verdict'] = verdict
    print(f'=== mesh split: {verdict} — OUT-vs-BACK surface gap median '
          f'{o["median_m"] * 100:.2f} → {c["median_m"] * 100:.2f} cm, p90 '
          f'{o["p90_m"] * 100:.2f} → {c["p90_m"] * 100:.2f} cm ===')
    if args.out:
        Path(args.out).mkdir(parents=True, exist_ok=True)
        (Path(args.out) / 'mesh_split.json').write_text(
            json.dumps(results, indent=2) + '\n')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
