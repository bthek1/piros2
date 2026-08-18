"""
Wall-flatness numbers for a saved mesh — the todo's "walls stay put" as
a measurement (SLAM plan P3's gate, and the sweep gate's missing metric).

`just mesh-views` shows a doubled wall; this counts it. RANSAC the
dominant planes of the mesh's vertices (Open3D `segment_plane`, the same
call `tools/recon/room_layer.py` uses), then for each plane report the
inlier fraction and the *thickness* of the surface around it: RMS and
p95 of the |distance| of every vertex within `--band` metres of the
plane. A wall that was integrated twice from drifted poses is a thick
plane (two sheets a few centimetres apart); a wall the graph pulled
back together is thin. Prints a table and writes <mesh>_planes.json;
`--compare OTHER.ply` scores a second mesh with the same settings and
prints the paired verdict (the largest plane's p95 thickness must
shrink) — the P3 gate compares the odometry-posed surface against the
graph-corrected one from the same bag.

Perception venv (open3d); run through `just mesh-planes`.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np


def plane_stats(points, max_planes, distance, band, min_inliers):
    import open3d as o3d
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    rest = cloud
    stats = []
    total = len(points)
    for _ in range(max_planes):
        if len(rest.points) < min_inliers:
            break
        (a, b, c, d), inliers = rest.segment_plane(
            distance_threshold=distance, ransac_n=3, num_iterations=1000)
        if len(inliers) < min_inliers:
            break
        normal = np.array([a, b, c])
        # Thickness on the WHOLE mesh, not just RANSAC's inliers: the
        # second sheet of a doubled wall sits outside the inlier band by
        # construction, so measure everything within `band`.
        dist = np.abs(points @ normal + d)
        near = dist[dist < band]
        stats.append({
            'normal': [float(v) for v in normal], 'd': float(d),
            'inliers': int(len(inliers)),
            'inlier_fraction': float(len(inliers) / total),
            'near_points': int(len(near)),
            'rms_m': float(np.sqrt((near ** 2).mean())) if len(near) else None,
            'p95_m': float(np.percentile(near, 95)) if len(near) else None,
            'vertical': bool(abs(normal[2]) < 0.3),
        })
        rest = rest.select_by_index(inliers, invert=True)
    return stats


def load_points(path):
    import open3d as o3d
    mesh = o3d.io.read_triangle_mesh(str(path))
    if mesh.has_triangles():
        return np.asarray(mesh.vertices)
    cloud = o3d.io.read_point_cloud(str(path))
    return np.asarray(cloud.points)


def describe(label, stats):
    print(f'{label}:')
    for k, s in enumerate(stats):
        kind = 'wall ' if s['vertical'] else 'floor/ceil'
        print(f'  plane {k} {kind} n=({s["normal"][0]:+.2f},'
              f'{s["normal"][1]:+.2f},{s["normal"][2]:+.2f}) inliers '
              f'{s["inliers"]} ({s["inlier_fraction"] * 100:.1f}%)  '
              f'thickness rms {s["rms_m"] * 100:.2f} cm  p95 '
              f'{s["p95_m"] * 100:.2f} cm  over {s["near_points"]} pts')


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('mesh')
    p.add_argument('--compare', help='second mesh: verdict = its largest '
                                     'plane is thinner than the first\'s')
    p.add_argument('--max-planes', type=int, default=3)
    p.add_argument('--distance', type=float, default=0.02,
                   help='RANSAC inlier distance (m)')
    p.add_argument('--band', type=float, default=0.15,
                   help='count thickness over vertices within this (m)')
    p.add_argument('--min-inliers', type=int, default=500)
    args = p.parse_args()

    results = {}
    for label, path in (('mesh', args.mesh), ('compare', args.compare)):
        if path is None:
            continue
        pts = load_points(path)
        if len(pts) < args.min_inliers:
            print(f'{path}: only {len(pts)} points')
            sys.exit(2)
        stats = plane_stats(pts, args.max_planes, args.distance, args.band,
                            args.min_inliers)
        results[label] = {'path': path, 'points': int(len(pts)),
                          'planes': stats}
        describe(path, stats)
        Path(path).with_suffix('').with_name(
            Path(path).stem + '_planes.json').write_text(
            json.dumps(results[label], indent=2) + '\n')
    if args.compare is None:
        sys.exit(0 if results['mesh']['planes'] else 1)
    a, b = results['mesh']['planes'], results['compare']['planes']
    if not a or not b:
        print('=== mesh planes: NO DATA (no plane found)')
        sys.exit(2)
    # Pair the compare mesh's plane nearest in normal to the first
    # mesh's largest plane.
    n0 = np.array(a[0]['normal'])
    j = int(np.argmax([abs(np.dot(n0, s['normal'])) for s in b]))
    p95_a, p95_b = a[0]['p95_m'], b[j]['p95_m']
    rms_a, rms_b = a[0]['rms_m'], b[j]['rms_m']
    verdict = 'PASS' if p95_b < p95_a and rms_b <= rms_a * 1.05 else 'FAIL'
    print(f'=== mesh planes: {verdict} — largest plane thickness p95 '
          f'{p95_a * 100:.2f} → {p95_b * 100:.2f} cm, rms '
          f'{rms_a * 100:.2f} → {rms_b * 100:.2f} cm ===')
    sys.exit(0 if verdict == 'PASS' else 1)


if __name__ == '__main__':
    main()
