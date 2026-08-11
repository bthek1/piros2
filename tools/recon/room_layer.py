"""
Extract the structural layer from a fused mesh: planes -> room.json.

World fusion plan P5 — the argument that a mesh alone isn't a *room
model*. RANSAC finds the dominant planes, gravity comes from the floor
normal (the no-IMU fallback), walls snap to a Manhattan frame, and the
result is a JSON layer you can measure and edit, plus the mesh as GLB:

    just room-layer meshes/<name>.ply

The concepts, in the order they run:

- RANSAC (Open3D's segment_plane): sample 3 points, fit the plane
  (n, d) with n·p + d = 0, count inliers within a distance, repeat, keep
  the best — the principled version of the keypoint estimator's
  reject-worst-and-refit rounds. Iterating it (find, remove inliers,
  find again) peels the scene one dominant plane at a time.
- Gravity alignment: the largest near-horizontal plane is taken as the
  floor and the model rotated so its normal is exactly +Z. World frames
  from a mocap rig (TUM) are already close; frames from our captures
  (first keyframe's optical frame) are not — the floor plane is the
  gravity reference either way.
- Manhattan snap: indoor walls cluster around two orthogonal
  directions. The dominant wall azimuth is estimated (mod 90 deg), the
  frame yawed onto it, and each wall normal snapped to the nearest
  axis; offsets refit from inliers. Distances between opposite snapped
  walls become measurable numbers — the point of the whole plan.
- The JSON schema is info.md's: up/units/handedness, planes with
  boundary polygons and labels, openings and objects left as honest
  empty arrays (named later work, not silently missing).
"""

import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d


def segment_planes(cloud, max_planes, distance, min_inliers):
    """Peel dominant planes off a point cloud, RANSAC each time."""
    planes = []
    rest = cloud
    for _ in range(max_planes):
        if len(rest.points) < min_inliers:
            break
        (a, b, c, d), inliers = rest.segment_plane(
            distance_threshold=distance, ransac_n=3, num_iterations=1000)
        if len(inliers) < min_inliers:
            break
        normal = np.array([a, b, c])
        # Keep n unit-length with d in metres: n.p + d = 0.
        scale = np.linalg.norm(normal)
        planes.append({'n': normal / scale, 'd': d / scale,
                       'points': np.asarray(
                           rest.select_by_index(inliers).points)})
        rest = rest.select_by_index(inliers, invert=True)
    return planes


def classify(plane, up):
    dot = abs(plane['n'] @ up)
    if dot > 0.85:
        return 'horizontal'
    if dot < 0.25:
        return 'wall'
    return 'slab'


def convex_hull_2d(points_2d):
    """Andrew's monotone chain: Nx2 points -> hull corners, CCW order."""
    pts = np.unique(np.round(points_2d, 4), axis=0)
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]
    if len(pts) <= 2:
        return pts

    def half(iterable):
        chain = []
        for p in iterable:
            while len(chain) >= 2 and np.cross(
                    chain[-1] - chain[-2], p - chain[-2]) <= 0:
                chain.pop()
            chain.append(p)
        return chain[:-1]

    return np.array(half(pts) + half(pts[::-1]))


def boundary_polygon(points, n, d):
    """Inliers -> a 2D-hull boundary polygon, back in 3D coordinates."""
    # Basis in the plane: two vectors orthogonal to n.
    u = np.cross(n, [0.0, 0.0, 1.0])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(n, [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    corners = convex_hull_2d(np.column_stack([points @ u, points @ v]))
    return [(float(x), float(y), float(z)) for x, y, z in
            (np.outer(corners[:, 0], u) + np.outer(corners[:, 1], v)
             - d * n)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('mesh', type=Path)
    parser.add_argument('--max-planes', type=int, default=8)
    parser.add_argument('--distance', type=float, default=0.03,
                        help='RANSAC inlier distance, metres (default '
                             '0.03 — matches ~2 cm voxels + mono noise)')
    parser.add_argument('--min-inliers', type=int, default=2000)
    parser.add_argument('--out-dir', type=Path, default=None,
                        help='default: alongside the mesh')
    args = parser.parse_args()

    mesh = o3d.io.read_triangle_mesh(str(args.mesh))
    cloud = o3d.geometry.PointCloud(mesh.vertices)
    print(f'{len(cloud.points)} vertices from {args.mesh}')

    planes = segment_planes(cloud, args.max_planes, args.distance,
                            args.min_inliers)
    if not planes:
        raise SystemExit('no planes found — mesh too small or too noisy '
                         'for the current --min-inliers')

    # Gravity: the floor is the horizontal plane with the SCENE ABOVE
    # it — not simply the largest one. In fr1/desk the desk surface
    # out-inliers the real floor, but half the scene lies below a desk
    # and none lies below a floor, so the above-fraction separates
    # them. First pass classifies against the file's +Z; if nothing is
    # horizontal (our captures' optical-frame worlds), fall back to the
    # largest plane and say so.
    all_points = np.asarray(cloud.points)
    for p in planes:
        signed = all_points @ p['n'] + p['d']
        frac = float(np.mean(signed > -args.distance))
        if frac < 0.5:                     # flip n toward the bulk
            p['n'], p['d'], frac = -p['n'], -p['d'], 1.0 - frac
        p['above'] = frac
    horizontal = [p for p in planes
                  if classify(p, np.array([0., 0., 1.])) == 'horizontal']
    grounded = [p for p in horizontal if p['above'] > 0.9]
    if grounded:
        floor = max(grounded, key=lambda p: len(p['points']))
    else:
        floor = max(horizontal or planes, key=lambda p: len(p['points']))
        print('no horizontal plane with the scene above it — treating '
              'the largest plane as the floor; check the result')
    up = floor['n']

    # Rotate the model so up -> +Z exactly (Rodrigues via cross/dot).
    z = np.array([0., 0., 1.])
    axis = np.cross(up, z)
    s, c = np.linalg.norm(axis), float(up @ z)
    if s > 1e-9:
        k = axis / s
        kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]],
                       [-k[1], k[0], 0]])
        r_align = np.eye(3) + s * kx + (1 - c) * (kx @ kx)
    else:
        r_align = np.eye(3)

    # Manhattan yaw from the dominant wall azimuth (mod 90 deg).
    walls = [p for p in planes if classify(p, up) == 'wall']
    if walls:
        azimuths = [np.arctan2(*(r_align @ p['n'])[[1, 0]]) % (np.pi / 2)
                    for p in walls]
        weights = [len(p['points']) for p in walls]
        yaw = float(np.average(azimuths, weights=weights))
        cz, sz = np.cos(-yaw), np.sin(-yaw)
        r_align = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ r_align

    floor_height = float(
        np.mean((floor['points'] @ r_align.T)[:, 2]))
    out_planes = []
    for i, plane in enumerate(planes):
        n = r_align @ plane['n']
        pts = plane['points'] @ r_align.T
        label = classify(plane, up)
        if label == 'horizontal':
            height = float(np.mean(pts[:, 2])) - floor_height
            n = z if n[2] > 0 else -z
            label = ('floor' if plane is floor else
                     'ceiling' if height > 1.5 else 'horizontal')
        elif label == 'wall':
            axis_dir = np.argmax(np.abs(n[:2]))
            snapped = np.zeros(3)
            snapped[axis_dir] = np.sign(n[axis_dir])
            n = snapped
        d = float(-np.mean(pts @ n))
        out_planes.append({
            'id': f'{label}_{i}', 'label': label,
            'n': [round(float(v), 6) for v in n], 'd': round(d, 4),
            'inliers': len(pts),
            'boundary': [[round(v, 3) for v in corner] for corner in
                         boundary_polygon(pts, n, d)]})
        print(f'{label}_{i}: n=({n[0]:+.2f},{n[1]:+.2f},{n[2]:+.2f}) '
              f'd={d:+.3f} inliers={len(pts)}')

    # Measurable spans: opposite parallel planes, the tape-measure hook.
    for axis_i, name in ((0, 'x-walls'), (1, 'y-walls'), (2, 'floor-ceiling')):
        ds = [p['d'] for p in out_planes
              if abs(p['n'][axis_i]) > 0.9]
        if len(ds) >= 2:
            print(f'{name} span: {max(ds) - min(ds):.3f} m')

    out_dir = args.out_dir or args.mesh.parent
    room = {'up': [0, 0, 1], 'units': 'm', 'handedness': 'right',
            'source_mesh': args.mesh.name,
            'planes': out_planes, 'openings': [], 'objects': []}
    room_path = out_dir / f'{args.mesh.stem}_room.json'
    room_path.write_text(json.dumps(room, indent=2) + '\n')

    mesh.rotate(r_align, center=(0, 0, 0))
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    glb_path = out_dir / f'{args.mesh.stem}.glb'
    o3d.io.write_triangle_mesh(str(glb_path), mesh)
    print(f'wrote {room_path} and {glb_path}')


if __name__ == '__main__':
    main()
