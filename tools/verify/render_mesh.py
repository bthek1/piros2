"""
Render a saved mesh from fixed viewpoints to PNG — the screenshot nobody
has to take.

`just view-mesh` opens an interactive window a person rotates; this
writes the views a check needs, deterministically, so a saved surface
(`just mesh-save`, `just fuse-capture`) can be inspected after the fact
by anyone — or anything — that can open a PNG:

    <out>/
    ├── origin.png   # from the camera's start pose (odom origin, +x
    │                #   forward, z up): what the operator saw
    ├── top.png      # plan view straight down: walls as lines — the
    │                #   view that shows doubling, smearing, drift
    ├── oblique.png  # three-quarter view from above-behind
    └── sheet.png    # the three side by side, labelled

Uses Open3D's OffscreenRenderer (Filament, EGL) under the perception
venv — no window, no Wayland pin needed. Run through `just mesh-views`.
"""

import argparse
from pathlib import Path

import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('mesh')
    p.add_argument('out')
    p.add_argument('--size', type=int, nargs=2, default=(1280, 800),
                   metavar=('W', 'H'))
    p.add_argument('--fov', type=float, default=70.0)
    args = p.parse_args()

    import open3d as o3d  # slow import, keep --help fast
    mesh = o3d.io.read_triangle_mesh(args.mesh)
    if not mesh.has_triangles():
        raise SystemExit(f'{args.mesh}: no triangles')
    mesh.compute_vertex_normals()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    bbox = mesh.get_axis_aligned_bounding_box()
    centre = bbox.get_center()
    extent = float(bbox.get_extent().max())
    w, h = args.size
    renderer = o3d.visualization.rendering.OffscreenRenderer(w, h)
    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = 'defaultLit'
    renderer.scene.add_geometry('mesh', mesh, material)
    renderer.scene.set_background([1.0, 1.0, 1.0, 1.0])
    renderer.scene.scene.set_sun_light([-0.3, -0.3, -1.0], [1, 1, 1], 60000)
    renderer.scene.scene.enable_sun_light(True)

    # Frame conventions: meshes are saved in odom (REP-103: x forward,
    # y left, z up; the camera started at the origin looking +x).
    views = {
        'origin': (np.array([0.0, 0.0, 0.0]), centre, [0, 0, 1]),
        'top': (centre + np.array([0, 0, extent * 1.0]), centre, [1, 0, 0]),
        'oblique': (centre + np.array([-extent * 0.7, -extent * 0.6, extent * 0.6]),
                    centre, [0, 0, 1]),
    }
    files = []
    for name, (eye, look_at, up) in views.items():
        renderer.setup_camera(args.fov, look_at, eye, up)
        image = renderer.render_to_image()
        path = out / f'{name}.png'
        o3d.io.write_image(str(path), image)
        files.append((name, path))
        print(f'{name}: {path}')

    try:
        import cv2
        tiles = []
        for name, path in files:
            tile = cv2.imread(str(path))
            tile = cv2.resize(tile, (w // 2, h // 2))
            cv2.putText(tile, name, (12, 36), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 0, 255), 2)
            tiles.append(tile)
        cv2.imwrite(str(out / 'sheet.png'), np.hstack(tiles))
        print(f'sheet: {out / "sheet.png"}  '
              f'({len(mesh.triangles)} triangles, extent {extent:.2f} m)')
    except ImportError:
        pass


if __name__ == '__main__':
    main()
