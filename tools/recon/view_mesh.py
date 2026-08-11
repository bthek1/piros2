"""
Open a fused mesh in Open3D's interactive viewer.

The meshes are offline artifacts — `just world` deliberately never
publishes them (the live session's job is the dashboard; heavy
reconstruction stays offline) — so this window is how you look at one:

    just view-mesh [meshes/<name>.ply]

Same display caveat as every GUI in this repo: the dev-box session is
Wayland and Open3D's viewer needs X11 — the recipe unsets
WAYLAND_DISPLAY *and* sets XDG_SESSION_TYPE=x11 (GLFW keeps choosing
Wayland on the first alone) so it runs through Xwayland
(docs/info/troubleshooting.md).
"""

import sys

import open3d as o3d


def main():
    path = sys.argv[1]
    mesh = o3d.io.read_triangle_mesh(path)
    if not mesh.has_vertices():
        raise SystemExit(f'no vertices read from {path} — wrong path?')
    mesh.compute_vertex_normals()
    print(f'{len(mesh.vertices)} vertices — drag to orbit, '
          'scroll to zoom, q to quit')
    o3d.visualization.draw_geometries(
        [mesh], window_name=path, width=1280, height=800)


if __name__ == '__main__':
    main()
