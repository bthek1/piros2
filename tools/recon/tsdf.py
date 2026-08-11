"""
Shared TSDF integration for the recon scripts (world fusion plan P1/P4).

One `VoxelBlockGrid` loop serves both data sources — TUM sequences
(fuse_tum.py) and our own exported captures (fuse_capture.py) — because
the fusion genuinely is the same operation; only the frame list, the
intrinsics and the depth units differ. Both scripts stay thin frontends,
which is itself the lesson: capture formats vary, fusion doesn't.
"""

from pathlib import Path
import time

import numpy as np
import open3d as o3d
import open3d.core as o3c
from piros2_world.depth_align import ScaleAligner
from piros2_world.se3 import invert


def integrate(frames, k_matrix, depth_scale, voxel_size, trunc_voxels,
              depth_max, device, block_count=100000, align=False):
    """
    Fuse (rgb_path, depth_path, T_wc) frames into a VoxelBlockGrid.

    depth_scale is the PNG's units-per-metre (TUM: 5000, our exports:
    1000). Open3D's extrinsic maps world points into the camera — T_cw —
    so each pose goes through se3.invert, subscripts doing the guarding.

    align=True applies per-frame depth scale alignment (live mesh plan
    P2 — the neural depth's ±4% wobble fix): each frame after the first
    ray-casts the TSDF from its own pose and conforms by the median
    ratio. The aligned path works in float metres (integrate()'s dtype
    rule pairs float depth with float colour); returns the applied
    scales for reporting.
    """
    vbg = o3d.t.geometry.VoxelBlockGrid(
        attr_names=('tsdf', 'weight', 'color'),
        attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
        attr_channels=((1), (1), (3)),
        voxel_size=voxel_size,
        block_resolution=8,
        block_count=block_count,
        device=device)
    intrinsic = o3c.Tensor(k_matrix, o3c.float64)

    def render_expected(metres, extrinsic):
        probe = o3d.t.geometry.Image(o3c.Tensor(metres)).to(device)
        blocks = vbg.compute_unique_block_coordinates(
            probe, intrinsic, extrinsic, 1.0, depth_max,
            trunc_voxel_multiplier=trunc_voxels)
        cast = vbg.ray_cast(
            blocks, intrinsic, extrinsic,
            metres.shape[1], metres.shape[0],
            render_attributes=['depth'], depth_scale=1.0,
            depth_min=0.1, depth_max=depth_max,
            weight_threshold=3.0)
        return cast['depth'].cpu().numpy().reshape(metres.shape)

    start = time.perf_counter()
    count = 0
    scales = []
    aligner = ScaleAligner()
    for rgb_path, depth_path, t_wc in frames:
        depth = o3d.t.io.read_image(str(depth_path)).to(device)
        color = o3d.t.io.read_image(str(rgb_path)).to(device)
        extrinsic = o3c.Tensor(invert(t_wc), o3c.float64)
        scale = depth_scale
        if align:
            # Work in metres so the wobble correction multiplies in.
            metres = depth.as_tensor().cpu().numpy().astype(
                np.float32).squeeze() / depth_scale
            if count:
                factor, _ = aligner.scale_for(
                    render_expected(metres, extrinsic), metres)
                metres = metres * np.float32(factor)
                scales.append(factor)
            depth = o3d.t.geometry.Image(o3c.Tensor(metres)).to(device)
            color = o3d.t.geometry.Image(o3c.Tensor(
                color.as_tensor().cpu().numpy().astype(
                    np.float32) / 255.0)).to(device)
            scale = 1.0
        blocks = vbg.compute_unique_block_coordinates(
            depth, intrinsic, extrinsic, scale, depth_max,
            trunc_voxel_multiplier=trunc_voxels)
        vbg.integrate(blocks, depth, color, intrinsic, intrinsic,
                      extrinsic, scale, depth_max,
                      trunc_voxel_multiplier=trunc_voxels)
        count += 1
    if scales:
        arr = np.array(scales)
        print(f'align: scale {arr.mean():.3f} ± {arr.std():.3f} over '
              f'{len(arr)} frames')
    return vbg, count, time.perf_counter() - start


def extract_mesh(vbg, weight_threshold):
    """Marching cubes over the zero crossing, CPU fallback on OOM.

    Extraction allocates a dense assistance structure over the active
    blocks — at small voxels that spike outgrows the GPU even when
    integration fit (measured in P1). One-shot work, so the CPU copy
    costs seconds, not the run.
    """
    try:
        return vbg.extract_triangle_mesh(weight_threshold=weight_threshold)
    except RuntimeError:
        print('GPU marching cubes ran out of memory; retrying on CPU')
        return vbg.cpu().extract_triangle_mesh(
            weight_threshold=weight_threshold)


def pick_device():
    device = o3c.Device(
        'CUDA:0' if o3d.core.cuda.is_available() else 'CPU:0')
    print(f'device: {device} (open3d {o3d.__version__})')
    return device


def write_mesh(mesh, out_path, frame_count, elapsed, block_count):
    """Save as PLY and print the run's numbers."""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    legacy = mesh.to_legacy()
    o3d.io.write_triangle_mesh(str(out_path), legacy)
    extent = (np.asarray(legacy.get_max_bound())
              - np.asarray(legacy.get_min_bound()))
    print(f'{frame_count} frames integrated in {elapsed:.1f} s '
          f'({elapsed / max(frame_count, 1) * 1000:.0f} ms/frame)')
    print(f'{block_count} voxel blocks allocated '
          f'(~{block_count * 8 ** 3 * 20 / 1e6:.0f} MB if dense in-band)')
    print(f'mesh: {len(legacy.vertices)} vertices, '
          f'{len(legacy.triangles)} triangles')
    print(f'bounding box: {extent[0]:.2f} x {extent[1]:.2f} '
          f'x {extent[2]:.2f} m')
    print(f'wrote {out_path}')
    return legacy
