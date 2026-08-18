# Copyright 2026 Benedict Thekkel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Unit tests for mesh completion: pure numpy, no open3d, no graph.

The fixtures are the mesh-completion plan's acceptance cases made
executable: a punched plane comes back closed with interpolated colour,
a box missing one face keeps that face open (it is the component's
frontier), debris islands get pruned rather than capped, oversized
loops respect the radius guard, and the debug tint marks exactly the
assumed geometry.
"""

import numpy as np
from piros2_world_mesh.mesh_fill import (
    boundary_loops,
    complete_mesh,
    fill_interior_holes,
    prune_small_components,
)


def grid_plane(n=6, hole=None):
    """
    Build a unit-spaced n×n grid plane in z=0.

    Optionally the four triangles around one interior vertex are
    removed — a punched hole. Returns (vertices, triangles, colours); colours encode the x
    coordinate so interpolation is checkable.
    """
    xx, yy = np.meshgrid(np.arange(n, dtype=float), np.arange(n, dtype=float))
    vertices = np.stack([xx.ravel(), yy.ravel(), np.zeros(n * n)], axis=1)
    triangles = []
    for r in range(n - 1):
        for c in range(n - 1):
            i = r * n + c
            triangles.append((i, i + 1, i + n))
            triangles.append((i + 1, i + n + 1, i + n))
    triangles = np.array(triangles)
    if hole is not None:
        keep = ~np.any(triangles == hole, axis=1)
        triangles = triangles[keep]
    colours = np.stack([vertices[:, 0] / n,
                        np.zeros(n * n), np.zeros(n * n)], axis=1)
    return vertices, triangles, colours


def open_box():
    """Build a unit cube missing its top face: one loop, no holes."""
    v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                  [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], dtype=float)
    t = np.array([
        [0, 2, 1], [0, 3, 2],              # bottom
        [0, 1, 5], [0, 5, 4],              # front
        [1, 2, 6], [1, 6, 5],              # right
        [2, 3, 7], [2, 7, 6],              # back
        [3, 0, 4], [3, 4, 7],              # left
    ])
    c = np.full((8, 3), 0.5)
    return v, t, c


def interior_loop_count(vertices, triangles):
    """Boundary loops minus one frontier per connected component."""
    loops = boundary_loops(triangles)
    if not loops:
        return 0
    # cheap component id: flood via shared vertices of loop starts is
    # overkill here — the fixtures are single-component by construction.
    return len(loops) - 1


def test_punched_plane_is_closed_with_interpolated_colour():
    hole_vertex = 2 * 6 + 3  # interior vertex (row 2, col 3)
    v, t, c = grid_plane(6, hole=hole_vertex)
    assert interior_loop_count(v, t) == 1  # the punch is a real hole

    v2, t2, c2, filled = fill_interior_holes(v, t, c, max_hole_radius=5.0)
    assert filled == 1
    assert interior_loop_count(v2, t2) == 0  # only the frontier remains
    # The patch centroid sits inside the hole and takes the ring's mean
    # colour — the "assumed from surrounding detail" contract.
    centroid, colour = v2[-1], c2[-1]
    assert 2.0 < centroid[0] < 4.0 and 1.0 < centroid[1] < 3.0
    loops = boundary_loops(t)
    ring = min(loops, key=lambda lp: _radius(v, lp))  # the hole, not the rim
    assert np.allclose(colour, c[ring].mean(axis=0))


def _radius(vertices, loop):
    pts = vertices[loop]
    return np.linalg.norm(pts - pts.mean(axis=0), axis=1).max()


def test_box_missing_face_keeps_its_frontier_open():
    v, t, c = open_box()
    loops = boundary_loops(t)
    assert len(loops) == 1  # the missing top is the only loop

    v2, t2, c2, filled = fill_interior_holes(v, t, c, max_hole_radius=5.0)
    assert filled == 0  # the largest (only) loop is the frontier
    assert len(t2) == len(t)


def test_radius_guard_leaves_wide_holes_open():
    hole_vertex = 2 * 6 + 3
    v, t, c = grid_plane(6, hole=hole_vertex)
    # The punched hole has radius ~1; a guard below that must refuse it.
    _, t2, _, filled = fill_interior_holes(v, t, c, max_hole_radius=0.5)
    assert filled == 0
    assert len(t2) == len(t)


def test_debris_components_are_pruned_not_capped():
    v, t, c = grid_plane(4)
    # A floating two-triangle flake far from the plane.
    flake_v = np.array([[10.0, 10, 0], [11, 10, 0], [10, 11, 0],
                        [11, 11, 0]])
    flake_t = np.array([[0, 1, 2], [1, 3, 2]]) + len(v)
    v = np.vstack([v, flake_v])
    t = np.vstack([t, flake_t])
    c = np.vstack([c, np.full((4, 3), 0.9)])

    v2, t2, c2, pruned = prune_small_components(v, t, c, min_triangles=3)
    assert pruned == 1
    assert len(t2) == len(t) - 2
    assert len(v2) == len(v) - 4  # flake vertices gone, plane reindexed
    assert t2.max() < len(v2)  # indices stay dense and valid


def test_tint_marks_exactly_the_assumed_geometry():
    hole_vertex = 2 * 6 + 3
    v, t, c = grid_plane(6, hole=hole_vertex)
    v2, t2, c2, filled = fill_interior_holes(
        v, t, c, max_hole_radius=5.0, tint=(1.0, 0.0, 1.0))
    assert filled == 1
    assert np.allclose(c2[-1], (1.0, 0.0, 1.0))  # the patch vertex
    assert np.allclose(c2[:len(c)], c)           # nothing else touched


def test_complete_mesh_prunes_then_fills():
    hole_vertex = 2 * 6 + 3
    v, t, c = grid_plane(6, hole=hole_vertex)
    flake_v = np.array([[10.0, 10, 0], [11, 10, 0], [10, 11, 0]])
    flake_t = np.array([[0, 2, 1]]) + len(v)
    v = np.vstack([v, flake_v])
    t = np.vstack([t, flake_t])
    c = np.vstack([c, np.full((3, 3), 0.9)])

    v2, t2, c2, stats = complete_mesh(
        v, t, c, min_component_triangles=3, max_hole_radius=5.0)
    assert stats == {'pruned': 1, 'filled': 1}
    assert interior_loop_count(v2, t2) == 0


def test_pinched_holes_are_still_filled():
    # Two punched holes whose rings share one vertex — the non-manifold
    # pinch that marching-cubes surfaces produce. A vertex-keyed walk
    # drops one of the loops silently (found live 2026-08-18); the
    # edge-keyed walk must close both.
    v, t, c = grid_plane(7, hole=2 * 7 + 2)
    keep = ~np.any(t == 4 * 7 + 4, axis=1)  # second punch at (4,4)
    t = t[keep]
    assert len(boundary_loops(t)) >= 2  # frontier + at least one hole

    v2, t2, c2, filled = fill_interior_holes(v, t, c, max_hole_radius=5.0)
    assert filled >= 1
    # Whatever the walk fused, only the frontier may remain open.
    assert len(boundary_loops(t2)) == 1


def test_fill_preserves_winding_orientation():
    hole_vertex = 2 * 6 + 3
    v, t, c = grid_plane(6, hole=hole_vertex)
    v2, t2, _, _ = fill_interior_holes(v, t, c, max_hole_radius=5.0)
    # All original normals point +z; a correctly-wound patch must too.
    patch = t2[len(t):]
    a, b, cc = v2[patch[:, 0]], v2[patch[:, 1]], v2[patch[:, 2]]
    normals = np.cross(b - a, cc - a)
    assert (normals[:, 2] > 0).all()


def test_completion_is_idempotent():
    # Measured live 2026-08-18 (a completed save re-completes to a
    # no-op); pinned here so it stays true.
    hole_vertex = 2 * 6 + 3
    v, t, c = grid_plane(6, hole=hole_vertex)
    v1, t1, c1, stats1 = complete_mesh(
        v, t, c, min_component_triangles=3, max_hole_radius=5.0)
    assert stats1['filled'] == 1

    v2, t2, c2, stats2 = complete_mesh(
        v1, t1, c1, min_component_triangles=3, max_hole_radius=5.0)
    assert stats2 == {'pruned': 0, 'filled': 0}
    assert len(t2) == len(t1) and len(v2) == len(v1)


def test_empty_mesh_passes_through():
    v = np.zeros((0, 3))
    t = np.zeros((0, 3), dtype=int)
    c = np.zeros((0, 3))
    v2, t2, c2, stats = complete_mesh(
        v, t, c, min_component_triangles=30, max_hole_radius=0.25)
    assert len(t2) == 0
    assert stats == {'pruned': 0, 'filled': 0}


def test_closed_surface_is_untouched():
    # A closed box has no boundary loops at all — completion must be a
    # strict no-op, not "fill something anyway".
    v, t, c = open_box()
    lid = np.array([[4, 5, 6], [4, 6, 7]])
    t = np.vstack([t, lid])
    assert boundary_loops(t) == []

    v2, t2, c2, filled = fill_interior_holes(v, t, c, max_hole_radius=5.0)
    assert filled == 0
    assert len(t2) == len(t)
