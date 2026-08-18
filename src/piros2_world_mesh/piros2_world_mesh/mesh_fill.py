"""
Mesh completion: prune debris, classify boundary loops, fill interior holes.

Pure numpy on (vertices, triangles, colours) arrays — no open3d — so the
whole policy is unit-testable on the system interpreter, the same split
as the marker/PLY helpers (mesh-completion plan P2).

The P0 census showed "holes" are three different things, and only one
of them should be filled:

- **Debris islands**: hundreds of tiny disconnected components (noise
  flakes under ~64 triangles on a live scan). Their outer boundaries
  masquerade as holes; the honest fix is to remove the flakes, not cap
  them into blobs.
- **Frontiers**: every connected component's largest boundary loop is
  the edge of what the camera has seen — for the main surface, the
  open edge of the scan. Filling it would invent unseen space, so each
  component keeps its largest loop open.
- **Interior holes**: every other boundary loop, bounded by observed
  surface on all sides. These get filled by assuming the surface from
  the surrounding detail — a fan to the loop's centroid, coloured from
  the ring — under a radius guard so a pathological giant loop is
  never bridged silently.
"""

import numpy as np


def _vertex_components(n_vertices, triangles):
    """Union-find over triangle edges -> component root per vertex."""
    parent = np.arange(n_vertices)

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b, c in triangles:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra
        rc = find(c)
        ra = find(ra)
        if ra != rc:
            parent[rc] = ra
    return np.array([find(i) for i in range(n_vertices)])


def prune_small_components(vertices, triangles, colours, min_triangles):
    """
    Drop connected components with fewer than min_triangles triangles.

    Returns (vertices, triangles, colours, n_pruned_components) with
    vertices reindexed densely. min_triangles <= 1 is a no-op.
    """
    if min_triangles <= 1 or len(triangles) == 0:
        return vertices, triangles, colours, 0

    roots = _vertex_components(len(vertices), triangles)
    tri_roots = roots[triangles[:, 0]]
    keep_roots, counts = np.unique(tri_roots, return_counts=True)
    big = set(keep_roots[counts >= min_triangles])
    keep_tri = np.array([r in big for r in tri_roots])
    n_pruned = int((counts < min_triangles).sum())
    if n_pruned == 0:
        return vertices, triangles, colours, 0

    triangles = triangles[keep_tri]
    used = np.unique(triangles)
    remap = np.full(len(vertices), -1, dtype=triangles.dtype)
    remap[used] = np.arange(len(used))
    return vertices[used], remap[triangles], colours[used], n_pruned


def boundary_loops(triangles):
    """
    Boundary loops as lists of vertex indices, in triangle winding order.

    A boundary edge belongs to exactly one triangle; following each
    directed boundary edge a->b chains the loops in the orientation the
    surrounding surface implies, which is what lets a fill patch keep
    consistent winding. The walk consumes directed *edges*, not
    vertices: marching-cubes surfaces pinch loops through shared
    (non-manifold) vertices, and a vertex-keyed walk silently drops
    every loop that crosses one — measured live as small holes the
    filler never saw. At a pinch the greedy choice may fuse two loops
    into one figure-eight cycle; it still gets classified and filled.
    """
    if len(triangles) == 0:
        return []
    directed = np.concatenate(
        [triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
    key = np.minimum(directed[:, 0], directed[:, 1]).astype(np.int64) << 32 \
        | np.maximum(directed[:, 0], directed[:, 1])
    uniq, counts = np.unique(key, return_counts=True)
    boundary_keys = set(uniq[counts == 1].tolist())
    # a -> [b, ...] successor lists over directed boundary edges
    succ = {}
    boundary_edges = []
    for a, b in directed:
        k = (min(a, b) << 32) | max(a, b)
        if k in boundary_keys:
            a, b = int(a), int(b)
            succ.setdefault(a, []).append(b)
            boundary_edges.append((a, b))

    loops, used = [], set()
    for start_edge in boundary_edges:
        if start_edge in used:
            continue
        loop = [start_edge[0]]
        cur = start_edge
        while cur not in used:
            used.add(cur)
            _, b = cur
            loop.append(b)
            nxts = [n for n in succ.get(b, []) if (b, n) not in used]
            if not nxts:
                break
            cur = (b, nxts[0])
        # Closed if the walk returned to where it began.
        if loop[0] == loop[-1] and len(loop) >= 4:
            loops.append(loop[:-1])
    return loops


def _loop_radius(vertices, loop):
    pts = vertices[loop]
    return float(np.linalg.norm(pts - pts.mean(axis=0), axis=1).max())


def fill_interior_holes(vertices, triangles, colours,
                        max_hole_radius, tint=None):
    """
    Close interior boundary loops; leave every component frontier open.

    Fill = a fan from the loop's centroid, wound to match the directed
    boundary (patch triangle (b, a, centroid) for boundary edge a->b).
    The centroid vertex takes the ring's mean position and mean colour —
    the hole assumed from its surroundings — or `tint` (an RGB triple)
    when set, so assumed surface is visually distinguishable. Loops
    wider than max_hole_radius stay open: a guard against silently
    bridging something frontier-sized.

    Returns (vertices, triangles, colours, n_filled).
    """
    loops = boundary_loops(triangles)
    if not loops:
        return vertices, triangles, colours, 0

    roots = _vertex_components(len(vertices), triangles)
    frontier = {}  # component root -> (radius, loop index)
    radii = []
    for i, loop in enumerate(loops):
        r = _loop_radius(vertices, loop)
        radii.append(r)
        root = roots[loop[0]]
        if root not in frontier or r > frontier[root][0]:
            frontier[root] = (r, i)
    exempt = {i for _, i in frontier.values()}

    new_vertices, new_colours, new_triangles = [], [], []
    n_filled = 0
    next_index = len(vertices)
    for i, loop in enumerate(loops):
        if i in exempt or radii[i] > max_hole_radius:
            continue
        ring = vertices[loop]
        centroid = ring.mean(axis=0)
        colour = np.asarray(tint, dtype=float) if tint is not None \
            else colours[loop].mean(axis=0)
        new_vertices.append(centroid)
        new_colours.append(colour)
        # succ order: loop[j] -> loop[j+1] is a directed boundary edge
        for j in range(len(loop)):
            a, b = loop[j], loop[(j + 1) % len(loop)]
            new_triangles.append((b, a, next_index))
        next_index += 1
        n_filled += 1

    if n_filled == 0:
        return vertices, triangles, colours, 0
    vertices = np.vstack([vertices, np.array(new_vertices)])
    colours = np.vstack([colours, np.array(new_colours)])
    triangles = np.vstack([triangles, np.array(new_triangles,
                                               dtype=triangles.dtype)])
    return vertices, triangles, colours, n_filled


def complete_mesh(vertices, triangles, colours, min_component_triangles,
                  max_hole_radius, tint=None):
    """
    Run the whole completion policy in one call.

    Prunes debris components, then fills interior holes. Returns
    (vertices, triangles, colours, stats) where stats =
    {'pruned': n_components, 'filled': n_holes}.
    """
    vertices, triangles, colours, pruned = prune_small_components(
        vertices, triangles, colours, min_component_triangles)
    vertices, triangles, colours, filled = fill_interior_holes(
        vertices, triangles, colours, max_hole_radius, tint)
    return vertices, triangles, colours, {'pruned': pruned,
                                          'filled': filled}
