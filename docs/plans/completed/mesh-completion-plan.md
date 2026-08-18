# Mesh completion plan — a live surface without gaps

**Goal:** upgrade `piros2_world_mesh`'s `tsdf_mesher` so the surface it
publishes (and saves) has no interior gaps or holes — where the scan
left a hole surrounded by real geometry, the mesher fills it by
*assuming the surface from the surrounding detail*, and says so
honestly. Per the repo convention, all work lands in the fork (and
stays behind parameters where it touches shared code); `piros2_world`'s
frozen copy of the mesher is not touched.

**Scope honesty up front:** "no holes" cannot mean watertight — a
hand-held scan always has an open outer boundary (the frontier the
camera never crossed), and inventing geometry there would be fiction.
The target is: every *interior* hole — one bounded by observed surface
on all sides — gets closed with a plausible patch; the outer boundary
stays open in the live mesh; and a separate export tier can optionally
close everything for downstream tools that need watertight input.
Filled geometry is an *assumption*, so it must be inspectable: a debug
parameter tints assumed patches a distinct colour.

## Where today's holes actually come from

Four distinct sources, each needing a different fix — conflating them
is how hole-filling efforts go in circles:

1. **The triangle cap is a sieve.** `cap_triangles` enforces
   `max_triangles: 120000` by *evenly subsampling* the triangle list —
   over budget, it deletes every Nth triangle across the whole mesh,
   peppering the entire surface with pinholes (the mesher's own warning
   calls itself the louder source of visible holes; a static desk scan
   already extracts ~200k triangles against the 120k budget).
2. **Extraction-time gaps.** Marching cubes over partially-observed
   voxel blocks leaves small interior gaps. The existing
   `fill_hole_radius: 0.06` (Open3D `fill_holes`, VTK-backed) already
   triangulates the small ones; bigger interior holes survive because
   the radius bound — there to protect the open outer boundary — cannot
   tell a large hole from the frontier.
3. **Never-observed patches.** Depth speckle (invalid or far-clipped
   pixels) and grazing-angle surfaces leave voxels with no observations
   at all — the TSDF has literally nothing there, so no extraction
   parameter can help. Filling these means assuming at the *volume* or
   *depth-image* level, not the mesh level.
4. **Weight-starved voxels.** Voxels seen once or twice carry TSDF
   values too uncertain for a clean zero-crossing; they flicker in and
   out between re-meshes and leave ragged edges around real holes.

## Phases

### P0 — measure and classify before assuming anything ✓ 2026-08-18

Instrument a bounded live session and a saved PLY: a boundary-loop
census (count and perimeter of every boundary edge loop — the one giant
loop is the frontier, everything else is an interior hole), the cap
deficit (extracted vs published triangle count), and the fraction of
holes that sit over *unobserved* voxels vs merely weight-starved ones
(query the volume along each small loop's centroid). Also pin down what
the venv's Open3D 0.19 actually ships for this work —
`fill_holes(hole_size=)` semantics, quadric decimation on tensor vs
legacy meshes, Poisson reconstruction — before designing against
imagined APIs. Ends with: a table in this plan attributing today's
holes to sources 1–4 by count and area, and a verified API inventory.

**Measured 2026-08-18** (live static scan, two in-session saves — which
also proved the `~/save` live gate; venv Open3D 0.19.0):

- **API inventory:** `t.fill_holes(hole_size)` = *max hole radius*,
  CPU-only; `t.simplify_quadric_decimation(target_reduction)` takes a
  *fraction*, legacy takes `target_number_of_triangles` (use legacy);
  legacy Poisson and `VoxelBlockGrid.extract_point_cloud` both present.
- **The published mesh** (weight 3, after today's 0.06 fill): 47k
  triangles, **47 surviving interior loops** — p50 radius **0.073 m**,
  sitting just above the 0.06 fill bound; 21 in 0.06–0.15 m, 5 in
  0.15–0.4 m, 4 above.
- **But the mesh is 374 connected components**: 3 real surfaces (36k /
  5.9k / 2.8k triangles) and ~371 noise flakes under ~64 triangles.
  Most "big holes" are the *outer boundaries of debris islands or
  secondary components*, not holes in a surface — classified
  per-component (each component's largest loop is its own frontier),
  the true interior hole count is **29**. Filling therefore needs
  three moves, not one: prune debris components, exempt every
  component's frontier, fill the rest under a radius guard.
- **Weight-starved voxels are not the hole source:** weight 1 vs 3
  *grows the fringe* (frontier radius 0.98 → 1.44 m, +1.5k triangles)
  while interior holes stay put (47 → 52, same size profile). And the
  neural depth is dense — no invalid-pixel speckle exists to inpaint.
  Both P3 levers are therefore pointed at a problem this pipeline
  doesn't have (decision recorded in P3).
- **The sieve, quantified:** even-subsampling this scan's triangles at
  50% turns an intact surface into **37,677 boundary edges across
  4,973 loops**. The 2026-08-15/16 session logs show the cap actually
  engaging at that ratio (206,528 extracted vs 103,264 shown).

### P1 — a triangle budget that doesn't punch holes ✓ 2026-08-18

Replace even-subsampling with **quadric decimation to budget**: when
the extracted mesh exceeds `max_triangles`, simplify it (legacy
`simplify_quadric_decimation` via a tensor→legacy round-trip if 0.19
demands it) instead of deleting scattered triangles. The surface stays
closed everywhere at slightly lower detail — strictly better than a
sieve. Measure the cost on a ~200k-triangle desk scan; if decimation
blows the re-mesh budget (currently 1.4–2.0 s per refresh), fall back
to decimating only when over budget by >2× and raising the cap
otherwise, with the measured numbers deciding. The pure
`cap_triangles` helper and its tests are replaced, not kept alongside
(one budget mechanism, not two). Ends with: a live static scan
publishes with zero cap warnings and zero subsampling pinholes, cost
figures recorded here.

### P2 — interior holes filled from their surroundings ✓ 2026-08-18

The core of the ask. Replace the single radius bound with **boundary
loop classification**: extract all boundary edge loops from the
triangle array (pure-numpy, unit-testable without Open3D, per the
repo's testing convention); the loop with the largest perimeter (or
enclosing area) is the scan frontier and is left open; every other loop
is an interior hole and gets closed regardless of size. Fill = ear-clip
or fan-triangulate the loop, then Laplacian-relax the patch interior so
it follows the curvature the ring implies rather than a flat lid, and
interpolate vertex colours from the ring — "assume the hole from the
surrounding detail", literally. `fill_hole_radius` is retired in favour
of `fill_interior_holes: true` plus `fill_debug_tint: false` (tint
assumed patches magenta when true). Unit tests on synthetic arrays: a
punched plane comes back closed with interpolated colour, a box missing
one face keeps that face open (it's the largest loop), a two-hole plane
closes both. Ends with: `just dev` on a static desk shows a surface
whose only opening is the scan frontier, tests green.

Done — `mesh_fill.py`, pure numpy, 8 unit tests. The build's one live
surprise: the first loop walk was vertex-keyed, and marching-cubes
surfaces pinch loops through non-manifold vertices — the filler
silently never saw those loops (12 survivors on the first live scan).
The walk is now edge-keyed (a pinch may fuse two loops into one
figure-eight, which still fills); regression test added. Live:
**pruned 373 debris components and filled 142 interior holes** in one
session, completion costs ~200-260 ms inside a 750-820 ms refresh, and
the saved mesh re-censuses to **zero interior loops ≤ 0.25 m** — every
remaining opening is a component frontier. Re-running completion on
its own output is a no-op (idempotent).

### P3 — the holes nothing observed (choose the lever P0 measured) ✓ 2026-08-18 — both levers skipped

For source 3, two candidate levers, picked by P0's attribution table
rather than taste:

- **Depth-image pre-fill:** inpaint small invalid/far-clip speckles in
  the depth image before integration (`cv2.inpaint` on masks below a
  size threshold, guarded by `depth_fill_max_px`). Cheap, runs at the
  paced 5 Hz, and fixes the hole before it ever reaches the volume —
  but only helps where the *image* had speckle, not where the camera
  never pointed.
- **TSDF neighbourhood diffusion:** after integration, propagate TSDF
  values one or two voxels into unobserved neighbours (bounded
  dilation), letting marching cubes close what the surroundings imply.
  Stronger, but touches the volume's honesty and costs GPU time per
  re-mesh; if P0 shows most unobserved holes are speckle-shaped, skip
  it entirely.

Ends with: the chosen lever behind a parameter, its cost measured, and
the unchosen one recorded here with the number that killed it.

Decision: **both levers skipped, by P0's numbers.** Depth-image
pre-fill has nothing to fill — the neural depth is dense, no invalid
pixels exist. TSDF diffusion targets weight-starved voxels, and P0
measured weight 1 vs 3 growing the *fringe* (frontier 0.98 → 1.44 m)
while interior holes stayed put (47 → 52) — the lever moves the wrong
thing. P2's loop fill already closes what both were aimed at.

### P4 — the watertight export tier, docs, and gates ✓ 2026-08-18 (one live gate open)

`~/save` (`just mesh-save`) gains `save_watertight` (default false):
on save, run Poisson reconstruction from the TSDF's point cloud +
normals (or VTK's fill on the full-detail mesh — P0's API inventory
decides), producing a closed mesh for downstream tools, saved
alongside the honest one (`live_<stamp>_closed.ply`), never replacing
it — the blobby-Poisson trade-off documented where the parameter is
declared. Update the fork's docs surface: `world_mesh.yaml` comments,
the diagrams page's mesher rows and TSDF figures, CLAUDE.md's
world-mesh paragraph, troubleshooting if any new trap surfaced. Live
gates: a hand sweep whose mesh shows no interior holes at rest, one
in-session save of both PLYs opened in `just view-mesh`. Ends with:
gates ticked with dates, figures final, this plan moved to
`docs/plans/completed/`.

Built — `save_watertight` (fork yaml: true): Poisson at depth 9 over
the TSDF's own cloud, plus an unbounded fill pass because Poisson
clips at its reconstruction box (measured: 223 boundary edges left at
the clip; after the pass, **zero** — verified closed, colours
interpolated). Writes `live_<stamp>_closed.ply` beside the honest PLY,
never instead of it. Gates: the in-session save ran live twice
(2026-08-18, both tiers) — that gate is closed. The hand-sweep gate
was written as "needs a human with the camera"; it ran the same evening
by replay instead ([docs/info/verification.md](../../info/verification.md)):
`just run-bag bags/sweep3` (a real 44 s hand sweep through the full
rgbd session), `just snap` (120k live triangles latched, 6 keyframe
strokes), `just mesh-save` (723 604 honest triangles + the
Poisson-closed tier) and `just mesh-views` rendering the PLY from
above — one far sheet, no visible doubling at plan scale
(`captures/verify/mesh_live_20260818-175522/sheet.png`). Reading that
render is still a judgement, but about a file anyone can reopen;
`fill_debug_tint: true` still shows exactly what was assumed if the
patches are the question.
