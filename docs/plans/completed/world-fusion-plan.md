# World fusion plan — from a voxel panorama to a room you can measure

> **Completed 2026-08-10 — started and finished the same day**, across
> three sessions: P0–P3 + all tooling in the first, the live checks and
> the (dark, then unplugged, then good) sweep saga in the second, the
> scale check and close-out in the third. Kept as the build log;
> per-phase annotations record what actually happened, including the
> two findings that outgrew the plan: pose error and per-frame neural
> depth wobble are *separate* failure modes with distinct mesh
> signatures, and the scale check pinned `depth_scale` at 2.69 with
> the wobble measured at ±4%.
>
> The learning plan for the topics in
> [info.md](../../../info.md) (capture / fusion / output, TSDF, pose
> graphs, meshing, plane fitting), each phase pairing study with a
> concrete improvement to `src/piros2_world`. It exists to close the
> standing todo — *"cloud map isn't great, i need a better data fusion
> pipeline"* — the honest way: understand the representation first, then
> replace the latest-wins voxel dict with real fusion.

The reference text is the three-representation argument in `info.md`:
**capture** (keyframes + poses, the only data you can't regenerate),
**fusion** (a sparse voxel-hashed TSDF, because a running weighted average
is what kills depth noise), and **output** (marching-cubes mesh plus a
structural plane/object layer). This plan works through those in order,
against this repo's actual hardware and its actual constraints.

## Topic → phase map

| Topic from info.md | Learned in | Applied in |
| --- | --- | --- |
| SE(3), `T_wc` conventions, quaternions | P0 | `se3.py`, every later phase |
| SDF / TSDF, weighted integration, voxel hashing | P1 | P1 script, P2 node, P4 |
| Marching cubes, mesh formats (PLY/GLB) | P1 | P1, P5 |
| Capture layer, TUM RGB-D layout, keyframes | P3 | `export_capture.py` |
| Pose graphs, information matrices, loop closure | P4 | RTAB-Map trajectory export |
| Bundle adjustment | P4 (reading only) | — |
| RANSAC, plane fitting, Manhattan frame, OBBs | P5 | `room_layer.py` |
| Surfels, 3D Gaussian Splatting | sidebar reading | — (recorded out of scope) |

## The honest scope

Three limits shape everything below; none is a surprise, all are already
documented elsewhere in this repo:

- **Depth is monocular and approximately scaled.** Depth Anything V2's
  output is scaled by a hand-set `depth_scale`, and the tape-measure
  check ([perception.md](../../info/perception.md)) is still open. A TSDF
  fused from it is *geometrically consistent but metrically unproven*
  until P4 closes that check — no phase before P4 may claim metres.
- **Live odometry is rotation-only.** The keypoint detector's Kabsch
  estimator measures orientation, not position
  ([world-3d-plan.md](../completed/world-3d-plan.md)). Fusing under
  rotation-only poses gives a **panorama TSDF** — denoised, meshable, but
  one viewpoint. Real 6-DoF poses come from RTAB-Map in P4, offline.
- **Header stamps stay untrusted** (~0.73 s camera fault,
  [camera.md](../../info/camera.md#timestamps)). Live TF lookups stay
  latest-only, as `cloud_mapper` already does; offline pipelines pair
  depth with its exact source frame by construction, which is one of the
  reasons the heavier fusion work is offline.

There is no IMU, so "Z-up gravity-aligned" comes from the existing
convention (the camera starts level on the desk; `odom` is Z-up by
REP-103) and, in P5, from snapping to the fitted floor-plane normal —
exactly the fallback `info.md` names.

## Where the new code lives

Offline reconstruction scripts are **not ROS nodes** and depend on
Open3D, which (like onnxruntime) is PyPI-only. They go in a new
top-level `tools/recon/` directory, run under the existing perception
venv (`~/.venvs/piros2-perception`, the repo's documented escape hatch),
and are invoked through `just` recipes — keeping the colcon workspace and
its test suite free of imports it can't satisfy. Only P0 and P2 touch the
ROS package itself.

```
src/piros2_world/
├── piros2_world/
│   ├── se3.py                     # P0: NEW — shared transform pure functions
│   ├── keypoint_detector.py       # P0: quaternion helpers move to se3.py
│   └── cloud_mapper.py            # P0: same; P2: VoxelMap → weighted fusion
├── test/
│   ├── test_se3.py                # P0: NEW
│   └── test_cloud_mapper.py       # P2: fusion-semantics tests
tools/recon/                       # NEW — offline, venv python, no ROS deps
├── fuse_tum.py                    # P1: TSDF + mesh from a TUM sequence
├── export_capture.py              # P3: bag → TUM-layout keyframe directory
├── fuse_capture.py                # P4: TSDF + mesh from our own captures
└── room_layer.py                  # P5: planes / Manhattan snap / room.json
captures/                          # P3: exported keyframe dirs — git-ignored
datasets/                          # P1: TUM downloads — git-ignored
meshes/                            # P1/P4/P5 outputs — git-ignored
```

## P0 — Conventions and SE(3): one transform module, written down ✓ (2026-08-10)

> Landed as planned: `se3.py` carries `BASE_FROM_OPTICAL` and both
> quaternion conversions (moved from the detector and mapper — they were
> each other's inverse living in different files), plus `make_transform`
> / `invert` / `transform_points` with frame-subscript argument names.
> `cloud_mapper.on_cloud` now builds an explicit `t_wc` and calls
> `transform_points` — same arithmetic, the convention named. The
> geometry tests moved to a new `test_se3.py` and grew: the quaternion
> round trip now drives **all four branches** of the conversion (the
> three non-trace branches only fire near 180°, exactly where a bug
> would hide), plus invert/compose/hand-worked-composition tests and the
> optical-axis mapping. Suite 70 → 74 green. One stale-artifact snag:
> `build/piros2_world` still referenced the deleted `world3d.launch.py`
> and failed the rebuild — cleaned and rebuilt. Remaining: a `just
> world` glance to confirm nothing observable changed (refactor only;
> node-level tests instantiate both nodes already) — **passed
> 2026-08-10 evening, human-confirmed: `just world` behaves as
> before.**

**Study first** (this phase is mostly reading; the code is small on
purpose):

- Rotation representations — matrix ↔ quaternion ↔ axis-angle, and why
  each exists. Barfoot, *State Estimation for Robotics*, ch. 6 (free
  PDF), or the first lecture of any visual-SLAM course.
- The `T_wc` naming convention: a 4×4 `T_wc` maps *camera-frame points
  into world frame*, and `T_wc = T_wb @ T_bc` composes right-to-left.
  Re-read [REP-103](https://ros.org/reps/rep-0103.html) /
  [REP-105](https://ros.org/reps/rep-0105.html) with this in hand — tf2's
  `lookup_transform('odom', 'camera', …)` *is* `T_wc`; the repo has been
  using the convention all along without naming it.
- A first pass at so(3)/se(3) and the exp/log maps — only far enough to
  know why optimisers (P4's pose graphs) parameterise rotations as
  3-vectors, not 9 matrix entries.

**Build:** `piros2_world/se3.py`, pure functions, no ROS imports:
`quaternion_from_rotation` / `rotation_from_quaternion` (moved from
`keypoint_detector.py` and `cloud_mapper.py` — they are each other's
inverses and currently live in different files), plus `make_transform(R,
t)`, `invert(T)`, and `transform_points(T, xyz)`. Field and argument
names carry the frame convention (`T_wc`, never a bare `T`). Both nodes
import from it; behaviour is unchanged.

Unit tests: quaternion round-trips through all four branches of the
conversion, `invert(T) @ T = I`, composition order against a hand-worked
example, and the `BASE_FROM_OPTICAL` conjugation reproducing the
canonical −90/0/−90 optical rotation.

**Runnable check:** `just test` green with the new `test_se3.py`;
`just world` behaves exactly as before (the refactor changed nothing
observable).

## P1 — TSDF on known-good data: the Open3D tutorial, made ours ✓ (2026-08-10)

> Landed as planned: open3d 0.19.0 into the perception venv (the wheel
> is CUDA-enabled out of the box — `CUDA:0` on the GTX 1660 SUPER, no
> extras needed, unlike onnxruntime), `datasets/` + `meshes/` +
> `captures/` git-ignored, `tools/recon/fuse_tum.py`, and `just
> fetch-tum` / `just fuse-tum` recipes. fr1/desk: **596 frames fused in
> 5.9 s (10 ms/frame)** at 8 mm — 24,079 voxel blocks, 708k-vertex mesh,
> 4.2 × 3.4 × 2.0 m bounding box; an offscreen render (EGL headless
> works) is recognisably the desk — chair, monitors, papers, floor.
> The experiments told their story in numbers: **4 mm** → 123k blocks
> (5×) and a 3.9M-vertex mesh — and GPU *marching cubes* OOMs on the
> 6 GB card even though integration fit, teaching that mesh extraction
> allocates a dense assist structure (the script now falls back to CPU
> extraction); **16 mm** → 5k blocks, 140k vertices, melted detail;
> truncation **2×** voxel → 489k vertices (thin band, fewer confirmed
> voxels), **10×** → 768k (band thickens every surface). Bounding box
> stays ~constant throughout — resolution changes the surface, not the
> scene.

The single highest-value exercise on the list: real fusion, no SLAM
needed, someone else's perfect poses — so every remaining concept is
isolated from our capture problems.

**Study first:**

- Signed distance functions: a scalar field, surface at the zero
  crossing; then *truncated* — store only the band near surfaces (that is
  the "~2% of voxels" claim, and the whole reason hashing works).
- TSDF integration: per voxel, project into the camera, compare depths,
  update a running weighted average. Understand why averaging
  *distances* denoises where averaging *points* (our current map) does
  not — this sentence is the entire justification for the plan.
  KinectFusion (Newcombe et al. 2011), §3 only.
- Voxel block hashing (Nießner 2013): a hash map from int3 block coords
  to 8³ voxel blocks — conceptually our `VoxelMap` dict, one level
  blockier and storing distances instead of points.
- Marching cubes: the per-cell triangle lookup-table idea. Understand it,
  never implement it.

**Build:**

- `pip install open3d` into `~/.venvs/piros2-perception`; record the
  version and whether the wheel is CPU-only in the tool's docstring
  (CPU integration is fine for offline work — verify, don't assume,
  per repo rule).
- Download a TUM RGB-D sequence with ground truth (`fr1/xyz` to start,
  `fr1/desk` for a real scene) into `datasets/` (git-ignored).
- `tools/recon/fuse_tum.py`: associate rgb/depth/groundtruth by
  timestamp (the TUM association recipe), integrate every frame into an
  `open3d.t.geometry.VoxelBlockGrid` (voxel 8 mm to start), extract the
  mesh, write `meshes/<sequence>.ply`. Expose `--voxel-size` and
  `--trunc` so their effects can be *seen*, not just read about.
- A `just fuse-tum` recipe owning the venv invocation, same pattern as
  `just depth`.

**Runnable check:** the fr1/desk mesh opens in a viewer and is
recognisably the desk. Then the experiment that proves the lesson:
re-fuse at 4 mm and 16 mm voxels and with truncation at 2× and 10× voxel
size, and look at what changes — noise modelling below 4 mm, melted
detail at 16 mm, surface thickening with fat truncation.

## P2 — Weighted fusion in the live map ✓ (2026-08-10; RViz thinner-wall glance still open)

> Landed as planned, one design refinement: the per-point Python loop
> could not survive per-point *arithmetic* (µs each × 45k points), so
> `VoxelMap` went array-backed in the same change — a dict maps voxel
> key → row, the means and weights live in preallocated numpy arrays,
> and each cloud is first collapsed to one mean per touched voxel
> (`np.unique` + `np.add.at`, the idiom the plan named) so the running
> average update runs vectorised. Weight counts *clouds*, not points —
> min_weight means "confirmed by N looks", the TSDF semantic. Measured:
> 27–30 ms per realistic 45k-point cloud (old loop: ~25 ms), 74 ms in a
> pathological all-distinct-voxel stress case; fine at 15 Hz. rgb
> unpack/repack (0x00RRGGBB float bits) averages channels, not bit
> patterns. New params `min_weight: 2`, `max_weight: 64` in world.yaml.
> Tests rewritten with the semantics: noise-averages-toward-the-plane
> (σ/√n beats σ), min_weight holdback, capped-inertia displacement (a
> "moved chair" pulls a 20-observation voxel to >190/200), plus the
> surviving cap/clear/union tests; suite at 77 green. Bag check:
> `world.launch.py` + `bags/static1` replay → clouds 46,021 points, map
> 46,239 voxels / 38,328 credible published — map ≈ live cloud on a
> static scene, the held-back 8k being the surface-edge noise the
> feature exists to suppress. The live `just world` glance **passed
> 2026-08-10 evening, human-confirmed: shimmering visibly reduced.**

Bring the averaging lesson home without the full TSDF machinery: this is
the phase that directly answers the todo. `VoxelMap`'s latest-wins rule
is replaced by per-voxel **running weighted averages** — the doc's
`weight` field, applied to the representation we already have.

- Each voxel stores summed position, summed colour, and a weight
  (observation count, capped so old evidence can still be displaced —
  the same weight cap real TSDFs use). `add()` folds new points in;
  `as_array()` divides out the weights.
- A `min_weight` parameter on the published map: voxels seen once are
  probably depth noise at a surface edge; voxels seen five times are
  furniture. Publish only the credible ones — the panorama stops
  shimmering.
- Everything else holds: dict-based sparsity, `max_voxels` cap with the
  one-shot log, `max_range`, `~/clear`, `POINT_DTYPE` wire format.
- Keep the vectorised path in mind: the insertion loop is already ~25 ms
  per 50k-point cloud; if averaging pushes it past the cloud period,
  `np.unique` on the key array + `np.add.at` is the numpy idiom — but
  measure first, optimise second.

Unit tests extend `test_cloud_mapper.py`: points scattered around a
plane with seeded noise average onto the plane (the flatness improves
with sample count — assert it); a single outlier voxel stays below
`min_weight`; the cap and clear still behave.

**Runnable check:** `just world`, camera still, on a textured wall: the
mapped wall in RViz is visibly *thinner* than before the change (one
voxel-ish, not a noise-thickened slab), and the map stops flickering as
duplicate observations pile up. A slow pan still widens the panorama as
in the world 3D plan's P3 check.

## P3 — The capture layer: bags → TUM-layout keyframes ✓ (2026-08-10)

> Landed as planned: `tools/recon/export_capture.py` + `just
> export-capture <bag> <name>`. CRC dup-skip then every-4th-frame
> keyframing; depth regenerated per keyframe by the ONNX model (CUDA,
> the estimator's own constants imported — stored as 16-bit
> *millimetre* PNGs, 1000 units/m where TUM proper uses 5000, stated in
> the fuse tool); poses composed by the detector's own
> `estimate_rotation` over consecutive keyframes and written to a
> separate `groundtruth.txt` whose header says ROTATION-ONLY out loud.
> Timestamps are bag receive times (the 0.73 s stamp fault stays
> untrusted). Zero-K bags die with a message naming the cause.
> Shape check on `bags/static1`: 72 rgb = 72 depth = 72 poses,
> K fx=fy=907 c=(640,360), max rotation from identity 0.0000° across
> the static capture (matching world-3d P0's ~0.001° figure), depth
> PNGs uint16 720×1280 with plausible relative values — absolute scale
> stays honestly unproven until P4.

`info.md`'s capture argument, translated to this repo: **the bag is
already the raw capture** — `just record` keeps the compressed frames,
`camera_info`, and TF, and everything else (depth, poses, maps) is
derived and re-derivable. What's missing is the *interchange* form that
existing tooling reads. This phase builds the exporter.

**Study first:** the TUM RGB-D format (directory layout + timestamped
file lists + trajectory file — ten minutes), and why depth is stored as
raw 16-bit millimetre PNGs, never filtered, never float.

**Build:** `tools/recon/export_capture.py` — reads a bag, writes
`captures/<name>/` in TUM layout:

- `rgb/<t>.png` — decoded keyframes. Keyframe selection is the concept
  here: skip usb_cam's CRC duplicates (reuse the detector's trick), then
  keep roughly every Nth distinct frame — fusion wants coverage, not
  60 fps.
- `depth/<t>.png` — 16-bit mm, produced by running the Depth Anything
  ONNX session per kept frame (the export runs under the venv python,
  which has onnxruntime; the depth node's model-loading code is
  importable already). Depth is *derived* — the honest place for it is
  the export, not the bag.
- `K.txt` from the bag's `camera_info` (guarding against the zero-K old
  bags, as the detector does).
- `trajectory.txt` — `T_wc` per keyframe in TUM `t tx ty tz qx qy qz qw`
  form, computed offline by re-running the ORB → Kabsch estimator over
  the exported frames (import the pure functions from
  `keypoint_detector`/`se3`). Translation is zero — rotation-only,
  stated in a header comment. This makes the doc's point physical:
  poses live in a separate file *because they get rewritten* — P4
  rewrites exactly this file and nothing else.

`just export-capture <bag> <name>` owns the invocation.

**Runnable check:** export `bags/static1`; the directory passes a shape
check (equal rgb/depth counts, valid K, one pose per keyframe,
quaternions ≈ identity for the static scene), and a spot-checked depth
PNG reopens with plausible mm values at a known object.

## P4 — Fuse our own room, then fix the poses ✓ (2026-08-10, three sessions; see the annotations)

> Built and proven on the static capture: the shared integration loop
> moved to `tools/recon/tsdf.py` (fuse_tum reproduces P1's numbers
> exactly after the refactor — 24,079 blocks, 707,605 vertices),
> `fuse_capture.py` + `just fuse-capture <name>` fuse an export at 2 cm
> voxels (mono noise deserves coarser than Kinect's 8 mm), and
> `captures/static1` → 77k-vertex mesh in 1.9 s whose render is the
> honest single-viewpoint result: surface patches in one frustum, a
> relief not a room. `--trajectory <file>` swaps the pose file —
> matched nearest-stamp-within-50 ms, unposed keyframes skipped aloud —
> so the RTAB-Map re-fuse is one flag. The export path is verified:
> **`rtabmap-export --poses_camera --poses_format 1 <db>`** emits
> exactly the TUM-form trajectory fuse_capture reads (format 1 =
> RGBD-SLAM; checked against the installed 0.22.1). Still open, needs
> hands: exposure fix + `just record 45 sweep2` (bags/sweep1 turned out
> to be a dead 0-second recording), `just map bags/sweep2`, the export
> + re-fuse comparison, and the tape-measure `depth_scale` pinning.
>
> **Evening session, after the first real sweep (2026-08-10):** the P0
> and P2 live checks passed (human-confirmed: `just world` unchanged,
> shimmering visibly reduced), and a real 45 s sweep was recorded — but
> **underexposed** (mean brightness 9–10/255: `just camera-reset` sets
> `gain=0`, the room was dim — the documented C922 dim-room trap). The
> dark data still taught things and hardened the tooling:
>
> - The exporter now integrates rotation over **every distinct frame**
>   and only *saves* every Nth — estimating keyframe-to-keyframe was a
>   design error (4× the per-step motion). The sweep's 40% untrusted
>   steps turned out to be the darkness, not the stride: the composed
>   trajectory still shows a coherent 11 s-still-then-90° pan.
> - RTAB-Map couldn't track the dark bag at all (1 node, zero odometry
>   length); a control run on `bags/static1` tracked fine, isolating
>   the light as the only fault.
> - **`rtabmap-export` aborts on these databases** ("no odometry
>   poses!?", every `--opt` mode); `rtabmap-report --poses_raw` is the
>   export that works against 0.22.1 (TUM form + a trailing id column).
>   `just map-headless <bag>` now runs the whole viz-free chain and
>   drops `<bag>_odom.txt` / `_slam.txt` next to the bag.
> - Pose stamps from a replayed bag live in a different clock than the
>   capture's receive-time stamps; `fuse_capture` now removes the
>   constant offset (difference of medians), tolerates the id column,
>   and `--poses-frame base` converts RTAB-Map's base_link poses
>   through the static chain (`T_BASE_OPTICAL`). The full chain —
>   report → offset removal → frame conversion → TSDF → mesh — ran
>   end-to-end on static1.
> - **Exact-sync pairing was a coin toss** (0–6 odometry updates across
>   identical replays): rgbd_odometry's default 5-deep sync queue gives
>   a ~100 ms-late depth almost no window at camera rates.
>   `mapping.launch.py` now sets `topic_queue_size`/`sync_queue_size`
>   30 — same replay went to 24 updates, 6 nodes and a loop closure.
> - A static bag exports ~1 pose *by design* (lookalike-frame merging),
>   so the multi-pose validation genuinely needs the lit sweep.
>
> Remaining: re-record the sweep in a lit room with gain raised
> (`v4l2-ctl gain=128` or `just cam gain:=128` before `just record`),
> then export → fuse → `just map-headless` → re-fuse → tape measure.
>
> **The lit sweep (2026-08-10, bags/sweep3 — after one more casualty:
> the first retry died at 2.6 s because the C922's USB cable pulled out
> mid-pan; kernel log `usb 4-1: USB disconnect`).** 44 s, 2,632 frames,
> brightness 34–51/255, and **0 of 2,632 rotation steps untrusted** —
> the dark sweep's 40% failure rate was light, entirely. The exporter's
> trajectory shows a smooth 117° pan. Both fusions ran:
>
> - **Panorama (rotation-only poses)**: 937k vertices — and *radially
>   smeared*, surfaces fanned through orientations. RTAB-Map explains
>   why: it measured **0.9 m of real camera translation** during the
>   "rotation-only" hand pan. The arm arc is parallax the pose model
>   cannot express; the honest-scope section predicted exactly this.
> - **RTAB-Map poses** (418 odometry updates, 41 nodes, 12 exported
>   poses, 0 loop closures; 33 keyframes posed within tolerance,
>   2.2 s clock offset removed): the radial fan is gone — and a
>   *different* failure shows: layered shingles, the same surface
>   fused at slightly different depths. **With poses fixed, the
>   residual error is the neural depth itself** — monocular depth's
>   per-frame scale wobble puts the wall at a different distance every
>   frame, and TSDF averaging cannot converge on a surface that moves.
>   The known community fix — aligning each frame's depth to the
>   running TSDF (or to its neighbours) by a per-frame median scale —
>   is recorded here as the next lever, out of this plan's scope.
>
> So the P4 comparison taught *more* than planned: two pose files, two
> distinct failure signatures, and the conclusion that pose quality and
> depth consistency are separate walls — this rig has now hit both.
> Remaining: the tape-measure `depth_scale` check (which needs no sweep
> — a static capture of a wall at a measured distance suffices).

Two fusions of the same capture, differing only in `trajectory.txt` —
which is the entire lesson.

**First: the panorama TSDF.** `tools/recon/fuse_capture.py` (mostly
shared code with `fuse_tum.py`) fuses an exported capture. On a
hand-panned sweep (`just record 45 sweep2` — exposure fixed first, per
the mapping notes in [CLAUDE.md](../../../CLAUDE.md)) with rotation-only
poses, the result is the denoised panorama: the honest ceiling of the
live pipeline, now meshable. Compare it against the live P2 map of the
same sweep — same scene, TSDF vs weighted point voxels.

**Then: real poses.** **Study first:** Grisetti et al., *A Tutorial on
Graph-Based SLAM* — nodes as SE(3) keyframes, edges as relative
transforms weighted by a 6×6 information matrix (inverse covariance:
per-axis trust), loop closure as the constraint that makes optimisation
worth running. Read what bundle adjustment is (jointly optimising poses
*and* points against reprojection error) — reading only, no build.

Then get real 6-DoF poses the way this repo already can: run the sweep
through RTAB-Map (`just map bags/sweep2`) and export its optimised
trajectory in TUM format (rtabmap ships export tooling — verify the
exact command against the installed 0.22.1 at build time; worst case the
poses are read from the database). Overwrite `trajectory.txt`, re-fuse,
compare: walls that smear under rotation-only poses lock into place
under graph-optimised ones. Open the RTAB-Map database viewer once and
*look at* the pose graph — loop-closure edges included — so the
Grisetti reading has a picture attached.

**This phase also closes the tape-measure scale check** (open since
perception P2): fuse a scene containing a wall at a measured distance,
measure the same span in the mesh, and tune `depth_scale` until they
agree — recording the number in
[perception.md](../../info/perception.md). Metric claims start here.

**Runnable check:** two meshes from one capture; the RTAB-Map-posed one
is visibly more rigid, and one span in it matches the tape measure to
within ~5–10% (monocular honesty — record the actual figure, whatever
it is).

## P5 — The output layer: mesh + a room you can query ✓ (2026-08-10; rigid room spans deferred to the depth-consistency lever)

> `tools/recon/room_layer.py` + `just room-layer <mesh>`: iterative
> `segment_plane` peeling, gravity from the floor normal, Manhattan
> yaw from the inlier-weighted wall azimuths, `room.json` in the
> info.md schema (openings/objects as honest empty arrays) and a GLB
> alongside. Two lessons surfaced by real data: **qhull rejects planar
> 3D input**, so plane boundaries use a hand-rolled 2D monotone-chain
> hull in plane coordinates; and **"largest horizontal plane" is the
> wrong floor heuristic** — in fr1/desk the desk out-inliers the floor.
> The fix is the above-fraction: a floor has the entire scene above it,
> a desk has half the room below. With it, fr1/desk labels correctly —
> floor at z≈0 (87k inliers), desk surfaces horizontal at +0.78/+0.81 m
> including a downward-facing underside, three walls snapped to axes
> (two being coplanar segments of the same wall, 6 cm apart). The
> static1 mesh exercises the fallback paths: slab-labelled mono-noise
> planes and a printed x-span (5.03 m) whose scale is exactly as
> unproven as P4 says. Run on the sweep3 RTAB-Map-posed mesh
> (2026-08-10): planes extract and label, but the structural layer
> honestly inherits the fusion's shingling — the two "x-walls" 0.97 m
> apart are the same physical wall's wobble extremes, so the printed
> spans are measurements of the depth inconsistency, not the room.
> Rigid spans need the depth-consistency lever P4 names. Remaining:
> the tape-measure scale check (static wall capture suffices).
>
> **The scale check passed 2026-08-10 evening**: a door face at a
> measured 2.50 m read a median 9.30 m over 35 keyframes at scale 10 →
> `depth_scale: 2.69` pinned in perception.yaml (and the exporter's
> default); re-export verification read 2.501 m, +0.1%. The same
> capture measured the per-frame wobble at ±4% on a static scene —
> ±10 cm at 2.5 m, five 2 cm voxels, exactly the shingle spacing in
> P4's mesh: the number that connects the scale check to the fusion
> quality. Recorded in
> [perception.md](../../info/perception.md).

**Study first:** RANSAC as the generic loop (minimal sample → fit →
count inliers → repeat — the principled version of the estimator's
reject-worst-refit rounds); a plane as `(n, d)`; the Manhattan-world
assumption; oriented bounding boxes as `{centre, R, extents}` — trivial
once P0's SE(3) landed.

**Build:** `tools/recon/room_layer.py` on the P4 mesh:

- Iterative `segment_plane` (Open3D's RANSAC): extract the dominant
  planes, label floor/ceiling by normal-vs-gravity, walls by
  verticality.
- Gravity-align: rotate the model so the floor normal is exactly +Z —
  the no-IMU fallback for `info.md`'s "Z-up gravity-aligned".
- Snap wall normals to the two dominant horizontal directions (the
  Manhattan frame); recompute plane offsets.
- Write `room.json` in the structural schema from `info.md` (`up`,
  `units`, `handedness`, `planes` with boundaries and labels), plus the
  mesh as `.glb` next to the `.ply`. Objects/openings are stretch —
  emit the empty arrays so the schema is complete and honest about it.

**Runnable check:** `room.json` reports wall-to-wall and floor-to-ceiling
distances for the scanned room; both compared against the tape measure
and the numbers recorded. The GLB opens in any glTF viewer. That is the
plan's finish line: a measurement taken off a model that started as
JPEG bytes on a Wi-Fi link.

## P6 — Bookkeeping ✓ (2026-08-10)

Docs map rows and current-state notes in `CLAUDE.md` / `README.md`, the
scale figure into [perception.md](../../info/perception.md), new
recipes grouped in the justfile, `.gitignore` entries for
`datasets/ captures/ meshes/`, suite green, and this file moves to
`docs/plans/completed/` — the move *is* the status change; fix inbound
links.

## Out of scope, recorded so nobody wonders

- **A live TSDF node.** Fusion heavier than P2's weighted voxels stays
  offline: Open3D lives outside the ROS environment (venv), and the live
  pipeline's job is the dashboard, not reconstruction. Revisit only if
  the offline loop proves too slow to iterate.
- **Surfels** — read the one-paragraph version (position, normal,
  radius, confidence; deform on loop closure) and know why not: worse
  for meshing, and this is a one-room project.
- **3D Gaussian Splatting** — the sidebar in `info.md` stands: far
  better renders, but nothing can be measured off it, and P5's entire
  point is taking a dimension off the model. Reading only.
- **Writing our own pose-graph optimiser (GTSAM/g2o)** — P4 *uses* an
  optimised graph and *inspects* it; implementing one is a different
  project. The Grisetti tutorial is the depth limit here.
- **UV texture atlases, object detection for the scene graph** — named
  in `info.md` as later quality jumps; per-vertex colour and empty
  `objects`/`openings` arrays are the honest v1.
- **IMU fusion** — still milestone 7 hardware territory; the floor
  normal is the gravity reference until then.
