# Live mesh plan — the live pipeline grows a surface

> **In progress, started 2026-08-11.** The redesign requested after the
> world fusion plan: the live chain `camera → depth → /points (Depth3D)
> → cloud_mapper → /world/map_points (CloudMap)` continues into a
> **live mesh** — a TSDF integrated in-session, re-meshed on a timer,
> visible in the same RViz window and updating as the camera looks
> around. No more offline crank-turning to see a surface.

This supersedes the fusion plan's scope line "fusion heavier than
weighted voxels stays offline" — deliberately, with eyes open: the
offline loop proved the tooling (GPU integration at 10 ms/frame,
meshes that render), measured the two failure modes (pose smear, ±4%
per-frame depth wobble), and produced the fix for the second one as a
standing todo. This plan brings that machinery live and **implements
the depth-alignment todo where it pays off most**.

## What already exists that this builds on

| Have | Where |
| --- | --- |
| GPU TSDF integration, ~10 ms/frame, `VoxelBlockGrid` | `tools/recon/tsdf.py` (fusion plan P1/P4) |
| `ray_cast` on `VoxelBlockGrid` (verified present in 0.19) | needed by P2's alignment |
| The venv `ExecuteProcess` pattern for non-apt Python deps | `depth_estimator` in `world.launch.py` |
| Exact-stamp depth↔RGB pairing (depth node copies headers) | `cloud_projector`'s message_filters sync |
| Latest-only TF lookups (the 0.73 s stamp fault rule) | `cloud_mapper` |
| A latched mesh display in RViz (`/world/mesh`, FusedMesh) | `mesh_marker` (2026-08-11) |
| Measured: RViz assimp loads PLY, rejects Open3D GLB | mesh_marker session |
| Measured: ±4% per-frame depth wobble on a static scene | fusion plan P4/P5 annotations |

## The honest scope

- **Poses are rotation-only at first.** The live TF is the keypoint
  detector's compass; a hand pan carries real translation (RTAB-Map
  measured 0.9 m in "rotation-only" sweep3), and that parallax smears
  any accumulation — CloudMap has this today, the live mesh inherits
  it. P3 offers the 6-DoF upgrade; P0–P2 do not pretend.
- **Scale alignment fixes wobble, not parallax.** P2 stops the mesh
  breathing (the ±4%); it cannot un-smear arm motion.
- **The mesh is dev-box-local.** A triangle-soup Marker at ~10–30 MB
  per refresh is fine on loopback and must never cross the Wi-Fi —
  same class of rule as "never stream raw images".
- **RViz mesh_resource caching is why we publish geometry.** RViz
  caches `file://` meshes by URI, so rewriting one file and reloading
  shows stale geometry (and unique-name churn leaks disk). The live
  mesh therefore ships as a `TRIANGLE_LIST` Marker — actual triangles
  in the message — sidestepping files entirely. `mesh_marker` keeps
  the offline-artifact job; the two displays coexist. *(They did until
  2026-08-12 — mesh_marker has since been removed; see the P1 note.)*

## Changes at a glance

```
src/piros2_world/
├── piros2_world/
│   ├── tsdf_mesher.py        # P0: NEW — TSDF integrate + timed re-mesh (venv node)
│   ├── depth_align.py        # P2: NEW — pure per-frame scale alignment, shared with tools/
│   └── keypoint_detector.py  # P3: publish_tf param (yield odom→base_link to rgbd_odometry)
├── config/
│   ├── world.yaml            # params per phase
│   └── world.rviz            # LiveMesh display on /world/mesh_live
├── launch/world.launch.py    # P1: + tsdf_mesher (venv ExecuteProcess); P3: + rgbd_odometry (opt-in)
└── test/
    ├── test_tsdf_mesher.py   # P0: marker-building pure functions (numpy-only)
    └── test_depth_align.py   # P2: alignment on synthetic depth pairs
tools/recon/fuse_capture.py   # P2: --align flag reusing depth_align (de-shingle offline too)
```

`tsdf_mesher` runs under the perception venv (Open3D is PyPI-only) and
must lazy-import open3d inside the class, the `depth_estimator`
pattern — the colcon test run uses the system interpreter and only the
numpy-pure marker/alignment functions are unit-tested.

## P0 — The mesher node, proven against a bag ✓ (2026-08-11)

> Landed as planned, one API lesson: `VoxelBlockGrid.integrate` accepts
> only (float, float) or (uint16, uint8) depth/colour dtype pairs — our
> float32-metres depth forced the colour to float32 [0,1] (the offline
> pipeline never hit this because PNG files load as uint16/uint8).
> Bag check against `bags/static1` (depth estimator + keypoint
> detector + mesher, 5 s refresh): **115 frames integrated at
> 52 ms/frame** (decode + transfer + integrate), refreshes of ~51–56k
> triangles in **630–780 ms** each, and the probe received the latched
> `/world/mesh_live` marker — 54,743 triangles in `odom`. Below the
> 60k cap on a single static view, so no capping fired. A teardown
> race (refresh mid-shutdown publishing into a dead context) got an
> `rclpy.ok()` guard. 5 new unit tests (cap, marker building, and the
> lazy-init contract: node constructs and resets on the system
> interpreter with no open3d); suite at 86 green.

`piros2_world/tsdf_mesher.py`:

- Subscribes `/depth` + `/image_raw/compressed`, exact-stamp synced
  (message_filters, the `cloud_projector` contrast lesson in reverse:
  here mismatched pairs would colour the wrong surface). Caches K from
  `/camera_info` with the zero-K guard.
- Per pair: latest-only TF lookup `odom ← camera_optical_frame`
  (stamp rule), integrate into a `VoxelBlockGrid` — voxel 2 cm
  (mono-noise resolution, the fusion plan's number), trunc 4×,
  `depth_max` 6 m, CUDA with CPU fallback, all `world.yaml` params.
- A refresh timer (default 10 s): `extract_triangle_mesh`
  (weight_threshold param) → one `TRIANGLE_LIST` Marker on
  `/world/mesh_live`, per-vertex colours, latched QoS so a late RViz
  still gets the last refresh. A `max_triangles` cap subsamples
  loudly (log what was dropped) — no silent truncation.
- `~/reset` Trigger clears the volume (mirrors `cloud_mapper/clear`;
  the drift/parallax recovery story is the same).
- Fail-honest: no K or no TF yet → throttled warn, integrate nothing.

Pure functions kept ROS-free and venv-free for tests: mesh arrays →
Marker points/colors, the triangle cap, timing bookkeeping.

**Runnable check:** `ros2 launch` the mesher alone + `bags/static1`
replay (the world stack pattern from the fusion plan's P2 bag check):
`/world/mesh_live` publishes within one refresh period, the mesh in a
standalone RViz roughly coincides with CloudMap, and a second replay
after `~/reset` rebuilds it. Integration cost logged per frame
(expect ~10–15 ms GPU); extraction cost logged per refresh.

## P1 — Into `just world` ✓ (2026-08-11; live hand-pan glance still open)

> Landed as planned: seventh process in `world.launch.py` (venv
> `ExecuteProcess`), **LiveMesh** display in `world.rviz`, and
> FusedMesh flipped to default-off — the live surface is the primary
> now, the offline artifact a toggleable reference. Full-stack bag
> check: `world.launch.py` + static1 replay → the probe received both
> latched markers (LiveMesh 58,272 triangles, FusedMesh pointing at
> the sweep3 PLY). Remaining: the human glance — camera still, the
> view solidifies; pan, the panorama grows; `~/reset` clears.
>
> **2026-08-12: FusedMesh retired entirely.** With the live surface
> proven, the unaligned offline overlay earned no keep — `mesh_marker`
> (node, config, RViz display, test) was removed from the stack a day
> after it landed. Its measured findings stay recorded here (assimp
> loads PLY, rejects Open3D GLB; RViz caches `mesh_resource` by URI)
> and in troubleshooting.md; offline meshes in `meshes/` remain
> viewable in external viewers, and the `tools/recon/` pipeline that
> produces them is untouched.

- `world.launch.py` gains the mesher as a venv `ExecuteProcess`
  (seventh process; the launch's docstring count updates).
- `world.rviz` gains **LiveMesh** on `/world/mesh_live` next to
  FusedMesh (offline artifact, unchanged job: comparison/reference).
  CloudMap stays — points vs surface of the same accumulation is
  exactly the representation lesson made visible; toggle whichever.
- Wi-Fi audit: the mesher subscribes topics already crossing the LAN
  (compressed) or local (`/depth`); the Marker is local. Nothing new
  crosses the network.

**Runnable check:** `just world`, camera still: within a refresh the
view solidifies into a surface that matches Depth3D; a slow pan grows
it into the panorama (with rotation-only smear, stated and expected);
`~/reset` clears it live. Closing RViz tears everything down.

## P2 — Depth scale alignment (the standing todo, landed) ✓ (2026-08-11 — with two findings the plan didn't predict)

> Landed, but not as first designed — the build log here matters more
> than the code:
>
> 1. **Conform-to-map is unstable.** The naive loop (scale each frame
>    to the ray-cast expected depth) made the wall *walk away* at
>    ~+1%/frame. Cause, isolated by experiment: `VoxelBlockGrid`'s
>    ray-cast surface reads ~1.25 voxels behind the integrated one
>    (+1.0% at 2 cm voxels on a 2.5 m scene, +0.5% at 1 cm —
>    voxel-proportional, truncation-independent), and any constant
>    error in a conform-to-map loop compounds. A frame-0 bias
>    calibration failed too — the bias shifts as the map accumulates.
> 2. **The stable shape is a high-pass**: `ScaleAligner` corrects only
>    each frame's *deviation* from a rolling median of raw ratios —
>    wobble is fast, bias is slow, and drift is impossible by
>    construction (correction factors have median 1 over the window).
>    Unit-tested on exactly the measured failure mode: constant 1%
>    renderer bias + ±4% wobble → wobble collapses, no net push.
> 3. **Honest ceiling, measured**: on the wallcheck capture the
>    per-frame centre-depth spread drops 4.0% → 2.9% — about half the
>    variance. The remainder is *spatially structured* depth error (the
>    model's wobble is not a single global scale), which a global
>    correction cannot touch; per-pixel/affine alignment is the named
>    next lever, out of scope. Ray-cast weight_threshold 1/3/8 barely
>    matters (2.99/2.91/2.92%); 3 kept. The sweep3 re-fuse experiment
>    was inconclusive *for pose reasons*: 33 keyframes sharing 12
>    RTAB-Map poses bake in up-to-a-second pose errors that dwarf the
>    wobble — alignment cannot and should not fix wrong poses.
>
> `depth_align.py` (pure numpy: `depth_ratio` + `ScaleAligner`), wired
> into `tsdf_mesher` (align/min_overlap/max_correction params, scale
> stats in the log) and `tools/recon/tsdf.py` (`fuse_capture --align`,
> `_aligned` mesh label). `align: true` by default. todo.md item closed.

The ±4% per-frame wobble is what shingled the offline sweep and what
would make the live mesh breathe. The fix, per frame, before
integration:

- `ray_cast` the current TSDF from the incoming frame's pose →
  expected depth image (the KinectFusion trick, reused for
  photometric-free alignment).
- Over pixels where both expected and incoming depth are valid and
  finite: `scale = median(expected / incoming)`; require a minimum
  overlap fraction (param) — a mostly-new view integrates unaligned
  (there is nothing to align to), which is correct: the first frame
  *defines* the scale, later overlapping frames conform to it.
- Clamp the correction (param, e.g. ±15%) so a genuinely wrong TF or
  a depth failure can't fold the map into itself; log when clamped.

`depth_align.py` holds this as pure numpy on (expected, incoming)
arrays — unit-tested with synthetic planes at known scale offsets —
and `tools/recon/fuse_capture.py` gains `--align` using the same
function, so the offline sweep gets the de-shingling for free.

**Runnable check (the measurable one):** camera still on the wall for
60 s: log the applied per-frame scale — its spread should collapse
from ±4% raw toward ±1% aligned — and the mesh visibly stops
breathing between refreshes. Offline: `just fuse-capture sweep3
--trajectory bags/sweep3_slam.txt --poses-frame base --align` versus
the shingled original — the layered wall collapses toward one
surface. Update `todo.md`: the alignment item moves to done.

## P3 — 6-DoF live poses (opt-in, gated on it actually tracking) *(wiring built and bag-verified 2026-08-11; the live hand-sweep gate needs a hand)*

> Built as planned: `odom:=rgbd` launch arg (default `kp`),
> `publish_tf` param on the detector (unit test: TF silenced, the
> orientation topic keeps publishing), and conditioned
> `image_republisher` + `rgbd_odometry` nodes carrying
> mapping.launch.py's params including the 30-deep sync queues. Bag
> check: `world.launch.py odom:=rgbd` + static1 replay → `/tf` is
> published by **rgbd_odometry and not the keypoint detector** (the
> REP-105 gate works), 101 odometry-quality updates over the 19 s
> replay, and the mesher integrates under the 6-DoF poses. Remaining:
> the live hand-held sweep — walls staying put under real translation
> — which decides adopted vs "measured, not adopted".

The one wall P2 leaves standing is parallax under rotation-only TF.
The repo already proved RTAB-Map's `rgbd_odometry` tracks this rig's
depth (24 updates on a 19 s static replay after the sync-queue fix;
live rates are the same ~10 Hz depth). The swap:

- `world.launch.py` optionally (launch arg `odom:=rgbd`, default off)
  starts `rgbd_odometry` with the mapping launch's remaps/params —
  it publishes real 6-DoF `odom → base_link` TF.
- `keypoint_detector` gains a `publish_tf` param; the rgbd mode sets
  it false so the two odometries don't fight over the frame
  (REP-105: one parent per frame). The detector keeps publishing
  `/camera/orientation` — its compass is still the dashboard's story.
- Everything downstream (cloud_mapper, tsdf_mesher) consumes TF and
  needs zero changes — the payoff of having gone through TF all
  along.

**Runnable check:** `just world odom:=rgbd`, hand-held slow sweep:
walls stay put as the camera *translates* (the thing rotation-only
could never do); odometry loss (it will happen — blur, blank walls)
logs visibly, and `Odom/ResetCountdown` recovers it. If live tracking
proves too flaky to demo, the phase closes as "measured, not adopted"
with the numbers — that is a valid outcome; the launch arg stays.

## P4 — Bookkeeping

Docs-map rows and current-state notes (`CLAUDE.md`, `README.md`), the
launch docstring node count, `todo.md` alignment item closed by P2,
suite green, and this file moves to `docs/plans/completed/` — the move
*is* the status change; fix inbound links.

## Out of scope, recorded so nobody wonders

- **Loop closure / relocalisation** — a drifted live mesh is reset,
  not repaired; `rtabmap` (the SLAM node) stays offline via
  `just map`/`map-headless`. Aligning the offline FusedMesh overlay
  to the live session is the same problem and stays out with it.
- **room.json live** — plane extraction stays offline
  (`just room-layer`); the live surface is for eyes, the structural
  layer for measurements.
- **Meshing on the Pi or over the network** — dev-box-local only.
- **Texture atlases, mesh simplification beyond the triangle cap** —
  quality work for later; per-vertex colour is the honest v1.
