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
  the offline-artifact job; the two displays coexist.

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

## P0 — The mesher node, proven against a bag

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

## P1 — Into `just world`

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

## P2 — Depth scale alignment (the standing todo, landed)

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

## P3 — 6-DoF live poses (opt-in, gated on it actually tracking)

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
