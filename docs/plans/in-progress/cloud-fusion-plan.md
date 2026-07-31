# Cloud fusion plan — a persistent room map with hand-rolled odometry

> **Working document.** [perception.md](../../info/perception.md) is the design context and
> [perception-plan.md](../completed/perception-plan.md) (closed 2026-07-29) built the per-frame
> pipeline this upgrades. Started 2026-07-30.

## The upgrade in one paragraph

Today `cloud_projector` rebuilds the whole cloud from scratch every frame:
~3 fps of independent snapshots that flicker with the neural depth's noise
and vanish the moment the camera looks away. The upgrade is a **persistent
map** that the camera feed *edits* instead of replaces: a fixed lattice of
cells seeded as a 2 m × 2 m × 2 m cube, where every new depth image nudges
each visible cell a little towards what was observed. Repeated agreement
saturates a cell — it is then treated as static (walls, furniture) and
stops drifting with per-frame noise — until a sustained, large disagreement
re-opens it (someone moved the chair). Once fusion works from a fixed
camera, the plan **hand-rolls the poses too**: while the camera is static,
keypoints detected in the image are back-projected into a 3D landmark set;
when the camera moves, re-finding those landmarks in the new image and
solving for the viewpoint that explains their shift gives the camera's
motion — direction and degree — and the map keeps fusing from the moving
camera.

## The ideas, named

Both halves of this plan are classic algorithms, built by hand on purpose.

**The map: truncated signed distance fusion.** The
push/pull-until-saturation scheme is, almost exactly, how the classic
fusion algorithms work:

- Each cell stores a **value `D`** (how far in front of / behind the
  observed surface this cell sits, truncated to a band ±τ) and a
  **weight `w`** (how many observations, capped at `w_max`).
- A new depth frame updates every cell that projects into it:
  `D ← (w·D + d_obs) / (w + 1)`, `w ← min(w + 1, w_max)`.
  That running average *is* the incremental push/pull: early observations
  move a cell a lot, later ones barely — and the cap is the saturation.
- The rendered surface is the set of cells where `D` crosses zero, so as
  `D` is nudged, the visible point effectively slides along the camera ray
  towards the real surface. The seed cube deforms into the room.
- Saturated cells get **hysteresis**: once `w = w_max`, a single outlier
  changes nothing, but `reopen_frames` consecutive observations differing
  by more than `change_threshold` reset `w` low, letting the cell
  re-converge to the new state.

In the literature this is a **TSDF** with weighted running-average fusion
(KinectFusion's update rule), same family as OctoMap's clamped log-odds.

**The poses: keypoint odometry via PnP.** Store-landmarks-then-track-them
is feature-based visual odometry — the front end of ORB-SLAM:

- **Keypoint detection**: ORB (FAST corners + oriented BRIEF descriptors,
  in stock OpenCV) finds a few hundred distinctive pixels per frame and
  gives each a 256-bit descriptor for re-finding it later.
- **Landmarks**: while the camera is known-static, each stable keypoint is
  back-projected through the depth image and K — the same pinhole maths as
  P2, inverted at a single pixel — into a 3D point in the map frame.
  Landmark = 3D position + descriptor.
- **Motion from re-observation**: a new frame's keypoints are matched to
  the landmark descriptors; each match says "this 3D point now appears at
  this pixel". Finding the camera pose that best explains all of them is
  the **Perspective-n-Point (PnP)** problem — `cv2.solvePnPRansac`, with
  RANSAC voting away the bad matches. The recovered pose *is* the camera
  movement: its translation is how far, its rotation how much it turned.
- Everything downstream stays honest about scale for free: the landmarks
  carry the neural depth's approximate scale, so PnP poses and the fused
  map live in the *same* metres-ish units — consistent even where not
  survey-grade.

No fusion or SLAM library doing the thinking; numpy and OpenCV primitives
only, same house rule as the hand-rolled projection in P2. RTAB-Map (built
and proven in the old plan's P3) stays available as a reference to compare
against and a fallback if the hand-rolled odometry cannot track this room.

## What it needs that the current pipeline already has

| Need | Have |
| --- | --- |
| Metric-ish depth aligned to RGB | `/depth` (32FC1) from `depth_estimator`, honest headers |
| Intrinsics, both directions | `/camera_info` (P0 approx K, fx = fy ≈ 907) |
| Keypoint machinery | OpenCV (already a dependency) — ORB, BFMatcher, solvePnPRansac |
| A pose reference to compare with | `rgbd_odometry` / `just map`, proven on `bags/static1` |
| Iterate without hardware | `just record` / bag replay; `bags/static1` has valid K |
| QoS for megabyte frames | RELIABLE / KEEP_LAST-1, as everywhere in this repo |

Constraints carried over: never gate on `header.stamp` age (the ~0.73 s
camera fault); depth is *relative* with hand-tuned `depth_scale`; the
C922's auto-exposure and rolling shutter fight feature matching, so the
exposure fix ([camera.md](../../info/camera.md#v4l2-controls)) matters from P3 on; everything
below runs on the dev box.

## The new nodes

Both in `piros2_perception`, alongside — not replacing — `cloud_projector`
(the per-frame cloud remains the debugging view).

**`cloud_fusion.py`** — the map:

- Subscribes the synced `/depth` + `/image_raw/compressed` pair
  (message_filters, as `cloud_projector` does) + `/camera_info`.
- Holds the grid as numpy arrays: `D` float32, `w` float32, colour
  3 × uint8. Origin/size/resolution are parameters in `perception.yaml`
  (2026-07-30: default re-aimed from the 2 m seed cube to 4 × 4 × 2.5 m
  at 4 cm — the desk scene's median ~3.2 scale-units fell outside the
  cube, so a live run visibly fused almost nothing; the seed-cube
  behaviour is unchanged, just bigger).
- Per frame, vectorised end to end: transform all cell centres into
  `camera_optical_frame`, project through K, sample the depth image,
  truncate, apply the update rule. No per-cell Python loops.
- Publishes `/map_points` (`PointCloud2`, near-surface cells with
  `w ≥ w_min`) from a ~1 Hz timer — extraction decoupled from fusion rate.
- Pose from tf2 (`Buffer` + `TransformListener`) at the frame's header
  stamp: `map_frame → camera_optical_frame`. With `map_frame: base_link`
  and a static camera that hits only static TF; pointing it at `odom`
  later picks up the odometry node with no code change.

**`keypoint_odometry.py`** — the poses (built in P3–P4):

- Subscribes the same synced pair + `/camera_info`; runs ORB per synced
  frame (~3 fps — slow deliberate motion is the regime anyway).
- Two modes, switched by its own state: **bootstrap** (camera assumed
  static — accumulate landmarks) and **track** (match → PnP → pose).
- Publishes the TF `odom → base_link` (`TransformBroadcaster` — the
  repo's first *dynamic* TF publisher), `/odom` (`nav_msgs/Odometry`),
  and `/features/preview/compressed` — the camera image with keypoints
  and match status drawn on it, the debugging window for everything here.
- The `base_link` pose comes from composing the PnP camera pose with the
  known static `base_link → camera_optical_frame` chain — TF composition
  done by hand once, then checked against `tf2_echo`.

## P0 — Scaffold and the seed cube

> **2026-07-30 — done, verified headless.** `cloud_fusion.py` exists:
> 100³-cell grid (1 M @ 0.02 m, ~32 MB with centres precomputed once),
> all parameters live in `perception.yaml`, tf2 Buffer + listener wired,
> synced-pair subscription in place, entry point in `setup.py`, node
> added to `perception.launch.py` and a `FusedMap` display to
> `perception.rviz`. Verified by running the node standalone and probing
> `/map_points`: 8000 seed-lattice points (20³ at stride 5), frame_id
> `base_link`, every point inside the configured cube, all in the faint
> seed grey. The RViz eyeball of the cube is still a human step —
> `just cloud` now shows it.
>
> **2026-07-30, later — seed display reshaped to a hollow shell.** The
> first RViz look showed the `[::stride]³` lattice reads as points
> *inside* the room — but an unobserved volume should render as its
> boundary only, like an empty room. `publish_map` now samples
> never-observed cells on the grid's six outer faces instead of
> throughout the volume; the seed test pins every published point to a
> boundary face. Fusion state is untouched — this is display-layer only.

`cloud_fusion.py` with the grid allocated, parameters declared, tf2
listening, subscriptions wired but fusion stubbed out. Unobserved cells
(`w = 0`) publish as a faint lattice so the initial state is *visible*: the
2 m cube, posed in front of the camera.

Parameters land in `perception.yaml` under `cloud_fusion:` —
`map_frame`, `grid_origin`, `grid_size`, `voxel_size`, `truncation`,
`w_max`, `w_min`, `change_threshold`, `reopen_frames`, `publish_rate`.

**Proves:** RViz (`just cloud` plus the new node, or a bag) shows the seed
cube in `base_link`, correctly posed via TF. `ros2 param list` shows the
knobs.

**Concepts:** tf2 buffer/listener and stamped lookups (the first dynamic
TF consumer in the repo), a node that owns long-lived state rather than
being a pure frame-in/frame-out filter.

## P1 — Fusion with a fixed camera

> **2026-07-30 — done, verified against `bags/static1`.** The projective
> update landed (transform → project → sample → truncate → weighted
> average, fully vectorised; quaternion→matrix hand-rolled). Measured on
> the bag run (grid overridden to cover the desk scene: 4 × 4 × 2.5 m at
> 4 cm = 620k cells): **29–101 ms per fusion pass**, ~200k cells updated
> per frame, weights climbing 1 → 35 over the 19 s bag (w_max 50 not
> reached — the bag is shorter than saturation at ~2 Hz synced pairs).
> Surface cells grew 1082 → 2034 as weights crossed w_min, median
> settling at (3.22, 1.58, −0.18) in `base_link` — consistent with P2's
> measured ~3.2 scale-units to the desk. The plan's persistence proof
> happened by construction: after the bag ended, the map republished
> **identical** count and median for 20+ s — state lives in the node,
> not the stream. Depth inference slowed to 342–704 ms/frame while
> sharing the CPU with fusion; expected, noted. Eight unit tests pin the
> mechanics (wall at 2 m converges within one voxel in 5 frames; free
> space stays at +truncation; occluded cells stay untouched; weight caps
> at w_max; the incremental pull shrinks per frame; colour averages to
> the wall's red; seed lattice before fusion, coloured surface after) —
> suite green at 34 tests. Still human: the RViz side-by-side
> (`/map_points` steady vs `/points` shimmering) and the live
> camera-restart check.
>
> **2026-07-30, later — first live RViz session.** Two findings. The
> "map switching between small and big" was two `cloud_fusion`
> instances on one topic — a scratch bag-test node had outlived its
> cleanup trap alongside the `just cloud` one (new troubleshooting
> entry). And the 2 m default grid missed the actual room: the desk
> scene's median ~3.2 scale-units sat outside it, so the defaults now
> cover 4 × 4 × 2.5 m at 4 cm (the bag-verified values). The RViz
> config now renders `/map_points` as voxel-sized Boxes and ships with
> `/points` off by default — toggle it on for the side-by-side.
>
> **2026-07-30, later still — depth moved to the GPU.** The note above
> measured depth inference degrading to 342–704 ms/frame while sharing
> the CPU with fusion; `todo.md`'s "reduce compute" answered by switching
> `depth_estimator` to `CUDAExecutionProvider` (GTX 1660 SUPER,
> `onnxruntime-gpu[cuda,cudnn]` in the venv — package README): 72–79
> ms/frame in-node (~13 fps), and fusion no longer competes with
> inference for cores. Synced pairs now arrive faster than the ~2 Hz
> quoted in these annotations — re-measure rates rather than reusing
> them. `perception.rviz` also gained two image sub-panels:
> `DepthPreview` (`/depth/preview/compressed`, live now) and `Keypoints`
> (`/features/preview/compressed` — blank until P3's
> `keypoint_odometry.py` publishes it).

Implement the projective update: transform → project → sample → truncate →
weighted average, with the camera on the desk and `map_frame: base_link`.
Iterate against `bags/static1` first (the milestone-6 loop), then live.

Even without motion this earns its keep: the neural depth flickers
frame-to-frame, and the running average converges to a **stable, denoised
surface** — watch the cube face get pushed back to the wall and stop
jittering, while `cloud_projector`'s per-frame cloud keeps shimmering next
to it.

Measure and record: ms/frame for the fusion pass (budget: tens of ms for
1 M cells — comparable to the 12 ms projection, and `todo.md`'s "reduce
compute" says stay honest here), frames-to-saturation at the default
`w_max`, point count published.

**Proves:** side-by-side in RViz, `/map_points` visibly steadier than
`/points`; killing and restarting the *camera* leaves the map standing
(state lives in the node, not the stream).

**Concepts:** the TSDF update rule met by hand; truncation (why only a
band around the surface is fused); why averaging in value space beats
moving points in position space.

## P2 — Saturation and re-opening

The static/dynamic split. Saturated cells (`w = w_max`) ignore lone
outliers; `reopen_frames` consecutive disagreements beyond
`change_threshold` reset `w` and let the cell re-learn. Tune the three
knobs against live disturbance.

**Proves:** with the map converged on the desk scene, wave a hand through
the view — the wall behind it does not erode. Then move an actual object
and hold it: within `reopen_frames`/fusion-rate seconds the old position
carves out and the new one solidifies. Both behaviours on camera, knobs
recorded in `perception.yaml` with comments.

**Concepts:** hysteresis as the answer to "static unless a large enough
change" — one threshold alone either smears moving things into the map or
lets noise chew the walls.

## P3 — Landmarks from a static camera

First half of `keypoint_odometry.py`: the bootstrap mode. Camera on the
desk, assumed static (the plan's stated starting assumption):

- ORB keypoints per synced frame; a keypoint must re-appear (descriptor
  match + same pixel ± tolerance) across `bootstrap_frames` consecutive
  frames to count — auto-exposure flicker and depth noise kill one-frame
  wonders.
- Each survivor is back-projected through the depth image (median of a
  small patch around the pixel, not the lone pixel) and K into a 3D point
  in `base_link`; its position is averaged over the bootstrap window.
- The landmark table (3D position float32 + 32-byte ORB descriptor,
  target a few hundred rows) is published as a `PointCloud2` for RViz and
  drawn on `/features/preview/compressed`.

Fix exposure and gain before measuring anything here — feature work is
where auto-exposure bites, per the house notes.

**Proves:** the preview shows keypoints pinned to real corners (monitor
edges, shelf brackets) and stable frame-to-frame; RViz shows the landmark
cloud embedded in P1's fused surface at the same depths. Recorded numbers:
landmark count, position jitter (std dev over the bootstrap window).

**Concepts:** what a feature detector actually finds and why corners
(aperture problem); binary descriptors and Hamming matching; back-projection
as the P2 pinhole equations run backwards at a single pixel.

## P4 — Motion from keypoints

Second half: the track mode, entered once bootstrap saturates.

- Per synced frame: detect, match against the landmark table (BFMatcher,
  Hamming, ratio test), feed the surviving 2D↔3D pairs to
  `cv2.solvePnPRansac` → the camera's pose in the map frame.
- Compose with the static `base_link → camera_optical_frame` chain and
  broadcast `odom → base_link`; publish `/odom` with the pose plus the
  **degree of movement** — translation in metres-ish and rotation in
  degrees since bootstrap, logged and drawn on the preview.
- Guard rails, stated as parameters: minimum inlier count to trust a pose
  (below it, hold the last pose and say so on the preview — don't
  hallucinate motion); reject poses that jump implausibly between frames.
- The camera *stays put* during this phase's first check: the published
  pose should sit at identity with millimetre-scale jitter — the noise
  floor, measured before any motion is attempted.

**Proves:** three staged checks, all on camera and against `tf2_echo
odom base_link`: (1) static camera → pose pinned at identity, jitter
recorded; (2) slide the camera ~20 cm along the desk edge (a ruler is the
ground truth) → reported translation within honest error, direction
correct; (3) return it to the start → pose returns near identity (closure
error recorded). RViz shows the camera frame moving through the landmark
cloud.

**Concepts:** PnP — recovering a 6-DoF pose from 2D↔3D correspondences;
RANSAC as voting against outliers; TF *broadcasting* and frame
composition; odometry as dead reckoning — error accumulates and nothing
here corrects it (that correction is what loop closure, RTAB-Map's whole
job, adds).

## P5 — Fusing from a moving camera

Point `cloud_fusion` at `map_frame: odom` and run both new nodes
together: the map now fuses each depth frame from wherever keypoint
odometry says the camera is. Grow the grid to room scale (e.g.
6 m × 6 m × 3 m at 4 cm ≈ 1.7 M cells) and feed a slow sweep —
`just record 45 sweep1`, then iterate on the bag.

The bootstrap landmark set only covers the initial view, so tracking dies
when the camera turns away. The fix is **landmark expansion**: when the
inlier count sags below a threshold, mint new landmarks from the current
frame's unmatched keypoints, back-projected through the current PnP pose
into `odom`. New landmarks inherit the current pose's accumulated error —
this is exactly how drift compounds in real visual odometry, and watching
it happen is the point. Cap the table and evict never-re-matched rows.

Known risks, stated up front: neural depth scale that varies with
viewpoint smears both the landmarks and the fused map; drift means the
sweep's end won't perfectly agree with its start. If a sweep won't
converge, the honest fallbacks, in order: fuse only from poses the
odometry reports with high inlier counts; substitute `rgbd_odometry`
(same TF contract, drop-in) to isolate whether odometry or fusion is the
problem; fall back to a handful of *held* poses as static transforms.

**Proves:** a recognisable multi-viewpoint room map in RViz from one slow
sweep — surfaces seen from several angles fused into one shell, not
ghosted copies — with the measured drift over the sweep recorded.
Comparison run: `just map` (RTAB-Map) on the same bag; note where each is
better and why.

**Concepts:** why a map needs poses; drift accumulation and why full SLAM
exists; keyframing/landmark maintenance as the difference between "PnP
against one snapshot" and odometry that survives a room.

## P6 — Repeatable and honest

- `just fuse [bag]` recipe (bag → republish → depth → odometry + fusion →
  RViz), with `pkill -f` cleanup traps per house rules; live variant
  alongside `just cloud`.
- Save/load: `~map/save` service writing the grid (and landmark table) to
  `.npz`; a loaded map re-publishes and keeps fusing — the map outlives
  the session.
- Unit tests in the existing no-weights style, synthetic scenes through a
  known K: a wall at 2 m converges to 2 m within N frames; weight caps at
  `w_max`; a lone outlier moves a saturated cell nothing; a sustained
  change re-opens and re-converges; synthetic keypoints back-project to
  known 3D positions; a known synthetic camera shift is recovered by the
  PnP path within tolerance. Anchored linters copied from an existing
  package; suite green in `just test` and the sidebar.
- Docs: measured fusion ms/frame, odometry accuracy from P4's staged
  checks, convergence times and knob values into
  [perception.md](../../info/perception.md); this plan annotated per phase and moved to
  `completed/` when done.

## Open decisions

| Question | Options | Lean |
| --- | --- | --- |
| Grid memory layout | dense numpy block vs sparse dict-of-blocks | Dense — 1–2 M cells fits easily; sparse only if room-scale resolution hurts |
| Feature detector | ORB vs GFTT + optical-flow (KLT) tracking | ORB — descriptors give re-findable landmarks, which the whole design leans on; KLT tracks but can't *recognise* |
| Odometry rate | synced pairs (~3 fps) vs RGB-only at 30 fps between pairs | Synced pairs first — simplest, and slow motion is the stated regime; RGB-only tracking is a P5 upgrade if 3 fps loses lock |
| Seed cube visual | publish `w = 0` lattice vs start empty | Publish it — the deforming cube is the point of P0/P1 |
| Colour fusion | weighted running average vs last-write | Running average, same weights as `D` |
| Landmark expansion policy | inlier-count trigger vs fixed keyframe spacing | Inlier trigger — expands exactly when tracking needs it |
