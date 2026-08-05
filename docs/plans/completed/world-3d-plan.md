# World 3D build plan — orientation and a cloud map in RViz

> **Completed 2026-08-05.** All four build phases landed 2026-08-04 (P0
> and P1 live-verified the same day), the P2/P3 live sweep passed
> 2026-08-05, and P4's bookkeeping moved this file to `completed/`.
> Kept as the build log; per-phase annotations record what actually
> happened.
>
> *Later the same day, the
> [world combined plan](world-combined-plan.md) merged this plan's
> session into `just world`: `world3d.launch.py`, `world.rviz` and the
> `just world3d` recipe named below no longer exist — their node set
> lives in `world.launch.py`, their displays in `orient.rviz` and
> `map.rviz`. References below are historical.*

> Two 3D displays on top of `src/piros2_world`: a live **camera orientation**
> estimated from the keypoint matches (start at identity; rotate the camera
> and the displayed axes rotate with it), and a **cloud map** — the depth
> clouds accumulated in the oriented frame, so a slow pan paints the room
> into a persistent panorama. Stable phases, each ending with something you
> can run and check before the next starts.

## The honest scope: orientation, not position

A single camera with no IMU can measure *rotation* well and *translation*
only up to an unknown scale — and this plan's use case (camera on a desk or
in a hand, turning to look around) is precisely the case where translation
information vanishes entirely. That kills the textbook tool: the essential
matrix (`cv2.findEssentialMat` → `recoverPose`) is **degenerate under pure
rotation** — with zero baseline there is no epipolar geometry to estimate,
and the recovered translation is noise.

So the estimator is rotation-only, and simpler than the textbook route:
unproject each matched pixel pair to unit **bearing rays** through K (from
`/camera_info` — the P0 approximate intrinsics are live), then solve the
orthogonal Procrustes problem (Kabsch: one SVD) for the rotation that maps
last frame's rays onto this frame's, with a couple of reject-worst-and-refit
rounds standing in for RANSAC. Composing the per-frame rotations integrates
to a running orientation. This is a *compass built from pixels*, and it
drifts — every frame's small error accumulates, there is no loop closure —
which is why the node gets a reset service, and why walking around while
trusting it is out of scope. Position, scale, and real SLAM stay with
RTAB-Map (`just map`), which exists for exactly that.

The cloud map inherits the same honesty: with orientation but no position,
accumulated clouds form a **panorama from one viewpoint**, not a walkable
map. That is still a genuine 3D map of everything visible from the desk —
and the plan says so plainly rather than implying more.

## Preconditions, verified

> Re-checked 2026-08-04 evening, after the day's detector work: matching
> runs against a 10-frame `match_window` pool and holds ≈90% matched at
> `max_features: 500` (the cap was briefly 100 for readability, restored
> once green/yellow colouring made density legible). The dashboard shows
> the match rate as a percentage; the wire keeps raw Int32 counts —
> which is what P0 consumes.

| Have | Where |
| --- | --- |
| Keypoint matching (green/yellow, `/keypoints/matched`; a 10-frame `match_window` since 2026-08-04 — P0's estimator needs strict *consecutive-pair* matches, computed separately from the display matching) | `keypoint_detector.py`, 2026-08-04 |
| K on `/camera_info` (approx intrinsics, fx ≈ 907 px) | perception P0, `c922_720p_approx.yaml` |
| Live point cloud, correctly posed, verified in RViz | `cloud_projector`, `just cloud` (perception P2) |
| RViz on the dev box, with its Wayland/GL env pins | `just cloud`, [troubleshooting.md](../../info/troubleshooting.md) |
| Static TF chain `base_link → camera_link → camera_optical_frame` | `camera.launch.py` (milestone 5) |
| Venv `ExecuteProcess` pattern for the depth node; no camera include | `perception.launch.py` rules |

Constraints that shape the phases:

- **Duplicate frames carry zero rotation.** usb_cam's 60 Hz grab timer
  republishes each frame ~twice; an identical pair must be *skipped*, not
  fed to the estimator (it would just dilute the average with identity
  votes). The JPEG bytes of a re-published frame are identical, so a cheap
  hash of `msg.data` detects them — this finally lands part of the standing
  "reduce compute" todo.
- **Header stamps stay untrusted.** The camera's ~0.73 s stamp fault means
  the mapper transforms clouds with the *latest* orientation, not a
  stamp-matched TF lookup. At hand-rotation speeds the smear is small; the
  plan notes it rather than pretending stamps work.
- **The dynamic TF must not fight the static one.** The camera launch owns
  `base_link → camera_link → camera_optical_frame`. Orientation is
  published as a new **`odom → base_link`** transform (REP-105's slot for
  drifting local odometry), leaving the static chain untouched — RViz set
  to fixed frame `odom` then shows the whole camera rig rotating.
- **RViz runs from the recipe, not the launch file** — it needs
  `QT_QPA_PLATFORM=xcb` (and the mesa fallback pins until the next reboot),
  same as `just cloud`. Cleanup by `pkill -f`, never `kill %N`.
- Only compressed topics cross the Wi-Fi; every new subscription is
  RELIABLE/KEEP_LAST-1 (`BIG_FRAME_QOS`).

## Changes to the package

```
src/piros2_world/
├── piros2_world/
│   ├── keypoint_detector.py       # P0: + rays→R estimation, dup skip, PoseStamped, reset service
│   │                              # P1: + odom→base_link TransformBroadcaster
│   └── cloud_mapper.py            # P3: NEW — accumulate /points into /world/map_points
├── config/
│   ├── world.yaml                 # new params per phase
│   └── world.rviz                 # P1: NEW — axes + image; P2 adds the cloud displays
├── launch/world3d.launch.py       # P2: NEW — detector + depth (venv) + projector (+ mapper in P3)
└── test/
    ├── test_keypoint_detector.py  # P0: + estimator pure-function tests
    └── test_cloud_mapper.py       # P3: NEW — synthetic-cloud accumulation tests
```

The estimator lives *inside* `keypoint_detector` rather than a new node: it
already holds the previous frame's keypoints and descriptors, and shipping
matched pixel pairs between nodes would need a custom rosidl message — the
same trade that put the counts on plain Int32.

## P0 — Rotation from matched rays ✓ (2026-08-04; live pan check passed same day via `just orient`)

> Landed as planned, in `keypoint_detector.py`: CRC dup-skip (also part
> of the "reduce compute" todo), cached K with the zero-K bag guard, the
> pure functions (`rays_from_pixels`, `kabsch`, `estimate_rotation`,
> `quaternion_from_rotation`), the `~/reset` Trigger service, and both
> gate params in `world.yaml`. One decision the plan left open, now
> fixed: the published pose is already conjugated into **base_link
> axes** (`BASE_FROM_OPTICAL` — a pan arrives as yaw-about-z-up), so
> P1's TF broadcast is a transcription, not more geometry. Verified
> against `bags/static1`: 196 poses, K cached from the bag (fx = 907),
> orientation held within ~0.001° of identity over the full replay,
> 19.8 ms/frame with estimation running. 12 new unit tests; suite at 59
> green. Remaining: the hand-pan runnable check below.

Extend `keypoint_detector.py`:

- Skip duplicate frames by hashing `msg.data`; count skips in the log line.
- Subscribe `/camera_info` once for K (cache it; no K yet → detect and
  publish as today, log a throttled warning, estimate nothing).
- Keep the previous frame's matched keypoint coordinates alongside the
  descriptors. Per matched pair: pixel → normalised ray via K⁻¹.
- `estimate_rotation(prev_rays, curr_rays)` as a **pure function** (Kabsch
  SVD, `det = +1` reflection guard, 2–3 reject-worst-and-refit rounds,
  minimum-pair and residual sanity gates → returns `None` when the match
  set is too thin or inconsistent to trust).
- Compose into a running orientation; publish `geometry_msgs/PoseStamped`
  on `/camera/orientation` (position zeros, `frame_id: odom`, stamp from
  our own clock — the source stamps are faulty and this pose is a
  *now*-estimate anyway).
- A `~/reset` service (`std_srvs/Trigger` — the repo's first service) to
  re-zero the orientation without restarting.
- New `world.yaml` params: `min_matched_pairs`, `max_residual_rad`.

Unit tests, no hardware: rotate a synthetic ray bundle by a known R and
recover it; verify the reflection guard, the `None` gates, and that a
duplicate frame publishes no pose.

**Runnable check:** `just world` running; `ros2 topic echo
/camera/orientation` shows a quaternion near identity while the camera is
still, sweeping smoothly as it is panned by hand ~90° and roughly returning
when panned back (drift visible and expected). Reset service re-zeros it.

## P1 — Orientation in 3D: TF + RViz axes ✓ (2026-08-04; live RViz check passed same day — axes track the hand-held camera)

> Landed as planned: a `TransformBroadcaster` in the same node sends the
> pose's quaternion as `odom → base_link` with zero translation (same
> own-clock stamp — one estimate, two representations),
> `config/world.rviz` (fixed frame `odom`, TF axes, the
> `/keypoints/compressed` panel — the compressed-topic Image display
> copied from the verified `perception.rviz`), and `just orient` on the
> `just cloud` recipe pattern (SSH camera + warm-up health check +
> `pkill -f` cleanup + the `QT_QPA_PLATFORM=xcb` pin). Verified over the
> `bags/static1` replay: `tf2_echo odom base_link` resolved 11/11
> lookups, translation zero, rotation identity. Suite at 60 green.
> Remaining: the RViz axes check below (needs a hand on the camera).

- Broadcast the same rotation as an `odom → base_link` transform
  (`tf2_ros.TransformBroadcaster`, same node).
- `config/world.rviz`: fixed frame `odom`, TF display (axes for `odom` and
  the camera chain), plus the annotated keypoint image panel for context.
- `just orient` recipe: camera over SSH, the detector, RViz with the env
  pins; verify-survived-warm-up and `pkill -f` cleanup per the camera
  rules.

**Runnable check:** the first requested display exists — RViz axes start at
the default orientation and *tilt and pan live* as the camera is moved by
hand. `ros2 run tf2_ros tf2_echo odom camera_optical_frame` agrees.

## P2 — The live cloud, oriented ✓ (2026-08-04; live sweep check passed 2026-08-05)

> Landed as planned: `world3d.launch.py` (detector + venv depth
> estimator + cloud projector, camera stays with the Pi), `/points` in
> `world.rviz`, `just world3d` on the same recipe skeleton as `orient`.
> Bag plumbing check: over the `bags/static1` replay all three nodes ran
> together — `/points` at 14.9 Hz (the bag's own rate; the GPU estimator
> keeps up), 41–46k points per cloud in ~6 ms, `odom → base_link`
> resolving identity throughout. Remaining: the live pan check below.

- `launch/world3d.launch.py`: detector + depth estimator (venv
  `ExecuteProcess`) + `cloud_projector`, no camera include (ownership
  rules); this is `world.launch.py`'s pattern with the projector added.
- `world.rviz` gains a PointCloud2 display on `/points`.
- `just world3d` recipe replaces `just orient`'s node set with the launch.

Nothing new is computed in this phase — it is plumbing that makes the
existing live cloud render *in the oriented frame*: with fixed frame
`odom`, TF rotates `base_link` and the cloud sweeps around the axes as the
camera pans. That free win is the point of choosing `odom → base_link` in
P1.

**Runnable check:** pan the camera slowly; the live cloud swings through
the RViz world instead of staying glued to the view axis, and the wall that
was ahead stays roughly where it was painted when the camera looks away and
back (modulo drift).

## P3 — The cloud map: accumulate ✓ (2026-08-04; live sweep check passed 2026-08-05 — the panorama widens and persists)

> Remaining slow-pan check passed 2026-08-05: the live cloud swings
> through the RViz world, the map widens with the pan and persists when
> the camera looks away and back.
>
> Landed as planned: `cloud_mapper.py` — the repo's first TF *consumer*
> (Buffer + TransformListener, latest-only lookups per the stamp
> constraint), a pure `VoxelMap` class holding the accumulation
> semantics, `~/clear`, and all four params in `world.yaml`. The wire
> format is imported from `cloud_projector` (`POINT_DTYPE`) rather than
> duplicated — the contract can't drift. Joined `world3d.launch.py` and
> `world.rviz` (`/world/map_points`); 9 unit tests (suite at 69).
> Bag check matched the phase's own prediction: static replay → map
> republishes at 0.999 Hz and converges to 42k voxels against ~45k-point
> live clouds — camera still, map ≈ live cloud. Remaining: the slow-pan
> widening check below.

`cloud_mapper.py`, the second requested display:

- Subscribe `/points`, transform each cloud by the *latest* orientation
  (see the stamp constraint), and accumulate into a voxel dict —
  `floor(xyz / voxel_size)` → colour, latest wins. A dict, not a list:
  bounded by scene volume rather than by runtime, and revisited voxels
  update instead of duplicating.
- Republish the map as one `PointCloud2` on `/world/map_points` at ~1 Hz
  (the numpy structured-array wire format from `cloud_projector`).
- Params: `voxel_size` (default 0.05 m), `map_publish_rate`, `max_voxels`
  (hard cap; when hit, log once and stop growing — no silent eviction).
  A `~/clear` Trigger service mirrors P0's reset.
- Depth beyond a `max_range` param is dropped — monocular depth degrades
  with distance, and far guesses would smear the panorama.
- Unit tests with synthetic clouds: two overlapping clouds under known
  rotations merge to the expected voxel set; the cap and clear behave.

Joins `world3d.launch.py`; `world.rviz` gains `/world/map_points`.

**Runnable check:** with the camera still, the map equals the live cloud;
a slow 90° pan *widens* it — the painted region persists after the camera
looks away. `just world3d`, one command.

## P4 — Bookkeeping ✓ (2026-08-05)

Docs map + current-state notes in `CLAUDE.md` and `README.md`, a line in
[roadmap.md](../../info/roadmap.md), `.vscode/tasks.json` already knows the
package, suite green, and this file moves to `docs/plans/completed/` —
the move *is* the status change; fix inbound links.

## Out of scope, recorded so nobody wonders

- **Translation, scale, 6-DoF pose** — needs parallax + a scale source
  (IMU, stereo, known geometry). RTAB-Map (`just map`) is the tool when
  position matters; this plan's odometry is rotation-only on purpose.
- **Drift correction / loop closure** — the reset services are the drift
  strategy. Anything smarter is SLAM, see above.
- **Gyro/IMU fusion** — no IMU is attached; milestone 7 hardware territory.
- **Serving RViz displays into the 2D dashboard mosaic** — stays out, same
  reasoning as the world plan: 3D lives in RViz.
- **Map persistence to disk** — the map lives and dies with the node; bag
  `/world/map_points` if a session is worth keeping.
