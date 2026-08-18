# Relocalization plan — remember the room, recover the pose

**Status (2026-08-18): all five phases built, unit-tested (35 new
tests, suite at 197) and live-verified as far as a static camera
allows — the whole plan in the drafting day's sitting.** What remains
is only the two live gates that need a human hand: the P1 flick test
(`just world_mesh odom:=kp`, pan hard away and back, watch for
`relocalized against keyframe N`) and the P2 cover-the-lens test
(default `just run`, occlude until rgbd resets, uncover on a stored
view, the mesh continues in place). P3's gate — save, restart with the
map, relocalize cold — passed live without a hand. Per-phase
annotations below; deviations from the spec are recorded after P2.

**Goal:** store the keypoint detections that matter, so that when the
camera flicks to a new direction and back — or tracking breaks for any
reason — the pose snaps back to what it was, instead of staying wrong
forever. Scope assumption, stated up front: **the camera stays in one
room**, so the whole map fits in a few megabytes and brute-force
matching stays cheap.

All work lands in `piros2_world_mesh` (and shared pure functions where
they belong), per the freeze convention — `piros2_world` is not
touched.

## Why: both pose sources are memoryless

Nothing in the session remembers the room beyond ~10 frames:

- **kp mode** (`keypoint_detector`): rotation is composed from strict
  *consecutive-pair* matches. A fast flick means motion blur and zero
  overlap — no pairs, no update — and the rotation spanning the flick
  is simply lost. When the camera settles on the old view, composition
  resumes *from a corrupted baseline*; the error is permanent. The
  `match_window` deque (10 frames ≈ well under a second) is display
  colouring, not memory.
- **rgbd mode** (the default): `rgbd_odometry` loses quality on the
  flick and `Odom/ResetCountdown: 1` resets it — to *identity*. The
  pose teleports to the origin, and the TSDF mesh starts integrating a
  second copy of the room on top of the first.

In SLAM terms (see the 2026-08-18 discussion recorded nowhere better
than here): the stack has odometry and mapping but no
**relocalization**. This plan builds exactly that piece — a
session-persistent landmark store and a recover-by-recognition path —
without pretending to be loop closure or pose-graph optimisation.

## Design decisions

- **What a stored keyframe is:** the ORB descriptors of one healthy
  frame, their pixel coordinates unprojected to unit bearing rays and
  rotated into the **odom** frame using the pose at capture time, plus
  (P2) per-keypoint 3D landmark points in odom, back-projected through
  K from a fresh `/depth`. Descriptors are the recognisers; rays/points
  are the geometry that turns recognition into a pose.
- **When to store:** novelty-gated. A frame becomes a keyframe when
  tracking is healthy *and* its view direction is far enough from every
  stored keyframe (`keyframe_novelty_deg`, ~15–20°) or its match ratio
  against the store is low. A room's worth of directions at 20° spacing
  is a few dozen keyframes; hard cap `keyframe_cap` (default ~100).
  Budget: 100 keyframes × 500 ORB descriptors × 32 B ≈ 1.6 MB — no
  bag-of-words needed at room scale, brute-force Hamming against a
  shortlist is the honest, teachable choice.
- **When to recover:** when consecutive-pair matching reports lost
  (`relocalize_after` frames without pairs, rgbd quality collapse), and
  as a cheap background check every Nth frame. Recovery = match current
  descriptors against the store (cross-checked, ratio-tested), take the
  best keyframe, then the **absolute** version of the maths already in
  the node: Kabsch between current-frame rays (camera frame) and stored
  rays (odom frame) gives orientation outright; with 3D landmarks (P2)
  the same SVD plus centroids (Umeyama, no scale) gives full 6-DoF.
- **Poses at capture come from latest-only TF lookups** — never
  TF-at-stamp, per the camera's 0.73 s stamp fault (the
  mapper/mesher/projector rule). Keyframes are captured when tracking
  is steady, so "latest" is honest.
- **The map inherits odom drift.** Keyframes are stored in the odom
  frame, so recovery returns the pose *as odom believed it when the
  keyframe was stored* — consistent with the mesh, which is the point.
  This is relocalization against a session-local map, not global
  consistency; drift between keyframe captures stays. Say so in every
  doc that mentions it.
- **Recovery fires after the flick settles**, not during it — motion
  blur kills ORB either way. The gate is "back to a known view", not
  "tracked through the blur".

## Phases

### P0 — the keyframe store ✓ 2026-08-18

`piros2_world_mesh/keyframe_store.py`: a pure-Python store (dataclass +
numpy arrays, no ROS imports) with `maybe_add(descriptors, rays_odom,
pose, …)` implementing the novelty gate and cap, and
`match(descriptors)` returning the best keyframe + matched index pairs.
`keypoint_detector` feeds it on healthy frames and reports store size
on the stats panel (one new line in the dashboard's block — arrival
counting untouched). Config lands in `world_mesh.yaml`
(`keyframe_novelty_deg`, `keyframe_cap`, `relocalize_after`).
`~/reset` clears the store along with the orientation.
**Ends with:** a live `just run` logging `keyframe 12 stored (yaw 63°,
store 12/100)` as the camera pans a room, and unit tests driving the
gate with synthetic descriptors (novel accepted, near-duplicate
refused, cap enforced, eviction policy exercised).

Built as specified (`keyframe_store.py`, novelty gate, nearest-view
replacement at the cap, margin-gated match; 9 unit tests) and verified
live the same day: a static rgbd session logged
`keyframe 0 stored (yaw 0°, store 1/100)` with no false captures and
the store count on the stats panel.

### P1 — orientation recovery (kp mode first) ✓ built 2026-08-18 — live flick gate open

Loss detection in the detector (no consecutive pairs for
`relocalize_after` frames), then store lookup and absolute Kabsch:
current rays in the camera frame against the matched keyframe's rays in
odom — the composed-orientation state is *replaced*, not incremented,
and the recovery is logged loudly (`relocalized against keyframe 4,
correction 38°`). Runs under `just world_mesh odom:=kp`.
**Ends with:** the flick test passes live — note a landmark's position
in RViz (axes display), flick the camera 90° away and back fast, and
the axes return to within a few degrees of where they started (today
they provably don't); unit tests build a store from synthetic bundles
at known rotations, corrupt the composed state, and recover it exactly.

Built and unit-tested (synthetic landmark fields at known rotations:
capture stores rays in odom exactly, a corrupted compass recovers to
<0.01°, unknown views are refused, loss arms recognition, `~/reset`
clears the memory). The **live flick test needs a human hand** — run
`just world_mesh odom:=kp`, pan hard away and back, watch for the
`relocalized against keyframe N` log and the axes returning.

### P2 — position too: 3D landmarks and the rgbd snap ✓ built 2026-08-18 — live cover-the-lens gate open

Keyframes additionally store 3D landmark points: captured only when a
fresh `/depth` (≤ ~0.5 s old, latest-wins sub — the room is static and
capture happens while steady) covers the frame; keypoint pixels sample
the depth image, back-project through K, transform to odom by latest
TF. Recovery solves 6-DoF: matched pairs of current-frame 3D points
(camera frame, from current depth) against stored odom points —
Kabsch + centroid translation as a shared pure function (`se3.py` gains
`rigid_transform_3d`, unit-tested on synthetic clouds both forks can
use). In rgbd mode the detector doesn't own TF, so the snap is
delivered to the owner: call RTAB-Map's **`/reset_odom_to_pose`**
(confirmed live on the session graph 2026-08-16) with the recovered
pose — rgbd resumes from the right place and the TSDF keeps
integrating one room instead of two.
**Ends with:** under default `just run`, cover the lens (or flick hard)
until rgbd resets, uncover on the old view — the mesh continues *in
place* (today it teleports to the origin); the recovered-pose error
against pre-flick is eyeball-small in RViz and the correction is
logged. Unit tests: `rigid_transform_3d` recovers known SE(3) on
synthetic clouds with noise and outlier rejection.

Built (`rigid_transform_3d` in `se3.py` with noise/outlier/refusal
tests; depth-sampled 3D landmarks; the discrepancy gate) and the
plan's named risk was verified live 2026-08-18: calling
`/reset_odom_to_pose` with (1.0, −0.5, yaw 0.8) made `odom →
base_link` exactly that pose — the service takes precisely the
`T_odom_base` our snap computes, in the odom frame. A bonus artifact
of that test proved the capture path end-to-end: after the synthetic
teleport the static scene was re-stored as `keyframe 1 (yaw 46°)`,
i.e. capture demonstrably reads the live TF. The **cover-the-lens
gate needs a human**: under default `just run`, occlude until rgbd
resets, uncover on a stored view, and the mesh should continue in
place with a `snapping odometry` log.

**Where the build deviated from the spec** (all deliberate, none
regressions):

- The "cheap background check every Nth frame" from the design
  decisions was **not** implemented — recovery runs only while lost,
  with retries on a `relocalize_retry` cadence (a full store query is
  tens of ms). A healthy-tracking background verify would mean
  auto-snapping against normal drift, which is a policy question P4 can
  take up with live evidence; until then, `~/reset` remains the drift
  answer.
- rgbd loss is detected by the detector's *own* pair-loss signal, not
  by watching rgbd's quality (no OdomInfo subscription): same camera,
  same blur, one detector. What protects a healthy rgbd from a spurious
  snap is the discrepancy gate — recovered-vs-live pose within
  `min_correction_m`/`min_correction_deg` means "recognised, no snap".
- Depth freshness for landmark capture is `depth_max_age: 1.0` s (the
  spec sketched ~0.5 s): at the paced 5 Hz pipeline, 0.5 s left too few
  frames depth-eligible; 1.0 s on a static-scene capture is honest.
- The config surface grew beyond the P0 sketch:
  `relocalize_retry/min_pairs/margin`, `depth_max_age`,
  `min_correction_m/deg` — all in `world_mesh.yaml` with the reasoning
  in comments.

### P3 — the room survives the session ✓ 2026-08-18

Persistence: `~/save_map` writes the store to `maps/room_<stamp>.npz`
(git-ignored; numpy arrays — descriptors, rays, points, poses — no
pickle), and a `map_path` parameter loads one at startup. A cold-start
session with a loaded map relocalizes before it has any odometry
history: the first successful recovery *defines* where odom is relative
to the stored room — the map's frame wins, which is what makes saved
maps meaningful.
**Ends with:** run a session, save, restart pointing anywhere in the
room — the pose lands consistent with the previous session's map
without moving the camera; round-trip unit test (save → load →
identical store, matching still works).

Built and **gate passed live** the same day: `~/save_map`
(`just map-save`) wrote `maps/room_20260818-170003.npz` (18 KB for one
keyframe — plain arrays, `allow_pickle=False` round-trips it), and a
cold-started `just run map_path:=…` on the same static scene logged
`loaded 1 keyframes … relocalizing before trusting any pose` and, 0.8 s
later, `relocalized against keyframe 0: adopting the map's frame` — the
snap fired *before rgbd had produced any odom*, so the map's frame won
by construction, exactly the semantics the spec asked for. That
ordering also exposed a wording bug (the log printed `Δ inf m`); the
cold-start case now says what it is. `map_path` is a launch argument
(`just run` passes args through), `maps/` is git-ignored.

### P4 — visibility and hygiene (optional) — visibility ✓ 2026-08-18, hygiene deferred

RViz sees the map: keyframe poses as a MarkerArray (small axes at each
stored viewpoint, landmark points as a sparse cloud) — the debugging
view that makes "why didn't it relocalize" answerable. Store hygiene if
the live runs demand it: keep per-keyframe only landmarks re-observed
≥ k times, refresh a keyframe when the same view returns much sharper.
**Ends with:** a `Keyframes` display in `world_mesh.rviz`, off by
default; whatever hygiene landed is unit-tested.

Visibility landed: `keyframe_marker()` (pure function, unit-tested)
draws every stored viewpoint as a short cyan stroke — from the stored
pose along the view direction for rgbd keyframes, from the origin for
kp-mode ones — on `/world/keyframes` (latched, 2 s timer), shown as the
`Keyframes` display in `world_mesh.rviz`, off by default; verified
publishing live in `odom`. Hygiene (re-observation counts, sharper
refresh) is **deferred by decision** until the live gates produce
evidence it is needed — a novelty gate plus nearest-view replacement
has not yet shown a store-quality problem on the static runs.

## Risks and honesty

- ORB under motion blur recovers nothing *during* the flick — by
  design; the promise is recovery after settling, typically within
  `relocalize_after` frames + one match (~1 s at the paced 5 Hz).
- A wrong relocalization is worse than none: matches must clear a
  margin (best-vs-second-best keyframe score, minimum inlier pairs
  after the robust refit) before any snap fires; below it, log and keep
  waiting. The thresholds are parameters, tuned in P1/P2 live runs.
- Repetitive texture (two identical posters) can alias keyframes —
  room-scale reality check, not solved here; the margin test is the
  mitigation.
- ~~`/reset_odom_to_pose` semantics (frame, timing) need one live
  verification early in P2 before building on it.~~ **Retired
  2026-08-18**: called live with (x 1.0, y −0.5, yaw 0.8);
  `odom → base_link` became exactly that pose — the service takes
  precisely the `T_odom_base` the snap computes, in the odom frame.
- Compute: store matching runs on loss or every Nth frame, never per
  frame; P0's budget maths keeps the worst case bounded and the stats
  line makes the cost visible.