# Relocalization plan — remember the room, recover the pose

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

### P0 — the keyframe store

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

### P1 — orientation recovery (kp mode first)

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

### P2 — position too: 3D landmarks and the rgbd snap

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

### P3 — the room survives the session

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

### P4 — visibility and hygiene (optional)

RViz sees the map: keyframe poses as a MarkerArray (small axes at each
stored viewpoint, landmark points as a sparse cloud) — the debugging
view that makes "why didn't it relocalize" answerable. Store hygiene if
the live runs demand it: keep per-keyframe only landmarks re-observed
≥ k times, refresh a keyframe when the same view returns much sharper.
**Ends with:** a `Keyframes` display in `world_mesh.rviz`, off by
default; whatever hygiene landed is unit-tested.

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
- `/reset_odom_to_pose` semantics (frame, timing) need one live
  verification early in P2 before building on it.
- Compute: store matching runs on loss or every Nth frame, never per
  frame; P0's budget maths keeps the worst case bounded and the stats
  line makes the cost visible.