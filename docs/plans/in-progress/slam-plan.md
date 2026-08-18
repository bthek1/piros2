# SLAM plan — what `world_mesh` is missing before it can call itself SLAM

**Status (2026-08-18, late night): P0–P2 done and gated, P3 built with a
provisional gate, P4 built and unit-tested with its gate written but
unrun.** Written the evening the question "what's needed to make
world_mesh a SLAM project?" was asked; most of it built the same night.
The answer, in one line, was: **the fork had a front-end and a map, but
no backend** — nothing detected a revisit while tracking was healthy,
nothing optimised the trajectory when one was found, and the map could
not be corrected afterwards because the TSDF integrates poses
destructively. As of tonight the fork *has* a backend: always-on
loop-closure detection from its keyframe store (P1), a hand-written
SE(3) pose-graph optimiser publishing `map → odom` (P2, checked
against the installed `g2o`), and a TSDF that rebuilds from its frame
memory when the graph moves (P3). Measured against RTAB-Map on the same
replays: loop gap 6.1 cm / 1.9° → 2.3 cm / 0.85° (RTAB-Map 0.7–1.4 cm /
0.6–1.6°), fr1/desk ATE 0.163 → 0.089 m (RTAB-Map 0.212 → 0.096 m).
**The claims have not flipped yet** — the repo's stated scope, the
`SLAM` GitHub topic and `docs/to_learn/emescent.md`'s "NOT SLAM" line
wait for P4's gate (`just gate-map`) to pass and P3's metric to be made
credible; see the annotations below.

All work lands in `piros2_world_mesh` (and `piros2_perception` /
`tools/` where a piece belongs there), per the freeze convention —
`piros2_world` is not touched.

## What SLAM actually requires

The definition worth holding: **estimate the sensor's trajectory and a
map of the environment *jointly*, so that recognising a place you have
been before corrects both the trajectory and the map** — global
consistency, not just local accuracy. Concretely, a system is SLAM when
it has all five of:

| Piece | Job | Typical form |
| --- | --- | --- |
| **Front-end odometry** | pose increments frame to frame | visual/RGB-D/LiDAR odometry, IMU preintegration |
| **Map** | the environment representation being built | keyframes + landmarks, TSDF, occupancy grid, point map |
| **Place recognition / loop-closure detection** | "I have seen this before" — *while tracking is fine*, not only after it breaks | bag-of-words / descriptor matching + geometric verification |
| **Backend** | fold the loop constraint into *every* past pose | pose-graph optimisation (g2o, GTSAM, Ceres), bundle adjustment |
| **Map correction** | make the map agree with the optimised trajectory | re-integration, submap re-posing, deformation graph |

Odometry alone is dead reckoning. Odometry + a place memory that resets
the pose on loss is *relocalization* (localisation against your own
recent map). Both are ingredients; neither is SLAM. That is exactly
where the fork stands.

## What `world_mesh` has today (measured, not aspirational)

| SLAM piece | In the fork | Honest grade |
| --- | --- | --- |
| Front-end odometry | `rgbd_odometry` (RTAB-Map, 6-DoF, `odom:=rgbd` default) over the mono depth net's `/depth` + `/depth/rgb` twin at 5 Hz; or the `keypoint_detector` rotation-only compass (`odom:=kp`) | ✓ have it. Weakness: monocular depth — scale from `depth_scale: 2.69` (tape-measured), ±4 % per-frame wobble (high-passed by `ScaleAligner`, not removed); rgbd resets to identity on loss (`Odom/ResetCountdown: 1`) |
| Map | `tsdf_mesher`: open3d `VoxelBlockGrid`, 1.5 cm, integrated at the *latest* TF, re-meshed every 15 s, mesh-completion pass, `~/save` PLY | ✓ have it — but the pose is baked into every voxel at integration time; **there is no way to move what has been integrated** |
| Place recognition | `KeyframeStore` (novelty-gated ORB keyframes, brute-force cross-checked Hamming, margin test) + `attempt_relocalization` | ◐ half. Queried **only after `relocalize_after` frames of lost tracking**. A healthy return to a seen wall is never noticed, so drift accumulated on the way is never measured |
| Backend | none. A relocalization *snaps* the current pose (kp: overwrite orientation; rgbd: `/reset_odom_to_pose`) — a hard reset of the present, no correction of the past | ✗ |
| Map correction | none. After a snap the surface integrated under the drifted poses stays where it was; new integration lands on the corrected pose, so the two disagree (the "layered shingling" the fusion plan photographed is the same disease from the other side) | ✗ |
| Frames | `odom → base_link` only (REP-105 says a SLAM system owns `map → odom`, the *correction* frame, and leaves odom continuous) | ✗ no `map` frame |
| Trajectory output | none published (`/camera/orientation` is a pose, not a path) | ✗ |
| Persistence | `maps/room_<stamp>.npz` keyframes (descriptors, rays, 3D landmarks, capture poses); `meshes/live_<stamp>.ply` | ◐ keyframes save, but no edges/graph, no per-keyframe depth to rebuild the map from |
| Verification | `just run-bag`, gate bags + `just gate`, `just snap`, `just mesh-views`; `just map-headless` already exports RTAB-Map's odometry *and* graph-optimised poses (`bags/<name>_odom.txt` / `_slam.txt`, TUM form); `just fetch-tum` holds fr1/desk with real ground truth | ✓ the yardstick infrastructure exists — this is the fork's real asset |
| Sensors | one C922 over Wi-Fi + a depth net. No IMU, no wheel odom, no LiDAR | fixed — every phase below is visual-only |

Also already on the box, unused by the fork: **`g2o` (CLI + libs) and
GTSAM 4.2** as rtabmap dependencies (`ros-jazzy-libg2o`,
`ros-jazzy-gtsam`), and RTAB-Map's own SLAM node `rtabmap` (used by
`mapping.launch.py` / `just map`, never by the world session).

## The two honest routes — and why this plan takes both

**Route A — put a finished SLAM in the loop.** Add RTAB-Map's `rtabmap`
node to `world_mesh.launch.py` (it already runs `rgbd_odometry`; the
SLAM node consumes the same synced pair, does bag-of-words loop closure
and g2o/GTSAM graph optimisation, and publishes `map → odom`). Cheap —
one Node block plus params. Also teaches almost nothing about *why* it
works, and the TSDF still cannot be corrected.

**Route B — build the backend ourselves.** Loop detection from the
`KeyframeStore` we already have, a hand-rolled pose-graph optimiser on
top of `se3.py`, and a TSDF that rebuilds from stored keyframes when
the graph moves. This is the learning project's point.

This plan does **A first, as the yardstick, then B as the build** — the
same shape the fusion plan used (RTAB-Map's poses versus ours). RTAB-Map
gives every later gate a reference trajectory on the *same* bag; a
hand-rolled optimiser gets checked against `g2o` on the *same* graph
file. Neither route needs a person once the loop bag exists (P0).

## Phases

Each phase ends with something runnable and a check a script closes
(verification.md's rule). Phase numbers are fixed once written; progress
is annotated, never renumbered.

### P0 — A loop to close, and the yardstick — ✓ done 2026-08-18 (evening)

**What happened.** No existing bag looped (`sweep3`'s RTAB-Map graph
poses equal its odometry to rounding; `sweep1` is empty), so the loop
bag became a *palindrome*: `make_gate_bag.py loop` plays `sweep3`
forward then reversed (header timeline mirrored so image/camera_info
stay stamp-paired) — the camera retraces to its start view with real
accumulated drift and every return frame has its outbound twin as
reference. `bags/gate_loop` (88.5 s, 5262 frames), built by
`just gate-bags`. `world_mesh.launch.py` gained `slam:=off|rtabmap`
(RTAB-Map's `rtabmap` node on the same synced pair, owning
`map → odom`) and `depth_source:=estimator|external`. New tools:
`traj_record.py` (odom → base_link, map → odom, Path topics → TUM
files), `traj_check.py loop|ate` (Umeyama, interpolation, the
verdicts), `tum_player.py` (fr1/desk as camera + depth + static TF).
Recipes `just gate-loop [rtabmap|off]`, `just gate-tum [rtabmap|off]`.

**Measured.** `gate-loop rtabmap`: raw odometry loop gap 7.1 cm / 19.0°
(second run 14.6 cm / 5.0° — the pipeline is not deterministic under
load), RTAB-Map's optimised graph 1.4 cm / 1.55° (0.7 cm / 0.58°),
`loops=9` in its DB — **PASS**; `gate-loop off`: **FAIL** ("no loop
closed"), the control. `gate-tum rtabmap` on fr1/desk: ATE RMSE
odometry 0.212 m → optimised 0.096 m over 184 poses / 19.8 s (`loops=6`)
— **PASS**. Two findings for the record: RTAB-Map's `/mapPath` poses
carry no per-pose stamps (all share the header's), so the optimised
graph is read from its DB via `rtabmap-report --poses_raw` after the
node is stopped; and that DB's per-node *odometry* is re-based on every
`rgbd_odometry` auto-reset (four in the loop run), so a correction must
be `optimised ∘ tf_odom(t_k)⁻¹` from the recorded TF, never from the
DB's own odometry column (the first scoring did that and read
"corrected = raw"). Yardstick for P2: **≤ ~1.5 cm / 1.5° loop gap,
ATE ≤ ~0.10 m on fr1/desk.**

You cannot test loop closure on a bag that never returns anywhere.

- **The loop bag.** `bags/sweep3` (the mesh-completion sweep) may or may
  not revisit its start — check with `just map-headless bags/sweep3`
  and look at `sweep3_slam.txt` versus `_odom.txt`: RTAB-Map's graph
  had 12 nodes and no reported closure. If it doesn't loop, this is
  the plan's **one "needs a human"**: record `just record 60 loop1` —
  walk the camera around the room with real translation and end on
  the view you started on, holding it 5 s. One recording turns every
  gate below into a replay. (`make_gate_bag.py`'s A → … → A′ splice
  gives a *fake* loop — the pipeline's own A poses as reference — and
  is a fine smoke test, but a spliced bag has zero drift by
  construction, so it can't show a closure *correcting* anything.)
- **Route A in the session.** `world_mesh.launch.py` gains
  `slam:=rtabmap|off` (default `off` — the session stays as it is):
  RTAB-Map's `rtabmap` node on the same `/depth/rgb` + `/depth` +
  `/camera_info` sync, `frame_id: base_link`, `map_frame_id: map`,
  publishing `map → odom` and `/rtabmap/mapPath`. `world_mesh.rviz`
  gets a `Path` display. Recipe trap already kills `rtabmap`.
- **The measurement.** A new `tools/verify/traj_check.py`: reads two
  TUM-form pose files, associates by stamp, aligns (Umeyama — `se3.rigid_transform_3d` is the fit;
  reuse it), and prints
  ATE RMSE + start-vs-end error. `just gate loop` = `run-bag
  bags/loop1 slam:=rtabmap` headless → dump `/rtabmap/mapPath` and the
  raw odometry → `traj_check` → **PASS if optimised start-end error <
  odometry's**, printing both numbers. `poses.png` as the picture.
- **Ground truth, not just a reference.** A `tum_player` (tools/verify
  or the fork; venv-free) publishes a TUM sequence's rgb + depth as
  `/depth/rgb` + `/depth` (32FC1 metres, identical stamps — the
  estimator's contract) + `/camera_info` from the fr1 intrinsics, so
  the *whole* downstream (rgbd_odometry, keypoint_detector, mesher,
  everything after this plan) runs on real depth against real ground
  truth. `just gate tum` = `traj_check` against `groundtruth.txt`.
  fr1/desk is already fetched (`just fetch-tum`); fr1/room has the
  bigger loop.

Gate: `just gate loop` and `just gate tum` both print numbers; Route A
closes at least one loop on `loop1` (RTAB-Map logs it) and beats raw
odometry on ATE. This is the bar Route B has to reach.

### P1 — Loop-closure *detection* while healthy — ✓ done 2026-08-18 (night)

**What happened.** `keypoint_detector` (rgbd mode) keeps a keyframe
graph: every stored keyframe gets a `pose_graph.py` node (its
`odom → base_link` at capture recorded beside it, plus the frame's
header stamp so `/world/trajectory` can be scored) and an odometry
edge to the previous one; the store gained a translation novelty axis
(`keyframe_novelty_m: 0.3` — a walked room needs it, a compass didn't),
`exclude` for `match()`, `force` for `maybe_add()`, and a persistent
`node_id` per keyframe. Every `loop_query_every` depth-paired frames the
live view is matched against keyframes older than `loop_min_age_s` and
not among the last `loop_exclude_recent`; the winner is verified by
**RANSAC PnP + LM** (`pnp_pose`: the keyframe's 3D landmarks against
the live pixels — one frame's monocular depth, not two) with a 3D-3D
rigid-fit cross-check, an implied-drift plausibility bound, and a
cooldown; a survivor is stored as a forced keyframe node with a *loop
edge* (`loop closure kf N -> kf M: I inliers, drift d m / a°`), drawn
magenta on `/world/keyframe_graph`. **The finding that mattered:**
building landmark geometry from the *latest* depth and *latest* TF —
the relocalization plan's accepted rule — misplaced keyframes by
several degrees at hand-pan speed (depth lands ~80 ms and the odom TF
~200 ms after the RGB frame), so closures disagreed with each other by
up to 0.5 m / 44° and had 20–55 inliers. rgbd geometry now runs on
exact triples — a frame's own ORB output, its own `/depth`, and TF *at
its stamp* (a depth queue drained by a 10 Hz timer once the TF has
landed, `sync_min_delay_s`) — after which the same bag's closures
agreed to ±0.4° with 105–360 inliers. Tests: PnP on a synthetic scene
with planted outliers, marker colouring, store exclude/force/novelty/
node_id round trip.

**Measured** on `bags/gate_loop` (odometry drift 17.6° that run): 11
closures against keyframes 0–3, drifts 16.6–17.4° — the same number
from every one; on fr1/desk 2 closures (109/122 inliers).

### P2 — The backend: pose-graph optimisation — ✓ done 2026-08-18 (night)

**What happened.** `piros2_world_mesh/pose_graph.py`: nodes SE(3),
edges (i, j, Z_ij, 6×6 information), residual `log(Z⁻¹ T_i⁻¹ T_j)`,
Gauss-Newton on the manifold with right perturbation and Levenberg
damping, first-order Jacobians (`J_r⁻¹ ≈ I + ½ad(e)`), Huber on loop
edges, dense normal equations (a room is ≤ a few hundred nodes; sparse
Cholesky is what g2o adds at city scale), and `.g2o` read/write.
`se3.py` grew `hat`/`so3_exp`/`so3_log`/`se3_exp`/`se3_log`/`adjoint`.
`test_pose_graph.py` (19 tests): Lie round trips and the adjoint
identity; a drifted 24-node circle that one exact closure pulls back
onto the truth (every node within 6 cm; the correction spread over the
loop, not dumped on the last node); a planted wrong closure ("node 12
is node 0") that folds the naive graph 1.9 m and moves the Huber one
< ¼ of that; and the **`g2o` oracle** — same graph through
`/opt/ros/jazzy/bin/g2o -solver lm_dense`, positions within 1 mm, and
under our cost our optimum ≤ g2o's (the two sit 0.07° apart in a flat
valley at identical χ² to six decimals; g2o's EDGE_SE3:QUAT residual is
[t, q_xyz] ≈ [t, φ/2], so its rotation information is ×4 in the file
to define the same cost). Wired into the detector: every new loop edge
runs `optimize_graph`, `map → odom` = optimised newest node ∘ its odom
pose⁻¹, broadcast on a 10 Hz timer when `publish_map_tf` (launch:
`slam:=own`), optimised poses on `/world/trajectory` (Path, RViz
display `Trajectory`) and edges on `/world/keyframe_graph`
(`KeyframeGraph`). RTAB-Map's `slam:=rtabmap` and ours never both own
the frame.

**Measured.** `just gate-loop own`: raw tail 6.1 cm / 1.90° →
**2.3 cm / 0.85°** — PASS (χ² 2–8 over 17 nodes / 11 loops: a coherent
graph; the earlier latest-TF geometry gave 5.2 → 3.5 cm / 7.45 → 1.18°
and 3.8 → 4.3 cm / 17.6 → 1.18° — a translation *regression* until the
PnP measurement replaced the two-depth rigid fit). `just gate-tum own`:
fr1/desk ATE 0.163 m → **0.089 m** — PASS (RTAB-Map's own run: 0.212 →
0.096 m; RTAB-Map's loop gate: 0.7–1.4 cm / 0.6–1.6°). Within the
stated factor of the yardstick; the residual translation is the
monocular depth's structured error, which a Sim(3) graph would not fix
either.

Turn the keyframe store from a crash-recovery memory into a place
recogniser that runs all the time.

- `keypoint_detector` keeps a **keyframe graph**: every stored keyframe
  gets a node id and its capture pose (already stored) plus an
  **odometry edge** to the previous keyframe (relative pose from the
  live odom TF at capture time). Publish `/world/trajectory`
  (`nav_msgs/Path`, keyframe poses) and grow `keyframe_marker` to draw
  edges (LINE_LIST).
- **Every N-th keyframe, query the store against all keyframes older
  than the last M** (the recency exclusion — matching your own
  previous frame is not a loop). Same margin/min-pair rules; then
  **geometric verification**: rigid fit on the 3D landmarks
  (`se3.rigid_transform_3d` exists) with a RANSAC inlier count, reject below a
  threshold. A survivor is a **loop edge**: relative pose + inlier
  count + a covariance proxy. Log it (`loop closure: kf 41 ↔ kf 3,
  inliers 87, Δ 0.31 m / 4.2°`) — the gate reads that line.
- Detection only in this phase: **nothing is corrected yet**. Store
  loop edges; publish them on the marker in a different colour.
- Also store per-keyframe **depth** (the P3 rebuild will need it):
  `/depth` downsampled to 320×180 uint16 mm + the JPEG bytes — ~150 kB
  per keyframe, 100 keyframes ≈ 15 MB, well inside RAM; on disk in
  `export_capture`'s TUM layout so `fuse_capture` can read it unchanged.

Gate: `just gate loop` (Route B mode) reports ≥ 1 verified loop edge on
`loop1` whose relative pose agrees with RTAB-Map's optimised poses for
the same pair within a threshold (say 0.2 m / 5°); zero false loops on
`bags/sweep3` if it doesn't loop, and zero on `gate_flick`'s A → B
segment. Unit tests: recency exclusion, RANSAC rejects a planted wrong
match, edge bookkeeping.

### P2 — The backend: pose-graph optimisation — *planned*

The phase that makes it SLAM.

- `piros2_world_mesh/pose_graph.py`, pure numpy, no ROS: nodes = SE(3)
  keyframe poses, edges = (i, j, relative T, information). Hand-rolled
  **Gauss-Newton on the manifold**: `se3.py` grows `log`/`exp`
  (twist ↔ 4×4), the adjoint, and the left-perturbation Jacobians;
  residual = `log(T_ij⁻¹ · T_i⁻¹ · T_j)`, Huber kernel on loop edges,
  first node fixed, sparse normal equations (`scipy.sparse` is fine at
  ≤ 100 nodes; dense would be too). This is the part worth writing by
  hand — it *is* the syllabus's "pose graph optimisation / Lie algebra
  / robust cost" line — and it is small: ~150 lines.
- **Check it against g2o.** `pose_graph.py` writes/reads the `.g2o`
  text format (`VERTEX_SE3:QUAT` / `EDGE_SE3:QUAT`); the same graph
  through `/opt/ros/jazzy/bin/g2o -o out.g2o in.g2o` must land within
  a millimetre/milliradian of ours. That is the unit test's oracle
  for real graphs; synthetic tests (a drifted circle that closes, a
  planted wrong loop the Huber kernel down-weights) cover the rest.
- Wire it in: on a new loop edge, optimise, then **publish `map →
  odom`** = optimised pose of the newest keyframe ∘ (its odom pose)⁻¹
  — REP-105 shape, `odom → base_link` continues untouched. When
  `slam:=rtabmap` is on, ours stays off (one owner per frame). The
  `Path` re-publishes with corrected poses.
- Scale: with `rgbd` odometry the graph is SE(3) and the mono scale
  error is a wobble the loop edges partly absorb; a Sim(3) graph is
  the named next lever if the residuals say scale is the dominant
  term. Say which it was.

Gate: `just gate loop` Route B: **ATE and start-end error after
optimisation < before, and within a stated factor of RTAB-Map's** on
`loop1`; `just gate tum` gives the absolute number on fr1/desk. Both
print before/after. Suite: graph tests + the g2o oracle test (skips
cleanly if the `g2o` binary is missing).

### P3 — Map correction: a TSDF that follows the graph — ◐ built 2026-08-18 (night), gate provisional

**What happened.** `tsdf_mesher` grew a frame memory (`rebuild: true`
under `slam:=own`; aligned depth as uint16 at `rebuild_downsample: 2`,
the JPEG bytes, the odom pose each frame was posed with and the
`map → odom` in force at the time — thinned evenly at `rebuild_keep`),
integrates live at `world_frame ← optical` (`map` whenever a backend
publishes it, so the surface is corrected from the start), and on every
new `/world/trajectory` (+ its odom twin `/world/trajectory_odom`, so
the per-node correction needs no TF history) computes each frame's
desired correction from its nearest node; when any frame's correction
moved beyond `rebuild_min_shift_*` it rebuilds the whole volume from
memory (`rebuilt TSDF #n: F frames at corrected poses in T ms`).
Measured: ~100 frames in 2.3–3.5 s on the GPU, four rebuilds over the
loop bag's return leg. Two things found on the way, both real: **the
mesher was starving itself** — at 1.5 cm voxels this scene meshes to
0.7–1.6 M triangles and complete + decimate took 12–21 s of a 15 s
period inline, so after the first refresh only ~50 frames of an 88 s
bag were ever integrated (in *both* runs); the refresh's heavy half now
runs on a worker thread (extract on the executor, decimate → complete →
publish off it, a refresh skipped while the previous one is busy) and
integration keeps its cadence — at the price of ~160 ms/frame under
contention instead of 33; and `~/save`'s Poisson-closed companion
takes minutes at that size, so the launch grew `mesh_watertight:=`
(default unchanged) and the gates turn it off. Also `mesh_save_frames:=`
dumps the frame memory beside the PLY (`live_<stamp>_frames.npz`).

**Gate.** `just gate-mesh` (one headless run) → `just mesh-views` and
two numbers: `tools/verify/mesh_planes.py` (RANSAC planes, thickness)
and `tools/verify/mesh_split.py` (the OUT and BACK halves of the
palindrome re-integrated into separate volumes at the odometry poses
and at the corrected poses; nearest-neighbour gap between the two
surfaces). Measured 2026-08-18: the plane metric does not discriminate
on this scene — `sweep3` is a close-range wall + object with no
dominant plane (4–5 % inliers, p95 thickness = the band) — and the
split metric **PASSes directionally (median gap 57 → 26 cm, p90 314 →
225 cm) but its absolute values say the two halves barely overlap**
(603k vs 140k surface points), so it is not yet evidence that walls
coincide. Open: make the halves comparable (equal frame counts per
half, a bounded region of interest, or score the same source frame's
two integrations directly) — until then P3 is *built*, not *proven*.

### P4 — Persist the graph, localise in it, and say the word — ◐ built 2026-08-18 (night), gate unrun

**What happened.** `~/save_map` (`just map-save`) now writes the pose
graph beside the keyframes in the same plain-array npz — optimised
poses, each node's odom pose at capture, stamps, every edge with
measurement / information / kind, and `map → odom` — and `map_path:=`
restores it (`load_map`): a session that relocalizes into a loaded room
extends the *same* graph (loaded nodes count as old for loop queries;
the next keyframe chains an odometry edge from the last stored node;
the stored `map → odom` is adopted). Unit-tested round trip. `just
gate-map` is written: session one saves; session two loads it, must
relocalize cold, close loops against loaded keyframes, and pass
`traj_check loop` on its own trajectory — **not yet run**.

**Docs.** Nothing has flipped: README / project-overview still state
the pre-SLAM scope with a pointer to this plan; the `SLAM` GitHub topic
is not set (an outward change for the owner to make once P4 passes);
`docs/to_learn/emescent.md`'s "NOT SLAM" line stands until then.

Without this the surface still lies after every closure.

- On an optimisation that moved any keyframe more than a threshold
  (say 2 cm / 1°), `tsdf_mesher` **rebuilds**: new `VoxelBlockGrid`,
  re-integrate every stored keyframe (P1's depth) at its optimised
  pose. Measured cost is the argument this is viable: the fusion plan
  clocked 10 ms/frame on the GPU, so 100 keyframes ≈ 1 s. Live frames
  between keyframes keep integrating at the corrected TF as now.
- Trigger: `keypoint_detector` publishes the optimised keyframe
  poses (`/world/keyframe_poses`, PoseArray with ids) and a graph
  version; the mesher rebuilds when the version changes and poses
  moved. Rebuild in the mesher's thread; the Marker republishes on the
  usual timer.
- Submaps (a grid per few keyframes, re-posed instead of rebuilt) are
  the scalable version — out of scope for one room; note it as the
  step after.

Gate: `just run-bag bags/loop1` → `mesh-save` → `mesh-views` before/after
the closure; plus the todo's **wall-flatness number** (RANSAC the saved
PLY's dominant planes — `tools/recon/room_layer.py` has the pieces —
report inlier fraction and thickness): the same wall's thickness after
correction < before. `fuse_capture --trajectory` on the exported
keyframes with the optimised `groundtruth.txt` is the offline twin of
the same check.

(P4 as planned: `~/save_map` writes graph nodes + edges beside the
descriptors; `map_path:=` loads it and keeps extending the same graph;
docs and claims flip together — README/project-overview scope line,
the diagrams page, the `SLAM` GitHub topic, `emescent.md`'s "NOT SLAM"
line — and only then. The per-keyframe depth store moved to the
mesher's own frame memory in P3.)

### P5 — Stretch, out of this plan's scope

- Sliding-window **bundle adjustment** on the ORB landmarks (the
  front-end's own accuracy, not just the graph's).
- **C++**: the pose-graph optimiser is the natural first thing to
  port (todo.md's "rewrite in C/C++"; a small, self-contained numeric
  kernel with a Python oracle and a g2o oracle already written).
- Sim(3) graph for monocular scale drift; an IMU if hardware ever
  arrives (none is attached — nothing here assumes one).

## What still needs a person

Nothing so far needed one: the palindrome bag stood in for the loop
recording (P0) and TUM fr1/desk supplied real ground truth. A recorded
loop with real translation (`just record 60 loop1`, ending on the start
view) would still be worth having — the palindrome's drift is a hand
pan's few centimetres, and a walked loop tests the translation axis the
monocular depth is weakest on. What still needs *Claude* (next
session): run `just gate-map`; make P3's split metric credible; then
flip the claims (P4).

## Session log

- 2026-08-18 evening/night: P0 (loop bag, `slam:=rtabmap` yardstick,
  `traj_record`/`traj_check`/`tum_player`, `gate-loop`, `gate-tum`);
  P1 (keyframe graph, always-on loop detection, exact-sync geometry,
  PnP verification); P2 (`pose_graph.py`, `se3` Lie helpers, g2o
  oracle, `map → odom`, `/world/trajectory`); P3 (frame memory,
  rebuild, threaded refresh, `gate-mesh`, `mesh_planes.py`,
  `mesh_split.py`); P4 (graph persistence, `gate-map` recipe). Fork
  suite 122 tests. Not run: `gate-map`. Not credible yet: `mesh_split`'s
  absolute numbers.

## Traps to carry forward (already known, still apply)

- Never gate on `header.stamp` — the camera's 0.73 s fault; associate
  by receipt where a live clock is involved, by stamp only between
  the estimator's twin topics (identical by construction).
- One owner per TF frame: `map → odom` is ours *or* RTAB-Map's, never
  both; `odom → base_link` stays with rgbd/kp.
- Every session node added here goes into the recipe's EXIT trap in
  the same change (`rtabmap` is already in the pattern list).
- The venv rule: anything importing open3d/onnxruntime runs as
  `python -m` under `~/.venvs/piros2-perception`; `pose_graph.py`
  should stay numpy/scipy so it lives outside it and unit-tests plain.
