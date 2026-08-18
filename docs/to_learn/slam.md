# SLAM — the study file for section 2 of [emescent.md](emescent.md)

Written 2026-08-18. One section per checklist item: what it is, the sentence an interviewer
is fishing for, and an honest **`piros2` line** — what this repo actually touches, so the
claim in the room matches the code. Per the honest-claim rule: `piros2` is rotation-only
orientation + TSDF meshing + a keyframe relocaliser. **It is not SLAM**, and none of the
sections below change that; they mark which *pieces* of SLAM it has exercised.

The Wildcat facts come from the paper itself (Ramezani et al., *"Wildcat: Online
Continuous-Time 3D Lidar-Inertial SLAM"*, [arXiv 2205.12595](https://arxiv.org/abs/2205.12595),
T-RO 2022 submission), read 2026-08-18 — see §13, the section that matters most for Emesent.

## Mental model to carry through the whole file

```
sensors ─► FRONTEND (odometry: "where did I move since the last frame?")
               │  relative poses + local map, drifts without bound
               ▼
           BACKEND  (optimisation: "make all the poses agree with all the constraints")
               │  pose graph / factor graph; loop closures are the constraints that kill drift
               ▼
           MAP + TRAJECTORY  (globally consistent, or as consistent as the constraints allow)
```

Every SLAM system in section 2 is a choice at four points: **what to match** (raw points,
features, surfels, pixels), **how to match it** (ICP variants, descriptors, photometric),
**how to fuse the IMU** (loosely, tightly, filter, smoother), and **how the trajectory is
represented** (discrete keyframes vs continuous time). Wildcat's answers: surfels ·
point-to-plane between surfels · tightly, in a sliding-window optimisation · cubic B-spline.

## 1. Definition: simultaneous localisation and mapping, and why the two are coupled

- **Localisation** = estimate the sensor pose given a map. **Mapping** = build a map given
  poses. SLAM does both at once from scratch, in an unknown environment, with no external
  position source (no GNSS, no mocap, no beacons).
- **The coupling** is a chicken-and-egg: every landmark's position is only known relative to
  the pose that observed it, and every pose is only known relative to the landmarks it saw.
  Errors are therefore *correlated* — a pose error shifts every landmark seen from it, and
  matching later scans against those shifted landmarks feeds the error forward. That
  correlation is why you can't solve the two problems independently and why the state
  vector (in EKF-SLAM) or the graph (in modern SLAM) contains *both* poses and map.
- The mathematical statement: maximise the posterior of poses **and** map given the
  measurements, `argmax_{x, m} p(x, m | z, u)`. Everything since ~2005 has been
  smoothing-and-mapping (all poses jointly, sparse) rather than filtering (marginalise old
  poses, dense covariance).
- **Interviewer's target sentence:** "SLAM is estimating the trajectory and the map jointly
  because their errors are correlated; the frontend gives you locally consistent motion,
  the backend gives you global consistency by optimising over the constraints, and loop
  closures are the constraints that bound drift."
- **`piros2` line:** it estimates orientation from features (a localisation problem
  against the *previous frame*), fuses depth into a TSDF *given* that pose (a mapping
  problem), and never lets the map correct the pose except through the keyframe
  relocaliser — so the two halves exist but the coupling isn't closed. That gap is the
  honest reason it's "not SLAM".

## 2. Frontend vs backend

- **Frontend:** the sensor-facing half. Deskews the scan (§4 sensors), extracts what to
  match (features/surfels/points), does data association (which thing in this frame is
  which thing in the map), and produces relative motion estimates — *odometry* — plus
  candidate loop closures. Fast, incremental, local, and it lies (drifts) without bound.
- **Backend:** the estimation-theoretic half. Takes the frontend's constraints (odometry
  edges, loop-closure edges, IMU factors, GNSS/prior factors) and solves the sparse
  nonlinear least-squares problem for the globally consistent set of poses (and optionally
  landmarks). Slower, batch or incremental (iSAM2), and it only knows what the frontend told
  it — a wrong loop closure is a wrong constraint, so backends carry robust kernels (§13's
  Cauchy) or switchable constraints.
- The split is architectural, not just conceptual: they run at different rates (Wildcat:
  odometry ~15 Hz on a laptop; PGO on submaps every few seconds), in different threads or
  processes, and the frontend must not block on the backend.
- **`piros2` line:** `keypoint_detector` is a frontend (ORB, Hamming matching with
  cross-check, Kabsch on bearing rays); there is no backend. `mapping.launch.py` runs
  RTAB-Map, which *has* one — that's the config-and-plumb experience, not the build.

## 3. Odometry vs SLAM vs localisation in a known map

| | Input | Output | Drift | Example |
| --- | --- | --- | --- | --- |
| **Odometry** | consecutive sensor frames | relative motion, integrated to a pose | unbounded, grows with distance/time | wheel encoders, VO, LiDAR odometry (frame-to-frame or frame-to-local-map), `rgbd_odometry` |
| **SLAM** | sensor frames, no map | pose *and* map, globally consistent via loop closures | bounded where loops are closed; still drifts on open paths | Wildcat, LIO-SAM, ORB-SLAM3, RTAB-Map |
| **Localisation in a known map** | sensor frames + a prior map | pose in that map's frame | none in the long run (anchored to the map), but can be *lost* | AMCL, ORB-SLAM3 in localisation mode, Hovermap "relocalise to a saved map" |

- The distinctions matter operationally: odometry is enough for a drone's *inner* control
  loop (Hovermap's position hold), SLAM is needed for a *deliverable* map, and localisation
  in a known map is what lets a robot repeat a mission (Spot Autowalk) or resume after a
  reset. Frame-to-map odometry (LOAM's mapping thread, FAST-LIO2's ikd-tree) blurs the
  first two: it drifts less than frame-to-frame but has no loop closure.
- **`piros2` line:** rotation-only *odometry* (frame-to-frame Kabsch, composed) with a
  keyframe-store *relocaliser* — recovery against a self-built map of landmarks via
  Kabsch/Umeyama, including cold-start relocalisation into a saved `maps/room_*.npz`. That
  is the third column in miniature (orientation only), and the reset/clear services are
  the drift strategy in place of the second column's loop closure.

## 4. Pose graph optimisation

- Nodes = poses (robot poses at keyframes, or submap origins). Edges = relative-pose
  measurements with a covariance: odometry edges between consecutive nodes, loop-closure
  edges between non-consecutive ones. Solve for the node poses minimising the sum of
  squared, covariance-weighted residuals `Σ ‖log(T_ij⁻¹ · T_i⁻¹ T_j)‖²_Σij` — a nonlinear
  least squares on a manifold (SE(3)), Gauss–Newton or Levenberg–Marquardt, with the
  sparsity of the graph making it tractable (Cholesky on a sparse Hessian; the ordering
  is what makes it fast).
- Landmarks are marginalised out — that's the point of a *pose* graph vs a full factor
  graph: cheaper, and the frontend's local matching already digested the landmarks into
  relative-pose edges.
- Robustness: a single wrong loop closure drags the whole graph. Fixes: robust kernels
  (Huber, Cauchy, Geman-McClure), switchable constraints / dynamic covariance scaling,
  max-mixtures, or Mahalanobis gating *before* adding the edge (Wildcat does the gating and
  uses Cauchy IRLS).
- Wildcat's twist: nodes are **6-second submaps**, redundant nodes are *merged* when their
  surfel maps overlap and their Mahalanobis distance is small, so graph size grows with the
  size of the *environment*, not mission duration; and it adds a gravity-direction ("up")
  term from the accelerometer to the standard PGO cost.
- **`piros2` line:** never built one. Adjacent experience: the world-fusion plan's offline
  pipeline consumed RTAB-Map's optimised poses (`rtabmap-report --poses_raw`) as the
  trajectory for TSDF fusion, and saw what unoptimised rotation-only poses do to a mesh
  (radial smear) vs what RTAB-Map's did (shingling from depth-scale wobble). That is a
  user's understanding of a backend, not a builder's.

## 5. Loop closure detection and correction

- **Detection** = recognising a place you've been. Two families: *geometric* (search
  nodes within a Mahalanobis radius of the current pose estimate — cheap, but only works
  when drift is small relative to the search radius) and *appearance/place recognition*
  (bag-of-words on visual descriptors — DBoW2 in ORB-SLAM/RTAB-Map; LiDAR descriptors —
  Scan Context, and learned ones). Wildcat supports both: radius search or "existing place
  recognition methods such as Scan Context".
- **Verification** = the candidate must survive a geometric check: register the two
  frames/submaps (ICP for LiDAR, PnP + RANSAC for vision), demand enough inliers, then a
  Mahalanobis gate on the resulting relative pose vs the graph's current belief. False
  positives are worse than misses — a missed loop leaves drift; a false loop *corrupts*.
- **Correction** = add the edge, re-optimise the graph (§4). The whole trajectory between
  the two ends of the loop gets redistributed. In map-centric systems the map is
  re-rendered from the corrected poses (submaps re-placed; TSDF systems have to re-fuse or
  deform, which is why many live-mesh systems keep submaps rather than one global volume).
- **`piros2` line:** the relocaliser is the nearest thing — descriptor matching against a
  keyframe store, geometric verification by rigid fit, then a *snap* of the current pose
  (kp mode: replace the orientation; rgbd mode: `/reset_odom_to_pose`). That's
  place-recognition + verification + a one-shot correction *without* a graph: only the
  current pose moves, history isn't redistributed. `just gate flick` / `occlude` measure it
  (65.3° correction, 0.48° tail; 18.4° snap, 0.95°/3 cm tail).

## 6. Drift, revisit, global consistency

- **Drift** is the integral of small per-step errors. Rule of thumb figures to have in your
  head: good LiDAR-inertial odometry drifts ~0.1–0.5 % of distance travelled; Wildcat's SubT
  figure "≪0.05 %" in the Tunnel Circuit; Emesent's spec sheet says ±0.03 %; Hovermap's
  onboard-only processing drifted >90 m over a 22 km drive (~0.4 %) — that's what "no
  global optimisation" costs. Rotation error is the killer: a heading error θ produces
  position error `d·θ` after distance d, so it grows *linearly in distance* on top of the
  translational random walk.
- **Revisit** is what makes drift correctable: without returning to a known place there is
  no constraint that pins the far end. Survey practice mirrors this — Emesent's KB tells
  operators to close loops and to place ground-control targets, and Aura's "non-rigid drift
  correction" is exactly redistributing the error along the trajectory once a constraint
  exists.
- **Global consistency** = every part of the map agrees with every other part — a wall seen
  from both ends of a loop is one wall, not two. *Local* consistency (the frontend's job) is
  enough to fly a corridor; global consistency is what makes a stope volume or a
  convergence heat-map trustworthy.
- **`piros2` line:** the ±4 % per-frame depth-scale wobble and the `ScaleAligner`
  high-pass — "correct only the deviation from a rolling median, so drift is impossible by
  construction" — is a real, measured story about *why* you don't let a map feed back into
  its own pose estimate without a bound (conform-to-map was unstable: the ray-cast reads
  ~1.25 voxels far and the loop walked walls away). That is a drift-feedback lesson, told
  honestly.

## 7. Scan matching: ICP (point-to-point, point-to-plane), NDT, GICP

- **ICP** (Besl & McKay 1992): alternate (a) find nearest-neighbour correspondences between
  source and target, (b) solve for the rigid transform minimising the residual, until it
  converges. Needs a good initial guess (basin of convergence is small — that's what the
  IMU/motion model is for), and correspondences are found with a KD-tree.
  - **Point-to-point** residual `‖R p_i + t − q_i‖²`: closed form per iteration (Kabsch/
    Horn/Umeyama), slow convergence on flat regions because points slide.
  - **Point-to-plane** residual `((R p_i + t − q_i) · n_i)²`: lets points slide along the
    surface, converges far faster, needs normals on the target; linearised with small-angle
    approximation into a 6×6 system. This is what LOAM's planar features, Wildcat's
    surfel-to-surfel cost and most LiDAR pipelines use.
- **GICP** (Segal 2009): plane-to-plane — both clouds get a per-point covariance (locally
  planar, disc-shaped), and the residual is weighted by the combined covariance
  `(d_iᵀ (C_q + R C_p Rᵀ)⁻¹ d_i)`. Generalises point-to-point (C = I) and point-to-plane
  (C degenerate). Wildcat's surfel weight `1/(σ² + λ_s + λ_s')` — the smallest eigenvalues
  of the two surfels — is the same idea: weight by the thickness of the matched pair.
- **NDT** (Biber & Straßer 2003): voxelise the target, fit a Gaussian per voxel, maximise
  the likelihood of the source points under those Gaussians. No explicit correspondences,
  smooth cost, larger convergence basin; the standard in Autoware-style localisation.
- Practical knobs interviewers probe: max correspondence distance, downsampling, outlier
  rejection (trimmed ICP, robust kernels), termination, and the difference between
  scan-to-scan and scan-to-map (map = local submap or KD-tree of recent points; less drift,
  more compute).
- **`piros2` line:** Kabsch is used twice — closed-form rotation from matched *bearing
  rays* (rotation-only, no translation, essential matrix degenerate under pure rotation)
  and rigid Umeyama for relocalisation against 3D landmarks. That is the inner solve of one
  ICP iteration with correspondences given by descriptors, not by nearest neighbour; the
  iterate-with-NN part has not been built.

## 8. Feature-based vs direct methods

- **Feature-based / indirect:** detect keypoints (ORB, SIFT, corners; LOAM's edge and
  planar points by local curvature), describe them, match by descriptor, then minimise
  *geometric* error (reprojection error in vision, point-to-plane in LiDAR). Robust to
  lighting/exposure, works with wide baselines, sparse maps.
- **Direct:** skip features, minimise *photometric* error over pixel intensities (LSD-SLAM,
  DSO, DVO for RGB-D) or, for LiDAR, use all points / dense surfels rather than curated
  features (FAST-LIO2 registers raw points direct to the map; Wildcat's surfels sit between
  — dense-ish, but structured). Direct vision needs photometric calibration, small
  baselines, is sensitive to exposure and rolling shutter, and gives semi-dense maps.
- Middle ground: semi-direct (SVO), or feature detection with direct alignment.
- **Interviewer's target sentence:** "Features buy robustness to appearance change and wide
  baseline; direct buys accuracy and use of all the image; in LiDAR the same axis runs
  from LOAM's curated edge/plane features to FAST-LIO2's raw points, and the trade is
  compute vs robustness to degeneracy."
- **`piros2` line:** squarely feature-based (ORB, Hamming, cross-check, the chessboard
  lookalike-corner test), and it learned the classic feature failure — a covered lens
  yields no descriptors and `could_estimate` silently stayed False until the
  `was_tracking` fix.

## 9. Visual SLAM: ORB-SLAM, VINS

- **ORB-SLAM (1/2/3, Mur-Artal, Campos et al.):** the reference feature-based system.
  Three threads: tracking (ORB features, PnP against local map), local mapping (local BA,
  keyframe culling), loop closing (DBoW2 place recognition, Sim(3)/SE(3) correction, then
  a global BA). ORB-SLAM3 (2021) adds visual-inertial (IMU preintegration, MAP
  initialisation), multi-map "Atlas" (relocalise into an old map after tracking loss),
  fisheye. Monocular = scale ambiguous; stereo/RGB-D/IMU fix scale.
- **VINS-Mono / VINS-Fusion (Qin, Shen, HKUST):** tightly-coupled visual-inertial
  odometry — sliding-window nonlinear optimisation with IMU preintegration factors and
  feature reprojection factors, marginalisation to keep the window bounded, online
  extrinsic/temporal calibration, loop closure via DBoW2 and a 4-DoF pose graph (yaw +
  position; roll/pitch observable from gravity). The canonical "how to do the IMU properly"
  reference; Wildcat's sliding-window structure is the LiDAR analogue.
- Why interviewers ask: to hear *keyframes, local BA, sliding window, marginalisation,
  preintegration, 4-DoF vs 6-DoF loop correction*, and the monocular scale problem.
- **`piros2` line:** ORB features and a keyframe store are the shared vocabulary; no BA, no
  IMU, monocular depth from a network instead of triangulation — the depth-scale wobble is
  the monocular scale problem showing up through a different door.

## 10. LiDAR SLAM: LOAM, LeGO-LOAM, FAST-LIO, KISS-ICP

- **LOAM (Zhang & Singh, RSS 2014):** the ancestor. Extract edge (high curvature) and
  planar (low curvature) points per scan line; a fast odometry thread (~10 Hz,
  scan-to-scan, linear motion interpolation for deskew) and a slow mapping thread (~1 Hz,
  scan-to-map); loosely coupled IMU as a motion prior. No loop closure in the original.
- **LeGO-LOAM (Shan & Englot, IROS 2018):** lightweight, ground-optimised for ground robots:
  segment the ground plane first, cluster the rest, then a two-step optimisation
  (ground planes → z/roll/pitch; edges → x/y/yaw); adds a pose graph with loop closure. Runs
  on embedded compute.
- **LIO-SAM (Shan et al., IROS 2020):** tightly-coupled — IMU preintegration + LiDAR
  odometry + optional GPS in a factor graph (GTSAM, iSAM2), keyframe-based, loop closure by
  radius search + ICP. Wildcat's paper compares against it (and it failed in QCAT's tunnel).
- **FAST-LIO / FAST-LIO2 (Xu, Zhang, HKU 2021/2022):** tightly-coupled iterated
  error-state Kalman filter (not a graph), IMU forward propagation + backward propagation
  for deskew, raw points registered directly to an incremental KD-tree map (**ikd-tree**),
  no feature extraction; very fast, very accurate in structured scenes; no loop closure in
  the core. The other system Wildcat compares against (also failed in QCAT's tunnel).
- **KISS-ICP (Vizzo et al., RA-L 2023):** "keep it simple" — point-to-point ICP,
  constant-velocity deskew, adaptive threshold, a voxel-hash local map, no IMU, no
  features; one parameter set works across many datasets. The counter-argument to
  complexity: a well-engineered baseline is hard to beat for pure odometry.
- The axis to draw in an answer: *features → raw points* (LOAM → FAST-LIO2/KISS), *loosely →
  tightly coupled* (LOAM → LIO-SAM/FAST-LIO), *filter → smoother* (FAST-LIO → LIO-SAM/
  Wildcat), *discrete → continuous time* (all of the above → Wildcat).
- **`piros2` line:** none of these run here. RTAB-Map's `rgbd_odometry` (visual, RGB-D) is
  the only odometry that has been run and tuned (30-deep exact-sync queues; the
  102-vs-0-updates finding).

## 11. LiDAR-inertial odometry, tightly vs loosely coupled

- **Why the IMU at all:** it gives a high-rate (100–400 Hz) motion prior for deskewing the
  scan and initialising the match, it makes roll/pitch observable via gravity, it bridges
  the ~0.1 s between scans, and it carries the pose through the seconds when the LiDAR is
  *degenerate* (§16) — the last is the safety-critical one for a drone.
- **Loosely coupled:** the IMU is integrated separately (an INS/EKF) and its output is used
  as the initial guess or fused with the LiDAR pose as two independent estimates (LOAM).
  Simple, modular, but the LiDAR match doesn't get to *correct IMU biases*, and the two
  estimators can disagree.
- **Tightly coupled:** raw IMU measurements and LiDAR residuals go into *one* estimator
  — a filter (FAST-LIO's iEKF) or a sliding-window/keyframe optimisation (LIO-SAM's
  preintegration factors, Wildcat's `f_imu + f_match`). Biases, gravity direction and
  extrinsics become part of the state and are estimated. More accurate and robust in
  degeneracy; more coupled to good calibration and time sync.
- **IMU preintegration** (Forster et al. 2015/2017): integrate the IMU between two
  keyframes *once*, in the body frame, into a relative-motion factor whose Jacobians w.r.t.
  bias let the optimiser correct bias without re-integrating — the trick that made
  tightly-coupled smoothing tractable.
- Wildcat's specifics: IMU at 100 Hz (`Δt_imu = 0.01 s`, 3DM-CV5 in the SubT pack), poses
  first initialised at IMU timestamps, then the surfel-matching and IMU cost functions
  optimised together over the window; the accelerometer also supplies the gravity "up"
  term in the pose graph.
- **`piros2` line:** no IMU anywhere. The nearest thing is the *sync* discipline —
  message_filters exact-time pairing of `/depth` and RGB, the stamp-twin `/depth/rgb`
  republish so exact sync always pairs, and the 0.73 s camera timestamp fault — i.e. the
  time-alignment prerequisites of any coupling, learned the hard way.

## 12. Continuous-time SLAM and trajectory representation (the CSIRO Wildcat lineage)

- **The problem it solves:** a spinning LiDAR does not take a snapshot — a "scan" is
  10⁵–10⁶ points each with its own timestamp, taken while the sensor moves (motion
  distortion / skew), and the IMU ticks asynchronously at another rate. Discrete-time
  systems pick one pose per scan and *deskew* the points to it using a motion model
  (constant velocity, or IMU propagation) — an approximation that gets worse as motion
  gets faster or the platform vibrates (drones, spinning heads).
- **Continuous-time answer:** represent the trajectory as a *function* `T(t)` you can
  evaluate at any timestamp — then every LiDAR point and every IMU sample gets its exact
  pose, and multi-rate, asynchronous sensors fuse naturally. Representations: **B-splines**
  (cubic, on SE(3) or split SO(3)×R³ — Wildcat, Spline Fusion, "Elastic LiDAR Fusion"),
  **Gaussian-process** regression over the trajectory (Barfoot's STEAM, "Elastic" GP
  variants), or piecewise-linear interpolation between sample poses (LOAM's cheap version).
- **Cost:** the optimisation variables are control points / sample poses, not one pose per
  scan; Jacobians through the spline are more involved; you need to choose knot spacing
  (too coarse and you can't represent the motion, too fine and it's under-constrained).
- **CSIRO lineage:** Bosse & Zlot's *Zebedee* (2009–2012, the hand-held spring-mounted
  LiDAR — the origin of the "wobbly stick" and of GeoSLAM's ZEB line), continuous-time
  trajectory estimation with surfels; → *Elastic LiDAR Fusion* (Park, Moghadam et al.
  2018, sliding-window continuous-time with surfels); → **Wildcat** (2022, the SubT
  system) → Hovermap's SLAM. Emesent's marketing "Wildcat SLAM" is this line.
- **`piros2` line:** discrete keyframes and TF lookups; the closest brush is *latest-TF
  lookups per the stamp fault* in the mapper/mesher and the odom-frame clouds — a
  deliberate choice to *not* interpolate poses to point timestamps because the camera's
  header stamps are wrong. That is a decision continuous-time SLAM would make the other
  way, and it's worth saying so.

## 13. Wildcat, precisely (from the paper — the section to know cold)

Two modules, Fig. 2 of the paper:

1. **Wildcat odometry** — sliding-window LiDAR-inertial odometry with a local map.
   - Time window `W_k`: fixed length, slid forward each step; new poses initialised at IMU
     timestamps by IMU integration from the last window.
   - **Surfel generation:** voxelise the new points, cluster within each voxel, fit an
     ellipsoid (mean + covariance); keep only sufficiently *planar* surfels (spectrum of
     the covariance), normal = eigenvector of the smallest eigenvalue; multi-resolution
     (repeated at several voxel sizes).
   - **Correspondence:** k-NN between surfels created in the current window and the map,
     reciprocal matches only, whose timestamps are far enough apart.
   - **Optimisation:** alternate (i) update correspondences, (ii) solve for a set of
     *sample poses* (correction poses at `n` equidistant timestamps in the window)
     minimising `f_imu + f_match` — IMU residuals (accel, gyro, bias random walks, gravity
     in the world frame) plus a **point-to-plane surfel-to-surfel** cost weighted by
     `1/(σ² + λ_s + λ_s')` — with a **Cauchy M-estimator** for outliers, (iii) fit a
     **cubic B-spline** through the corrected samples to get `T̂(t)`, re-place every surfel
     at its own timestamp, repeat. Poses at IMU rate (100 Hz) fall out of the spline.
   - Cost: ~63 ms per main optimisation loop (~15 Hz) on an Intel Xeon W-10885M laptop;
     the CSIRO perception pack ran it on an **NVIDIA Jetson AGX Xavier**.
2. **Pose-graph optimisation** — nodes are **6-second submaps** (a bundle of odometry
   estimates + the accumulated local surfel map in the submap's frame, ~100–170 KB each in
   SubT — small enough to share over a mesh radio); odometry edges between consecutive
   submaps; **loop-closure edges** from aligning overlapping submaps by point-to-plane ICP
   (a global method for the initial guess when the prior is poor); candidates from a
   Mahalanobis radius search *or* a place-recognition plug-in (Scan Context); a
   Mahalanobis gate before an edge is accepted; redundant nodes **merged** so the graph
   scales with environment size, not mission length; the cost is standard PGO + a
   gravity-direction term, minimised by IRLS with Cauchy. Runs periodically (~every 5 s).
3. **Multi-agent:** agents exchange *submaps* peer-to-peer and each optimises the collective
   graph independently — decentralised, no server. This is the "only distributed
   multi-agent SLAM" claim at SubT; DARPA scored the four-robot Final map at "0% deviation,
   91% coverage".
4. **Results to quote:** vs FAST-LIO2 and LIO-SAM on MulRan DCC03 (urban, Ouster OS1-64)
   and the in-house QCAT dataset (2 h hand-held, 63 surveyed targets, a tunnel): Wildcat
   mapped all 63 targets with mean absolute error 0.42 m (FlatPack) / 0.34 m (SpinningPack)
   vs LIO-SAM 0.92/1.69 m and FAST-LIO2 1.09/0.43 m on the *subset* of targets they reached
   before **both slipped in the tunnel** (Fig. 14). Multi-agent SubT map: mean 3 cm,
   σ 5 cm from DARPA's surveyed cloud, after alignment. Sensors: VLP-16 at 20 Hz on a
   0.5 Hz spinning servo (120° vertical FoV) or flat; 100 Hz 9-DoF IMU.
5. **What it is *not*:** not visual (cameras only for colour), no learned components, no
   published degeneracy detector in this paper — the robustness in the tunnel comes from
   the tightly-coupled IMU term and the surfel weighting.

**Questions worth asking them:** knot spacing / window length in production; how Cortex
detects and reports degeneracy and "SLAM slip"; whether onboard processing (0.1 m voxels,
subset of rings, no global optimisation, 80 s per 15-min mission) is Wildcat odometry with
PGO off; how the Aura re-process differs (full PGO + non-rigid correction); ROS 1 → 2.

## 14. Submaps and map merging

- **Why submaps:** a single global map can't be corrected after loop closure without
  re-rendering everything; a submap is a rigid block built over a short window whose
  *internal* consistency is trusted (Wildcat: "error accumulated within a submap is
  negligible because each submap's internal structure is already optimised"), so the
  backend moves whole submaps as rigid bodies. Cartographer's submaps, Wildcat's 6 s
  bundles, ORB-SLAM3's Atlas maps, TSDF systems' volumes-per-keyframe all serve this.
- **Merging:** align two submaps (ICP with a good prior, or a global registration —
  FPFH + RANSAC / TEASER++ / Scan Context — when there's no prior), verify overlap and
  uncertainty, then either add an edge (keep both) or fuse (Wildcat merges *nodes* whose
  surfel maps overlap significantly and are within a Mahalanobis threshold).
  Multi-session and multi-agent merging is the same operation with a bigger unknown
  initial transform.
- **`piros2` line:** `cloud_mapper`'s voxel map and the TSDF `VoxelBlockGrid` are single
  global volumes with no submaps — the "walls walk away when the map feeds back" finding
  is partly a consequence (nothing to re-place rigidly). The saved-map relocaliser is a
  minimal two-session merge: align the current view to a saved landmark set and snap.

## 15. Factor graphs: GTSAM, Ceres, g2o

- **Factor graph:** bipartite graph of *variables* (poses, landmarks, biases, extrinsics)
  and *factors* (measurements connecting them: odometry, IMU preintegration, projection,
  prior, GNSS). MAP inference = sparse nonlinear least squares; the graph *is* the sparsity
  pattern of the Jacobian. A pose graph is the special case with only pose variables and
  relative-pose factors.
- **GTSAM** (Georgia Tech, Dellaert): the factor-graph library — iSAM2 incremental solver
  (Bayes tree, relinearise only what changed), IMU preintegration factors built in, used by
  LIO-SAM, Kimera, and Georgia Tech's SubT work alongside CSIRO. **Ceres** (Google): a
  general nonlinear least-squares solver — you write residual functors, autodiff, choose a
  robust loss and a manifold; VINS, Cartographer, many custom pipelines. **g2o** (Kümmerle
  et al.): the classic graph-optimisation library — ORB-SLAM, older RTAB-Map;
  vertices/edges API, Levenberg–Marquardt or Gauss–Newton with sparse Cholesky (CHOLMOD/
  CSparse). Wildcat's paper describes the math but not the library.
- The concepts interviewers actually test: robust loss, manifold/retraction for SO(3)/SE(3),
  marginalisation vs sliding window, incremental vs batch, and why sparsity makes
  10⁴-node graphs solvable in milliseconds.
- **`piros2` line:** none used directly. `se3.py` (make_transform / invert /
  transform_points, the four-branch quaternion conversion) is the on-manifold algebra a
  factor's error function is written in — SE(3) as a group, not as six floats.

## 16. Bundle adjustment

- Jointly refine camera poses **and** 3D point positions to minimise total *reprojection
  error* — the photogrammetry problem (Triggs et al. 1999). Sparse structure: each point is
  seen by a few cameras, so the Hessian has a big block-diagonal point part that the
  **Schur complement** eliminates cheaply, leaving a small camera system. Local BA
  (ORB-SLAM's local mapping: recent keyframes + their points, others fixed) runs
  continuously; global BA runs after loop closure.
- LiDAR analogue: "bundle adjustment for LiDAR" (BALM, Liu & Zhang) — jointly optimise
  poses to minimise plane/edge feature residuals, eliminating the feature parameters —
  and Wildcat's within-window optimisation is BA-shaped over surfels.
- **`piros2` line:** no BA. Approximate spec-derived intrinsics (`fx = fy ≈ 907 px`) and the
  unfinished checkerboard calibration are the reprojection-error concept in its simplest
  form; the "tape-measure scale 2.69" is a one-parameter global adjustment by hand.

## 17. Degeneracy: why long featureless tunnels break scan matching

- **What happens:** in a straight tunnel with uniform walls, every point-to-plane residual
  constrains motion *perpendicular* to the walls but says nothing about motion *along* the
  tunnel axis (or about rotation about it, in a circular tunnel): the walls look identical
  one metre further on. The 6×6 Hessian of the registration problem has a (near-)zero
  eigenvalue along that direction; the solver either drifts freely, sticks (estimates zero
  motion — the classic "the robot stops moving on the map while it drives"), or snaps to
  noise. Same story for a corridor with no doors, an open field for LiDAR, a blank wall for
  a camera. FAST-LIO2 and LIO-SAM both "slipped" in the QCAT tunnel; Emesent's manual
  names "SLAM slip" and cancels position hold and RTH when it happens.
- **Detecting it:** Zhang, Kaess & Singh 2016 — eigen-decompose the Hessian (or `JᵀJ`) of
  the registration; eigenvalues below a threshold mark degenerate directions; solve only in
  the well-conditioned subspace and let the prior (IMU / other sensor) fill the rest.
  Alternatives: condition number, the localisability measures in X-ICP, monitoring the
  match's information gain, or the *observability* of the whole state in an EKF.
- **Mitigating it:** tightly-coupled IMU (carries the unconstrained axis for seconds, not
  minutes — bias random walk); wheel odometry / a second modality (camera sees texture on
  a wall LiDAR can't); LiDAR *intensity* as a feature; a wider-FoV or spinning sensor (a
  cross-section behind you constrains yaw); mining-specific — the vehicles, ventilation
  ducting, mesh, cabling and rock bolts are actually features, and operators are told to
  scan slowly and to close loops; ground-control targets to bound the error where it
  matters (Automated Ground Control); refusing to fly autonomously when the match is
  degenerate (fail loud > drift silent).
- **The interview line:** "Long straight tunnels are the classic degenerate case — the cost
  function is flat along the axis, so scan matching fails *silently*; you detect it from the
  Hessian's spectrum and you survive it by letting the IMU own that axis, or by refusing to
  trust the estimate. Wildcat's tightly-coupled window is why it walked through the QCAT
  tunnel where FAST-LIO2 and LIO-SAM slipped."
- **`piros2` line:** rotation-only Kabsch on bearing rays has its own degeneracy — a
  chessboard's lookalike corners defeat the cross-check by design (a test pins it), a
  covered lens yields nothing (the blackout fix), and pure rotation makes the essential
  matrix degenerate (why Kabsch was chosen at all). Recognising a degenerate estimate and
  counting it as *lost* rather than *zero motion* is exactly the `was_tracking` change.

## What to say if asked "have you built SLAM?"

"No — `piros2` estimates rotation-only orientation from ORB matches, fuses monocular depth
into a TSDF under those poses, and relocalises against a keyframe store; it has a frontend
and a relocaliser, no backend, no loop closure, no IMU. I've *run and tuned* RTAB-Map's
RGB-D odometry and used its optimised poses offline, and I've measured what unoptimised
poses do to a mesh. I can talk through Wildcat's structure — surfels, the sliding-window
B-spline, 6-second submaps, Cauchy IRLS — and where my pipeline would need a backend to
become SLAM." Then stop.
