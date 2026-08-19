# State estimation and maths — the study file for section 3 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at
"can hold a technical conversation" depth, the sentence an interviewer is fishing for, and
an honest **`piros2` line** — what the repo actually does, verified against the code before
it was written. The syllabus gives this section no priority note; it is breadth under
sections 8 and 2, but it is the *language* every SLAM and control conversation is held in,
so the vocabulary has to be exact. Per the honest-claim rule: reading ≠ holding — a tick
here means the conversation can be held; the `piros2` lines say what has been built, and
"not touched" means not touched. The real touchpoints are `se3.py` (SE(3) algebra, both
quaternion conversions, and since the SLAM plan's P2 the Lie exp/log/adjoint), Kabsch on
bearing rays in `keypoint_detector.py`, Umeyama-without-scale for relocalisation, a
hand-written Huber-robust Gauss–Newton pose graph checked against `g2o`, and the
latest-TF-vs-interpolated-TF decision.

## Mental model to carry through the whole file

```
represent a pose  ─►  SO(3) / SE(3)   (§1–3: which numbers, which algebra, how to perturb)
        │
estimate it       ─►  filters (KF/EKF/UKF, complementary, particle)  §4–6
        │              or smoothers (least squares on a graph)         §8
        │
know how sure     ─►  covariance, information, observability          §7
        │
fit geometry      ─►  RANSAC (which points), Kabsch/Umeyama (what transform)  §9–10
        │
move in time      ─►  interpolate / extrapolate between poses (SLERP)  §11
```

Every estimator in section 2 is built from these pieces: FAST-LIO is §4 (iterated EKF)
on §3 (error state on the manifold); Wildcat is §8 (sliding-window least squares with a
Cauchy kernel) on §11 (a B-spline in place of SLERP); a loop-closure check is §9 + §10 +
§7 (RANSAC, rigid fit, Mahalanobis gate).

## 1. SE(3), SO(3), rigid transforms

- **SO(3)** — the special orthogonal group: 3×3 matrices with `RᵀR = I` and `det R = +1`.
  Nine numbers, six constraints, **3 degrees of freedom**. `det = −1` is a reflection, which
  no physical body can perform — the guard every SVD-based rotation solver needs (§10).
- **SE(3)** — the special Euclidean group: rigid motions `T = [R t; 0 1]`, 4×4 homogeneous
  so that rotate-then-translate is one matrix product. **6 DoF**. Composition
  `T_ac = T_ab · T_bc` (inner frames cancel when the subscripts are read right-to-left);
  inverse `T_ba = [Rᵀ, −Rᵀt]` — no matrix solve; a point maps as `p_a = R_ab p_b + t_ab`,
  where `t_ab` is *frame b's origin expressed in a*.
- **The convention question** is the whole game. `T_ab` "maps b-coordinates into a" and
  "is the pose of b in a" — the same matrix, two readings; pick one and name variables by it
  (`T_world_cam`). tf2's `lookup_transform(target, source)` returns exactly `T_target_source`.
  Passive vs active, row- vs column-vector layouts, Hamilton vs JPL quaternions — the same
  trap in different clothes.
- **Classic mistake:** the inverted transform — plumbing `T_cam_world` where `T_world_cam`
  was meant. It runs, and the point cloud swings around the wrong origin.
- **Interviewer's target sentence:** "SO(3) is rotations, SE(3) is rigid motions; I keep
  transforms as 4×4 homogeneous matrices named `T_parent_child`, compose by cancelling inner
  frames, and invert with `[Rᵀ, −Rᵀt]` — the naming discipline is what stops inverted
  transforms reaching the pipeline."
- **`piros2` line:** `piros2_world_mesh/se3.py` is exactly this — pure numpy, no ROS —
  `make_transform`, `invert` (the `[R.T | −R.T @ t]` identity, docstring says why),
  `transform_points` (row-vector idiom), `BASE_FROM_OPTICAL` (the −90/0/−90 optical
  rotation as `R_base_optical`), and the module docstring pins the `T_ab` reading and notes
  that tf2's `lookup_transform('a','b')` *is* `T_ab`. It exists because two nodes had been
  carrying the same convention implicitly. `test_se3.py` covers it.

## 2. Quaternions vs Euler vs rotation matrices, gimbal lock

| Representation | Numbers / constraints | Compose | Interpolate | Singularities | Where it belongs |
| --- | --- | --- | --- | --- | --- |
| Rotation matrix | 9 / 6 | matrix product, cheap to apply to points | poor (need log/exp) | none | applying to point clouds; internal maths |
| Unit quaternion `(x,y,z,w)` | 4 / 1 (`‖q‖=1`) | Hamilton product | SLERP natural | none; **double cover** (`q ≡ −q`) | storage, messages, filters |
| Euler / RPY | 3 / 0 | no closed form worth using | bad | **gimbal lock** at pitch ±90° (ZYX) | human display, config files, launch args |

- **Gimbal lock**: in a ZYX (yaw-pitch-roll) chain, at pitch = ±90° the roll axis lines up
  with the yaw axis, the map from three angles to a rotation loses rank, and one DoF becomes
  unrepresentable — the *rotation* is fine, the *parameterisation* is degenerate. Numerically:
  `pitch = −asin(R[2,0])` is fine but roll/yaw become `atan2(0,0)`-shaped, and small
  rotations produce large angle jumps. It is why no estimator integrates gyros in Euler.
- **Quaternion facts to have cold:** 12 Euler conventions exist (intrinsic/extrinsic × axis
  order); ROS messages are `(x, y, z, w)` while Eigen's constructor is `(w, x, y, z)`; a
  quaternion drifts off unit length under integration and needs renormalising; a rotation
  matrix drifts off orthogonality and needs re-orthonormalisation (polar decomposition via
  SVD); `q` and `−q` are the same rotation, so a SLERP or an averaging step must flip the
  sign to the shorter arc first (§11); rotation from matrix uses the **Shepperd four-branch**
  method (branch on the trace / largest diagonal to avoid dividing by a small number).
- **Rule of thumb:** store and transmit quaternions or matrices; convert to RPY only at the
  boundary where a human reads it — RTAB-Map's `reset_odom_to_pose` takes RPY, launch args
  take RPY, nothing inside the estimator does.
- **Interviewer's target sentence:** "Euler angles are for humans; quaternions and matrices
  are for maths. Gimbal lock is a parameterisation singularity, not a physical one, and the
  double cover means `q` and `−q` are the same rotation — which bites SLERP and averaging."
- **`piros2` line:** `se3.py` holds both conversions side by side (they used to live in
  different nodes, and the pairing was invisible): `quaternion_from_rotation` is the
  Shepperd four-branch form and `test_se3.py` drives all four branches;
  `rotation_from_quaternion` is its inverse; `euler_from_rotation` (ZYX) exists *only*
  because RTAB-Map's `/reset_odom_to_pose` service wants RPY. The static
  `camera.launch.py` chain takes its −90/0/−90 optical rotation as RPY on the command line
  — the human boundary — and everything after that is matrix.

## 3. Lie groups, Lie algebra, manifold optimisation

- **The problem:** SO(3) has 3 DoF but no global 3-parameter chart without singularities
  (§2), and you cannot add two rotation matrices and get a rotation. Optimisers want to say
  "take this small step" as a plain vector.
- **The answer:** SO(3) and SE(3) are **Lie groups** — smooth manifolds that are also groups.
  The tangent space at the identity is the **Lie algebra**: `so(3)` = skew-symmetric matrices
  ≅ ℝ³ (rotation vector `φ = axis · angle`, `φ^` its skew matrix); `se(3)` ≅ ℝ⁶ (twist
  `ξ = [ρ, φ]`, translation-ish then rotation in the g2o/Sophus ordering). **exp** maps
  algebra → group (`so3_exp` is Rodrigues' formula; `se3_exp` needs the left Jacobian
  `J_l(φ)` to turn `ρ` into a translation), **log** is its inverse (with a special case near
  angle π where the sine formula degenerates).
- **Manifold optimisation** = keep the state on the group, parameterise *increments* in the
  algebra: `T ← T · exp(δ^)` (right perturbation) or `exp(δ^) · T` (left). A Gauss–Newton
  step is then an ordinary 6-vector, and the update stays a valid rotation *for free*. The
  **adjoint** `Ad(T)` moves a twist between the two sides: `T exp(ξ) = exp(Ad(T) ξ) T` — what
  the pose-graph Jacobians need. Generic notation: Hertzberg's ⊞ / ⊟ (`x ⊞ δ`, `y ⊟ x`).
  Libraries: Sophus, manif, GTSAM's Lie types, Ceres `Manifold` (was
  `LocalParameterization` before Ceres 2.1), g2o's vertex `oplus`.
- **Where it shows up:** every modern backend (§8), the error-state Kalman filter (§4), IMU
  preintegration, B-spline trajectories on SE(3) or SO(3)×ℝ³ (Wildcat).
- **Classic mistakes:** optimising Euler angles as three free parameters (works until it
  doesn't); adding a step to a quaternion without renormalising; mixing left and right
  perturbation conventions between the residual and its Jacobian — the optimiser then
  converges slowly or to the wrong place, and it is very hard to see why.
- **Interviewer's target sentence:** "I keep poses on SE(3) and optimise in the tangent
  space: the increment is a 6-vector twist, `T ← T·exp(δ)`, and the adjoint moves twists
  between frames — that is what makes a Gauss–Newton step on a rotation well-defined and
  singularity-free."
- **`piros2` line:** built, tested, and oracle-checked (SLAM plan P2, 2026-08-18): `se3.py`
  grew `hat`, `so3_exp`, `so3_log` (with the near-π branch), `_left_jacobian_so3`,
  `se3_exp`, `se3_log`, `adjoint`; `pose_graph.py` runs Gauss–Newton with right
  perturbation, residual `e = log(Z_ij⁻¹ T_i⁻¹ T_j)`, first-order Jacobians
  `J_r⁻¹(e) ≈ I + ½ ad(e)`, and `test_pose_graph.py` covers Lie round trips, the adjoint
  identity, and the `g2o` oracle (same graph through `/opt/ros/jazzy/bin/g2o -solver
  lm_dense`, positions within 1 mm). Small, dense, one room — but on the manifold.

## 4. Kalman filter, EKF, UKF

- **KF** — the optimal linear-Gaussian estimator. Predict: `x⁻ = F x + B u`,
  `P⁻ = F P Fᵀ + Q`. Update: `K = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹`, `x = x⁻ + K (z − H x⁻)`,
  `P = (I − K H) P⁻` (Joseph form for numerical safety). `Q` = process noise (how much you
  trust the model), `R` = measurement noise; the innovation `z − Hx⁻` and its covariance
  `S = HPHᵀ + R` are what you gate on (§7).
- **EKF** — same recursion with nonlinear `f`, `h` and their Jacobians evaluated at the
  current estimate. Cheap and everywhere (GNSS/INS, `robot_localization`), but the
  linearisation error makes it *inconsistent* (over-confident) when the state is far from
  where you linearised, and the covariance can drive the wrong linearisation point.
  Refinements: **error-state EKF** (nominal state integrated on the manifold, filter runs on
  a small tangent-space error — the standard for IMU attitude, see Solà's tutorial);
  **iterated EKF** (re-linearise the update until it converges — FAST-LIO's core); EKF-SLAM
  is quadratic in landmarks, which is why smoothing replaced it.
- **UKF** — sigma-point transform: choose `2n+1` deterministic points around the mean,
  push each through the nonlinearity, recover mean/covariance. No Jacobians, captures the
  posterior to second order, similar cost per step; wins when the nonlinearity is strong,
  loses nothing else — but sigma points on a manifold need care.
- **Filter vs smoother:** a filter marginalises the past every step (cheap, causal, at IMU
  rate — what a flight controller needs); a smoother keeps a window or a graph and
  relinearises (§8 — Wildcat, LIO-SAM, VINS). Rule of thumb: control loops get filters,
  maps get smoothers, and the two are often both present (FAST-LIO odometry feeding a pose
  graph).
- **Classic mistakes:** tuning Q and R by feel until it "looks smooth" (over-smoothing hides
  latency); feeding correlated measurements as independent.
- **Interviewer's target sentence:** "The EKF is a linearised Kalman filter — predict with
  the model, correct with the innovation weighted by the Kalman gain — and its weakness is
  inconsistency from linearising at the wrong point, which the error-state and iterated
  forms address; a UKF trades Jacobians for sigma points; smoothers relinearise the whole
  window and win on accuracy where the rate allows."
- **`piros2` line:** **not touched** — no IMU, no filter, no `robot_localization`. The
  nearest things are architectural: the odometry that feeds the pose graph is RTAB-Map's
  `rgbd_odometry`, and the SLAM plan's backend is a smoother (§8), so the repo has taken the
  smoother side of the fork without ever building the filter side. That is the honest
  boundary.

## 5. Complementary filter

- Fuse two signals whose errors live in *complementary* frequency bands: a gyro integrates
  to a clean short-term angle that drifts (good high-frequency, bad low-frequency), the
  accelerometer gives a noisy but unbiased gravity direction (good low-frequency, bad
  high-frequency). `θ = α (θ + ω Δt) + (1 − α) θ_acc`, with `α ≈ 0.98` at 100 Hz — the
  crossover time constant is `τ = α Δt / (1 − α)`. No covariance, fixed gains, a few
  multiplies: this is what runs on 8-bit flight controllers.
- Quaternion-form AHRS filters (§4 of the sensors file): **Mahony** (PI feedback on the
  gravity/magnetic direction error), **Madgwick** (one gradient-descent step per sample on
  the same error). Both are complementary filters in disguise; PX4's EKF2 is the Kalman
  alternative. Trade: a Kalman filter adapts its gains and gives you a covariance; a
  complementary filter is deterministic, cheap, and needs one number tuned.
- **Classic mistake:** letting the accelerometer term in during sustained acceleration (a
  turning drone "sees" gravity tilted) — gate or down-weight when `‖a‖ ≠ g`.
- **Interviewer's target sentence:** "A complementary filter is a high-pass on the
  integrated gyro plus a low-pass on the accelerometer tilt — one gain, no covariance; the
  Kalman version is what you use when you need adaptive gains or an uncertainty out."
- **`piros2` line:** no IMU, so no attitude fusion. The honest nearest thing is the same
  spectral idea applied to *scale*: `depth_align.py`'s `ScaleAligner` corrects only each
  frame's deviation from a rolling median of the depth-ratio (window 50, correction clipped
  at ±15 %) — a high-pass on the wobble that, by construction, exerts no net push on the
  map. Conform-to-map directly (a low-pass that trusts the map) was measured *unstable*
  (the ray-cast reads ~1.25 voxels far and the loop walked walls away), which is a
  complementary-filter lesson learned in a different domain: decide which source owns
  which band.

## 6. Particle filter / Monte Carlo localisation

- Represent the posterior by `N` weighted samples instead of a Gaussian. Loop: **predict**
  (move each particle by the motion model + sampled noise), **weight** by the measurement
  likelihood (beam model or the cheaper likelihood-field model against an occupancy map),
  **resample** (low-variance / systematic resampling when the effective sample size
  `1/Σwᵢ²` drops). Handles multimodal beliefs — the thing a Gaussian cannot — so it can do
  **global localisation** and recover from the **kidnapped-robot** problem by injecting
  random particles.
- **AMCL** (Nav2's localiser): adaptive particle count via KLD-sampling (few particles when
  converged, many when uncertain), 2D laser vs a prior map. Cost scales with particles ×
  beams; it works in 2D/3-DoF; in 6-DoF the sample count needed is why 3D localisation goes
  to scan matching or feature-based place recognition instead.
- **Classic mistakes:** too few particles for the initial spread; resampling every step
  (depletion); a likelihood so peaked that one particle takes all the weight.
- **Interviewer's target sentence:** "MCL is a sampled Bayes filter — predict, weight,
  resample — whose strength is multimodality and global localisation and whose cost is
  particles times measurements, which is why it stays 2D."
- **`piros2` line:** **not touched** — no occupancy map, no laser, no AMCL. The nearest
  thing is the relocaliser (`attempt_relocalization`): a *one-shot* global localisation by
  descriptor matching against a keyframe store then RANSAC-PnP / rigid fit — a deterministic
  hypothesis-and-verify, not a sampled belief. The "kidnapped robot" case was exercised —
  `just gate occlude` covers the lens, loses tracking, and re-finds pose against the store
  (Δ 18.4° snap, 0.95°/3 cm tail).

## 7. Covariance, uncertainty propagation, observability

- **Covariance** `Σ` — second moment of the error; its inverse is the **information matrix**
  `Λ = Σ⁻¹`, which is what factor graphs carry because information from independent
  measurements *adds*. A pose graph's edges each carry a 6×6 information matrix; sloppy but
  universal practice is diagonal from `(σ_t, σ_r)`.
- **Propagation** (first order): `Σ_y = J Σ_x Jᵀ`. Composing poses: uncertainties compound
  through the adjoint (`Σ_ac = Σ_ab + Ad(T_ab) Σ_bc Ad(T_ab)ᵀ` for right-perturbation
  errors). The one figure to keep: a heading error `σ_θ` becomes a position error `d · σ_θ`
  after distance `d` — rotational uncertainty is the expensive kind.
- **Mahalanobis distance** `√(rᵀ Σ⁻¹ r)` — a residual measured in sigmas; χ² gating (e.g.
  6-DoF residual, 95 % → χ² 12.6) is how loop-closure candidates and data associations are
  accepted or refused (Wildcat gates edges on it). **NEES** / **NIS** are how you check an
  estimator's covariance is honest — a filter that reports 1 cm and is off by 10 is
  *inconsistent*, and it will reject the good measurements that could save it.
- **Observability** — can the state be inferred from the measurements at all? Standard
  unobservable directions: VIO has a 4-DoF gauge (global position + yaw; roll/pitch are
  pinned by gravity), monocular vision has *scale*, IMU biases need excitation (a hovering
  drone cannot separate accelerometer bias from gravity misalignment), extrinsics need
  rotation, and a straight tunnel makes translation along its axis unobservable to a LiDAR
  (SLAM §17). Nonlinear observability analysis (Hermann–Krener) or simply the spectrum of
  the Hessian tells you which directions are flat.
- **Interviewer's target sentence:** "Covariance says how sure; propagate it with Jacobians
  (adjoints on SE(3)); gate with Mahalanobis; check consistency with NEES; and before any of
  that, ask which directions are observable at all — the estimator cannot invent what the
  sensors do not constrain, it can only drift there."
- **`piros2` line:** touched, in the small: `pose_graph.py`'s `information_matrix(σ_t, σ_r)`
  from `graph_odom_sigma_m/deg` (0.02 m / 1°) and `graph_loop_sigma_m/deg` (0.03 m / 2°) in
  `world_mesh.yaml`; the Huber threshold is in χ units (residual measured by the information
  matrix), and the measured χ² of 2–8 over 17 nodes / 11 loops was read as "a coherent
  graph"; node 0 is fixed to pin the gauge. Observability is a design fact of the whole
  fork: rotation-only Kabsch on bearing rays makes translation *unobservable by construction*
  (and RTAB-Map later measured 0.9 m of real arm-arc translation inside a "rotation-only"
  pan); the monocular scale is unobservable and was pinned by hand (`depth_scale: 2.69`).
  Nothing here propagates a covariance forward or checks NEES.

## 8. Least squares, robust cost functions (Huber, Cauchy)

- **Nonlinear least squares:** minimise `Σᵢ ‖rᵢ(x)‖²_Σᵢ` (weighted by information).
  Linearise `r(x ⊞ δ) ≈ r + J δ`, solve the normal equations `JᵀΛJ δ = −JᵀΛr`
  (**Gauss–Newton**), or damp them `(JᵀΛJ + λ D) δ = −JᵀΛr` (**Levenberg–Marquardt**: `λ`
  small → GN, large → gradient descent; increase `λ` when a step makes the cost worse).
  Sparsity of `J` (each factor touches few variables) is what makes graphs with 10⁴ nodes
  solvable — sparse Cholesky with a good ordering, or the Schur complement in BA.
- **Robust kernels** replace `r²` with `ρ(r)` so outliers get bounded influence. **Huber**:
  quadratic for `|r| ≤ δ`, linear beyond — convex, gentle, the safe default. **Cauchy**:
  `c²/2 · log(1 + r²/c²)` — non-convex, influence *decays* for large residuals, so gross
  outliers are nearly ignored (Wildcat's choice, applied by IRLS). Others: Geman–McClure,
  Tukey biweight (redescending — zero influence past a threshold), Barron's adaptive loss.
  Implementation: **IRLS** — reweight each residual by `w = ρ′(r)/r` and re-solve. Ceres
  exposes them as `LossFunction`; g2o as `RobustKernel`.
- **Rules of thumb:** the kernel width is in *sigmas* (normalise by covariance first — a
  Huber δ of 1–2 σ is common); a too-tight kernel throws away good data and slows
  convergence; robust kernels do not fix a *systematic* error, only a sparse one; and a
  wrong loop closure inside a convex kernel still bends the graph — Cauchy or switchable
  constraints if you must survive it.
- **Interviewer's target sentence:** "Gauss–Newton on the normal equations, LM when the
  linearisation is poor, information-weighted residuals, and a robust kernel — Huber for
  gentle, Cauchy for redescending — applied by IRLS so one bad loop closure bends the graph
  rather than breaking it."
- **`piros2` line:** built (SLAM plan P2, 2026-08-18): `pose_graph.py` `optimize()` is
  Gauss–Newton with Levenberg damping (`damping=1e-4`, back off and re-damp when χ² rises),
  dense `6N×6N` normal equations (a room ≤ a few hundred nodes; sparse Cholesky is what g2o
  adds), and a **Huber** kernel on loop edges only (`graph_huber: 2.0`, χ units) applied
  as an information down-weight `info · (huber / r)`. Measured in tests: a planted wrong
  closure ("node 12 is node 0") folds the naive graph 1.9 m and moves the Huber one < ¼ of
  that; the drifted 24-node circle closes to within 6 cm per node with the correction spread
  over the loop; against `g2o` the two optima sit 0.07° apart at identical χ² to six
  decimals. Live: `just gate-loop own` raw tail 6.1 cm / 1.90° → 2.3 cm / 0.85°;
  `gate-tum own` fr1/desk ATE 0.163 → 0.089 m. Cauchy has not been implemented.

## 9. RANSAC

- **Random Sample Consensus** (Fischler & Bolles 1981): repeatedly draw the *minimal* set of
  points needed to fit the model (3 for a plane, 3–4 for PnP, 5 for the essential matrix,
  8 for the fundamental), fit, count inliers within a threshold, keep the best consensus
  set, then refit on all its inliers. It answers "which correspondences are right" before
  Kabsch/PnP/least squares answers "what transform" — the two halves of every registration
  and loop-closure verification.
- **The one formula:** iterations `N = log(1 − p) / log(1 − (1 − ε)ˢ)` for success
  probability `p`, outlier fraction `ε`, sample size `s`. At `p = 0.99`, `ε = 0.5`: `s = 3`
  → 35, `s = 5` → 145, `s = 8` → 1177. Minimal sets matter *exponentially*, which is why
  minimal solvers (P3P, 5-point) are prized. Thresholds are in the residual's units — 1–3 px
  reprojection for vision, a few cm for planes.
- **Variants:** MSAC (truncated cost, not count), PROSAC (sample the best-scored matches
  first), LO-RANSAC (locally optimise each new best), MAGSAC++ (marginalises the threshold),
  adaptive termination once the inlier ratio is known.
- **Classic mistakes:** not refining on the inliers; a threshold set without thinking about
  the noise; degenerate samples (three collinear points, coplanar for the 8-point); using
  RANSAC where a robust kernel and a good initial guess would do (they are complementary —
  RANSAC for association, kernels for the final solve).
- **Interviewer's target sentence:** "RANSAC finds the consensus set by fitting minimal
  samples; the iteration count grows exponentially with sample size, so minimal solvers
  matter; verify, then refit on the inliers with a robust least squares."
- **`piros2` line:** used in three shapes. Proper RANSAC: `pnp_pose` in
  `keypoint_detector.py` — `cv2.solvePnPRansac` (EPnP, 300 iterations, 6 px reprojection,
  confidence 0.999) then `solvePnPRefineLM` on the inliers, the loop-closure measurement the
  SLAM plan adopted after a two-depth rigid fit *regressed* translation; and
  `tools/recon/room_layer.py`'s `segment_plane` (`ransac_n=3`, 1000 iterations) for floor
  and walls. Not-RANSAC-but-honest: `estimate_rotation` and `rigid_transform_3d` use two
  rounds of drop-the-worst-20 %-and-refit — a trimmed least squares standing in for RANSAC
  because descriptor matching leaves only a few percent of false pairs; the docstrings say
  so.

## 10. Kabsch / Procrustes, Umeyama alignment

- **The problem:** given corresponding point sets `pᵢ ↔ qᵢ`, find `R, t` (and optionally
  scale `s`) minimising `Σ ‖s R pᵢ + t − qᵢ‖²`. Closed form, one SVD:
  1. centroids `p̄, q̄`; centre both sets;
  2. cross-covariance `H = Σ (pᵢ − p̄)(qᵢ − q̄)ᵀ`, SVD `H = U S Vᵀ`;
  3. `R = V · diag(1, 1, det(V Uᵀ)) · Uᵀ` — the middle term is the **reflection guard**
     (without it a coplanar or noisy set can return `det = −1`);
  4. `t = q̄ − s R p̄`; **Umeyama (1991)** adds `s = tr(S · diag(1,1,±1)) / σ²_p` — a Sim(3)
     fit, which is what a monocular trajectory needs to be compared to ground truth.
  Kabsch = rotation only; orthogonal Procrustes = the same problem for matrices; Horn's
  1987 quaternion method gives the identical answer by another route.
- **Where it lives:** the inner solve of every point-to-point ICP iteration; ATE evaluation
  (`evo` aligns with Umeyama SE(3) or Sim(3)); hand-eye and extrinsic calibration;
  loop-closure verification after RANSAC has picked the inliers.
- **Classic mistakes:** forgetting the reflection guard; feeding it correspondences that are
  wrong (it minimises the *sum of squares*, so one bad pair moves the answer — RANSAC or
  trimming first); reading `R` in the wrong direction (`q ≈ R p` vs `p ≈ R q` — the transpose).
- **Interviewer's target sentence:** "Kabsch is the closed-form rigid fit — centre, SVD of
  the cross-covariance, fix the reflection — Umeyama adds scale for Sim(3); it is the inner
  step of ICP and the way trajectories are aligned for ATE, and it is only as good as the
  correspondences you hand it."
- **`piros2` line:** the repo's central maths, three times over. `kabsch()` on **bearing
  rays** (`rays_from_pixels` unprojects through K to unit vectors; no centroid step because
  rays pass through the optical centre — a pure rotation, chosen because the essential
  matrix is degenerate under pure rotation) with the det guard, and
  `test_kabsch_reflection_guard_on_coplanar_rays` pins the reflection case; measured on a
  static bag the composed orientation held within ~0.001° of identity. `rigid_transform_3d`
  in `se3.py` is Umeyama *without* scale (the docstring says so) for relocalisation against
  stored 3D landmarks. And `tools/verify/traj_check.py` carries a full `umeyama(src, dst,
  with_scale)` — SE(3) or Sim(3) — to align estimated trajectories to RTAB-Map / TUM ground
  truth before ATE (`--align se3|sim3`), which is how the SLAM gates are scored.

## 11. Pose interpolation and extrapolation (SLERP)

- **SLERP** (Shoemake 1985): constant-angular-velocity interpolation between unit
  quaternions, `slerp(q₀, q₁, α) = sin((1−α)θ)/sin θ · q₀ + sin(αθ)/sin θ · q₁`,
  `cos θ = q₀·q₁`. Two details that are the whole implementation: flip `q₁ → −q₁` when the
  dot is negative (shorter arc — the double cover again), and fall back to normalised
  linear interpolation when `θ` is tiny (`sin θ → 0`). NLERP is the cheap approximation;
  on the manifold it is `q₀ · exp(α · log(q₀⁻¹ q₁))`, which is also how you interpolate on
  SE(3) (or, more commonly, SLERP the rotation and lerp the translation separately — the two
  are not the same curve, and both are fine for small gaps).
- **Extrapolation** = predicting a pose beyond the last sample: constant velocity /
  constant twist (`T(t) = T_last · exp((t − t_last) ξ)`), or an IMU-propagated prediction.
  Needed for latency compensation (render the map where the camera *is*, not where it was
  30 ms ago) and for ICP initial guesses (KISS-ICP's constant-velocity model). Extrapolation
  is where estimators go wrong loudly — a 100 ms extrapolation at 90°/s is 9°.
- **tf2** does exactly this: a `lookup_transform` at a stamp between two buffered
  transforms **interpolates** (SLERP + lerp); before the earliest or after the latest it
  refuses (extrapolation-into-the-future error) — which is why `Time()` (latest available)
  and `Time(stamp)` are different requests, and why the buffer's cache time matters.
  Continuous-time SLAM (Wildcat's cubic B-spline over SO(3)/SE(3)) is this idea promoted
  from a two-sample interpolant to a whole-trajectory function you can query per LiDAR
  point.
- **Interviewer's target sentence:** "SLERP is the constant-angular-rate quaternion
  interpolant — take the shorter arc, guard the small angle — and tf2 uses it between
  buffered transforms; extrapolation is a motion-model prediction and should be bounded and
  short."
- **`piros2` line:** both requests are used, deliberately, in different places. Mapper,
  mesher and `cloud_projector` do **latest-TF lookups** (`Time()`) — because `/image_raw`
  header stamps lag wall clock by a steady ~0.73 s (the camera fault), a stamp-exact lookup
  would either fail or return the wrong pose, so the rule became "latest transform, never
  the faulted stamp"; the projector even publishes `/points` already in `odom` so RViz's own
  wait-for-TF-at-stamp cannot flap. The SLAM plan then did the opposite where it mattered:
  `keypoint_detector._lookup_at` asks tf2 for `odom → base_link` **at the image stamp**
  (interpolated between `rgbd_odometry` samples, after waiting `sync_min_delay_s: 0.25` for
  the ~0.2 s-late TF), because landmark geometry built from a stale pose measurably hurt the
  loop-closure translation. And `tools/verify/traj_check.py` has its own `slerp` (dot-flip,
  small-angle fallback) and `Trajectory.interpolate` (lerp + SLERP, `max_gap` refusal —
  the same interpolate-don't-extrapolate rule as tf2) to line up trajectories with ground
  truth stamps.

## What to say if asked "have you built a state estimator?"

"Not a filter — no IMU on the rig, no Kalman/EKF, no `robot_localization`. What I have
built is the smoother side: an SE(3) library with the Lie exp/log/adjoint, Kabsch on
bearing rays and Umeyama for relocalisation, RANSAC-PnP for loop-closure measurement, and a
hand-written Huber-robust Gauss–Newton pose graph on the manifold that I checked against
g2o to a millimetre and measured on replays against RTAB-Map and TUM ground truth. I can
talk through the EKF, error-state, UKF and particle-filter trade-offs, and I know which
piece I would need to add for a drone's inner loop — a filter at IMU rate — and why it is
not in this repo." Then stop.
