# Sensors — the study file for section 4 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist bullet: the concept at "can hold a technical
conversation" depth, the sentence an interviewer is fishing for, and an honest **`piros2`
line** — what this repo has actually measured or been bitten by, versus what is reading only.

This section is breadth, not one of the two priority gaps, but it is the section where
`piros2` has the most *scar tissue*: the C922's timestamp fault, its persistent V4L2 state,
the fabricated-intrinsics decision and the tape-measure scale check are all real sensor
lessons with numbers attached. Say those with confidence; say the LiDAR and IMU material as
reading. Reading a section does not tick its box.

## Mental model

Every sensor is four things, and an interviewer is testing whether you think in all four:

| | Question | Where it bites |
| --- | --- | --- |
| **What it measures** | the physical observable, not the number in the message | a depth camera measures disparity or time, not "metres" |
| **In what frame, at what time** | extrinsics + timestamp | the whole of SLAM is a frame-and-time problem wearing a hat |
| **With what error model** | bias, noise, drift, saturation, dropout | the difference between fusing and averaging garbage |
| **How it fails** | and whether the failure is loud or silent | silent failure is the expensive one |

A sensor whose *failure is silent* is the one that ends up in an incident report. `piros2`
has two of them on record: a camera that reported timestamps 0.73 s in the past while the
frames were live, and a GPU inference path that silently fell back to CPU.

## 1. LiDAR: spinning vs solid state, channels, range, FOV, returns, intensity

- **Spinning (mechanical):** a stack of laser/detector pairs on a rotating head — 16, 32, 64,
  128 **channels** — sweeping 360° horizontally at 5–20 Hz. Vertical FOV is the fixed fan of
  the channels (VLP-16: 30°, ±15°, 2° between rings); horizontal resolution is set by the
  firing rate against the spin rate, so *slowing the spin buys angular resolution and costs
  update rate*. Rings are the reason "ring number" is a per-point field: it identifies which
  laser fired, which is how LOAM-family systems compute per-scan-line curvature.
- **Solid state / semi-solid:** MEMS mirrors, optical phased arrays, or rotating prisms
  (Livox's non-repetitive scan). No large moving mass — cheaper, more robust to vibration,
  usually a *limited* FOV (a forward cone) with a non-uniform pattern that fills in over time.
  The catch for SLAM: a non-repetitive pattern means consecutive scans aren't comparable
  point-for-point, so feature-based matching has to be adapted.
- **Range** is a function of reflectivity, not a single number: a spec's "100 m" is usually at
  80 % reflectivity (a road sign); dark rock at 10 % might return at a third of that. This is
  exactly why underground range specs and surface range specs differ, and why Emesent's
  ST → ST-X jump (100 m → 300 m, 16 → 32 channels, 600k → 640k/1.92M points/s) is a real
  capability change and not marketing.
- **Returns:** one pulse can produce several echoes — the first off foliage or dust, the last
  off the ground behind it. Dual/triple return gives you vegetation penetration and, crucially
  underground, some robustness to **dust**: the first return is the dust cloud, the last is
  the wall. "Strongest/first/last" are the standard selectors and appear as a per-point field.
- **Intensity** is the returned energy (uncalibrated on most units): it encodes reflectivity,
  range and incidence angle mixed together. Useful as a *feature* channel (retroreflective
  survey targets saturate it, which is how automated ground-control target detection works),
  usable for place recognition, and not directly a material property unless calibrated.
- **Interviewer's target sentence:** "Channels set vertical resolution, spin rate trades
  horizontal resolution against update rate, range specs are quoted at a reflectivity, and
  multi-return is what keeps you alive in dust — which is why underground scanning cares
  about it more than a surface survey does."
- **`piros2` line:** no LiDAR in this repo at all — the sensor is a Logitech C922 webcam and
  the depth is inferred by a neural network. What transfers is the *shape* of the reasoning:
  the repo already treats depth as a measurement with an error model (a measured ±4 % per-frame
  scale wobble on a static scene) rather than as truth.

## 2. Motion distortion and scan deskewing

- A spinning LiDAR "scan" is not a snapshot: at 10 Hz, the last point in a sweep is measured
  100 ms after the first, and in that time a robot at 1 m/s has moved 10 cm and a drone
  yawing at 90°/s has turned 9°. Points are stamped in the sensor frame *at different sensor
  poses*, so building a cloud from them naively smears the geometry — walls bend, corners
  double. This is **motion distortion** (skew).
- **Deskewing** = transform every point into a single reference pose using an estimate of the
  trajectory over the sweep. The estimate can come from: a constant-velocity model (KISS-ICP),
  the previous odometry solution (LOAM's linear interpolation), IMU integration over the sweep
  (LIO-SAM, FAST-LIO2's backward propagation), or — the principled version — a **continuous-time
  trajectory** you can evaluate at each point's own timestamp (Wildcat; see
  [02_SLAM.md](02_SLAM.md) §12–13).
- The chicken-and-egg: you need the motion to deskew, and you want the deskewed cloud to
  estimate the motion. Solutions iterate (deskew → match → re-deskew) or lean on the IMU,
  which is fast enough to be trusted over 100 ms.
- **Per-point timestamps are the enabling requirement.** If your driver publishes a cloud
  without a `time`/`t` field per point, you cannot deskew properly — a real reason to care
  about driver configuration.
- **Interviewer's target sentence:** "A sweep is a trajectory, not a pose; you deskew by
  evaluating the trajectory at each point's timestamp, and the quality of that trajectory —
  constant-velocity vs IMU vs a continuous-time spline — is the accuracy difference between
  a smeared wall and a flat one."
- **`piros2` line:** a rolling-shutter webcam has the same disease in miniature (§9) and the
  repo does *not* correct it. What it does have is the discipline that deskewing depends on:
  per-message timestamps treated as suspect, and processing cost measured against a node's own
  clock rather than `header.stamp` — see §12.

## 3. Time-of-flight vs phase-shift ranging

- **Time-of-flight (pulsed):** fire a short pulse, measure the round-trip time, `d = c·t/2`.
  Because `c` is 0.3 m/ns, 1 cm of range accuracy needs ~67 ps of timing resolution — which is
  why the electronics, not the optics, dominate the cost. Strengths: long range, high pulse
  energy allowed (eye-safety is about average *and* peak power), multiple returns fall out
  naturally. This is what mobile/mapping LiDAR uses.
- **Phase-shift (AMCW):** modulate a continuous beam, measure the phase difference of the
  returned signal. Very high precision at short range, but ambiguity every wavelength — hence
  multiple modulation frequencies to disambiguate — and typically shorter range for a given
  power. Terrestrial laser scanners (the tripod survey instruments — Leica RTC360-class) use
  phase-shift, which is why they beat mobile scanners on precision and lose on speed.
- **FMCW** is the coming third option: chirp the frequency and measure the beat, which yields
  range *and radial velocity per point* and is immune to interference from other LiDARs. The
  reason to know it: per-point velocity would change SLAM (dynamic-object rejection for free).
- The commercial consequence, and the one to say out loud: a mobile SLAM scanner is not
  competing with a TLS on precision — it is competing on *time-to-deliverable*. Emesent's own
  white paper measures the ST-X against a Leica RTC360 (6.7 mm 1σ on a short walk) and the
  pitch is minutes instead of hours, not "better than a tripod".
- **Interviewer's target sentence:** "ToF buys range and multiple returns, phase-shift buys
  short-range precision; mobile mapping picks ToF and makes up the precision gap with SLAM,
  loop closure and control points."
- **`piros2` line:** neither — monocular RGB into a network. The relevant honesty is that the
  repo's depth has *no ranging physics behind it at all*, which is why its scale had to be
  pinned by tape measure (§8).

## 4. IMU: accelerometer, gyroscope, magnetometer; 6 vs 9 axis; MARG; AHRS

- **Accelerometer** measures **specific force** — the non-gravitational acceleration — so at
  rest it reads +1 g *upwards*, not zero. That is the whole trick behind attitude: at low
  dynamics the accelerometer vector points along gravity and gives you roll and pitch
  absolutely. It says nothing about yaw.
- **Gyroscope** measures angular rate. Integrating it gives attitude that is smooth and
  accurate short-term and drifts without bound long-term.
- **Magnetometer** measures the local magnetic field, giving an absolute yaw reference — *if*
  the field is the Earth's. Indoors, underground, near rebar, mesh, steel sets, motors and
  magnetic ore it is not, which is precisely why a stock drone's heading collapses in a mine
  (see [01_emesent-company-domain.md](01_emesent-company-domain.md) §6). Needs hard-iron
  (offset) and soft-iron (scale/skew) calibration, and the calibration is platform-specific.
- **6-axis** = accel + gyro (roll/pitch observable, yaw drifts). **9-axis** = plus
  magnetometer (yaw referenced). **MARG** = Magnetic, Angular Rate and Gravity — the sensor
  triad; **AHRS** = Attitude and Heading Reference System — the *algorithm* on top (Mahony,
  Madgwick, or an EKF) that outputs an orientation. An IMU gives you rates and forces; an AHRS
  gives you an attitude. Interviewers use the distinction to check you know a filter is
  involved.
- **INS** goes one step further: integrate accelerometer *and* attitude to get velocity and
  position — which double-integrates noise and bias, so an unaided INS drifts cubically in
  position. Cortex's navigation ladder (SLAM → GPS → INS, with INS tolerated only ~10 s before
  return-to-home) is exactly an admission of this.
- **Interviewer's target sentence:** "Accel gives absolute roll/pitch through gravity, gyro
  gives smooth short-term rates and long-term drift, magnetometer gives yaw and lies
  underground — so a 9-axis AHRS is fine outdoors and a GPS-denied robot has to get yaw from
  the map instead."
- **`piros2` line:** no IMU. The repo's orientation comes from image features alone (ORB
  matches → Kabsch on bearing rays), which is the *complement* of an AHRS: absolute-ish
  short-term structure, no gravity reference, and drift that only a relocalisation against
  stored keyframes corrects.

## 5. IMU bias, random walk, Allan variance, temperature drift

- **Bias** is a slowly varying offset added to the true measurement. Split into **bias
  repeatability** (different each power-up — so it must be *estimated online*, which is why
  bias is part of the state vector in every tightly-coupled system) and **bias instability**
  (drifts during a run).
- **Noise densities:** angle random walk (ARW) for the gyro, °/√h; velocity random walk (VRW)
  for the accelerometer, m/s/√h. The √ is the signature of integrating white noise: attitude
  error from gyro white noise grows as √t, and *bias* error grows as t — so bias dominates
  quickly. Position from accelerometer bias grows as ½·b·t², which is why an unaided INS is
  hopeless in seconds-to-minutes.
- **Allan variance** is the standard characterisation: log the IMU at rest for hours, compute
  the Allan deviation over averaging windows τ, and read the parameters off the log-log plot —
  slope −½ region = white noise (ARW/VRW), the flat minimum = bias instability, slope +½ =
  rate random walk. This is how you get the numbers a Kalman filter or a preintegration factor
  needs; tools like `imu_utils`/`allan_variance_ros` automate it. Knowing this by name is a
  reliable signal to an interviewer that you have actually tuned an estimator.
- **Temperature drift:** bias and scale factor move with temperature, and a drone's IMU
  self-heats after power-on. Mitigations: factory temperature calibration tables, warm-up
  periods before initialisation, keeping the IMU thermally stable, and letting the estimator
  re-learn bias continuously. "Don't move for ten seconds at the start" — Emesent's own
  instruction to operators — is partly initialisation of bias and gravity.
- **Interviewer's target sentence:** "Bias is not a constant you calibrate once; you estimate
  it online, you characterise the noise with an Allan variance plot to set the filter's
  parameters, and you expect it to move with temperature."
- **`piros2` line:** not touched — no IMU. The nearest equivalent reasoning in the repo is the
  depth model's *scale* being treated as a slowly varying quantity corrected by a rolling
  median (`ScaleAligner` as a high-pass, so drift is impossible by construction) rather than
  a constant calibrated once. Same instinct, different sensor.

## 6. IMU preintegration

- **The problem:** in a smoothing/optimisation framework, the IMU relates two keyframes. Naively,
  every time the optimiser changes the pose or bias at keyframe *i*, you would have to
  re-integrate hundreds of IMU samples to get the constraint to keyframe *j*. That is fatal to
  performance.
- **The trick** (Lupton & Sukkarieh 2012; Forster et al. 2015/2017 on the SO(3) manifold):
  integrate the IMU *once*, in the **body frame of the first keyframe**, into a relative motion
  increment (ΔR, Δv, Δp) that does not depend on the initial pose or velocity — gravity and the
  initial state are factored out analytically at optimisation time. Store the increment plus its
  covariance plus **Jacobians with respect to the bias**, so when the optimiser nudges the bias
  estimate, the increment is *corrected to first order* instead of re-integrated.
- The result is a single factor in the graph between two keyframes, cheap to evaluate, with a
  proper covariance — which is what makes tightly-coupled visual-inertial (VINS-Mono, ORB-SLAM3)
  and LiDAR-inertial (LIO-SAM) smoothing real-time. GTSAM ships it.
- Watch-outs: it assumes the bias is roughly constant over the interval (so keyframes can't be
  too far apart), and it needs good time synchronisation and extrinsics.
- **Interviewer's target sentence:** "Preintegration turns hundreds of IMU samples between
  keyframes into one relative-motion factor with a covariance and bias Jacobians, so the
  optimiser can change the bias without re-integrating — it's what made tightly-coupled
  smoothing tractable."
- **`piros2` line:** not touched. The concept has an analogue the repo *does* use: precomputing
  an expensive relationship once and correcting it cheaply later, rather than recomputing —
  the byte-identical frame CRC skip and the map's running weighted average are both that shape,
  but neither is preintegration and I would not claim it as such.

## 7. Depth cameras: stereo, structured light, time of flight

- **Passive stereo:** two cameras, known baseline `b` and focal `f`; match a pixel in one to the
  other, disparity `d` gives depth `Z = f·b/d`. Consequences worth reciting: depth error grows
  with **Z²** (`σ_Z = Z²·σ_d/(f·b)`), so doubling range quadruples uncertainty; texture-free
  surfaces produce no matches (a white wall is invisible); the baseline sets both the minimum
  range and the far-field accuracy. Rectification and calibration are prerequisites.
- **Active stereo / structured light:** project a pattern (dots, stripes) to create texture —
  Intel RealSense D4xx projects a dot pattern and still does stereo matching; the original Kinect
  correlated a known speckle pattern. Fixes the texture problem, fails in sunlight (the projector
  loses against the sun) and with multiple units interfering.
- **Time-of-flight cameras** (Kinect v2, PMD, and the ToF sensors in Leica's BLK2GO PULSE):
  per-pixel modulated ToF. Dense, works in the dark, low resolution, and prone to *multipath* —
  light bouncing off two surfaces before returning makes corners read too far. Also flying pixels
  at depth discontinuities.
- **The general rule to state:** each depth technology fails on a *different* material — stereo on
  texture-free, structured light in sunlight, ToF on corners and specular/absorbing surfaces, and
  all of them on glass and mirrors. That is why a scanner for real buildings ends up with LiDAR
  plus targets rather than a depth camera.
- **Interviewer's target sentence:** "Stereo error goes as Z²/(f·b) and dies without texture,
  active stereo fixes texture and dies in sun, ToF is dense and suffers multipath — you pick by
  the material you have to survive, not by the spec-sheet range."
- **`piros2` line:** the repo has none of these — it produces depth from a single RGB stream via
  Depth Anything V2 Small (ONNX), then hand-projects it through the camera matrix K into a
  `PointCloud2`. It knows the Z² lesson from the other side: the far parts of the neural depth
  map are where the mesh shows a "far sheet", visible in the offscreen renders (`just mesh-views`).

## 8. Monocular depth estimation and its scale ambiguity

- **The fundamental problem:** a single pinhole image is invariant to scale. A scene twice as big
  at twice the distance projects identically, so structure-from-motion and monocular networks alike
  can recover depth only **up to an unknown scale factor** (and, for many networks, only up to an
  affine transform — *relative* rather than *metric* depth). No amount of network quality removes
  this; it is geometry, not learning.
- **How scale gets fixed in practice:** a second camera (stereo baseline), an IMU (accelerometer
  gives metric acceleration, so scale becomes observable under sufficient *excitation* — constant
  velocity leaves it unobservable), a known object size, contact with a known ground plane, a
  laser rangefinder, or a measurement by hand.
- **Learned monocular depth** (MiDaS, DPT, Depth Anything, Metric3D, UniDepth): trained on mixed
  datasets with scale-invariant losses, so outputs are typically *relative* — an inverse-depth-like
  quantity needing an affine fit to become metres. "Metric" variants condition on intrinsics.
  Their failure modes are worth naming: they hallucinate plausible geometry on ambiguous surfaces,
  they are *not* temporally consistent frame-to-frame (each frame gets its own scale), and they
  degrade off-distribution (a mine drift is nothing like their training set).
- **Interviewer's target sentence:** "Monocular depth is scale-ambiguous by construction —
  the network gives you relative depth and you need an IMU, a baseline, or a measurement to make
  it metric; and because it's per-frame, the scale wobbles frame to frame, which is a problem the
  moment you fuse it."
- **`piros2` line:** **this is the repo's strongest sensor story.** Depth Anything V2 Small runs
  as fp32 ONNX (checksum-pinned, git-ignored, fetched by `just fetch-model`) at 72–79 ms/frame on
  a GTX 1660 SUPER via CUDA (280–305 ms on CPU) — and it silently falls back to CPU if the NVIDIA
  pip libraries are missing or `preload_dlls()` isn't called, so the node logs the winning
  provider deliberately. The scale was pinned **by tape measure**: a wall at a known 2.50 m read
  9.30 m at `depth_scale: 10`, giving **2.69**, verified to +0.1 % on re-export. The per-frame
  scale wobble was *measured* at ±4 % on a static scene, and the fix (`depth_align.py`'s
  `ScaleAligner`) is a high-pass — correct only the deviation from a rolling ratio median —
  after conform-to-map proved unstable (the TSDF ray-cast reads ~1.25 voxels far and the feedback
  loop walked walls away). Measured benefit: placement spread 4.0 % → 2.9 %; the residual is
  spatially structured model error a global scale cannot touch.

## 9. Cameras: intrinsics, distortion, rolling vs global shutter, exposure

- **Intrinsics:** the pinhole matrix `K = [[fx,0,cx],[0,fy,cy],[0,0,1]]` — focal lengths in pixels
  and the principal point. `fx` in pixels = (focal length in mm) × (pixels per mm), which is why
  it changes with resolution and why a "35 mm equivalent" number is useless without the sensor
  size. Projection is `u = fx·X/Z + cx`; the inverse — a **bearing ray** per pixel — is what
  turns a pixel into a direction.
- **Distortion:** radial (barrel/pincushion, `k1,k2,k3`) and tangential (`p1,p2`) in the standard
  Brown–Conrady model, with fisheye/equidistant and Kannala–Brandt models for wide lenses.
  Uncorrected distortion looks like a systematic bias that grows toward the image edges — it will
  quietly curve your straight walls.
- **Rolling shutter:** CMOS sensors expose row by row, so a frame taken during motion is sheared
  (the "jello" effect); a rotating drone or a vibrating mount makes each row a different pose —
  the image-domain twin of LiDAR motion distortion. **Global shutter** exposes all pixels at once
  and is what any serious VIO or photogrammetry rig uses. Rolling-shutter-aware SLAM exists but
  buying a global shutter is cheaper than modelling one.
- **Exposure:** auto-exposure changes the *frame rate* on many webcams (longer integration = fewer
  frames), changes appearance between frames (breaking photometric/direct methods), and motion
  blur destroys corners — features vanish exactly when you are moving fast, which is when you need
  them. Fixed exposure with gain set for the room is the standard robotics answer; HDR scenes
  (a lit portal in a dark drift) are the hard case.
- **Interviewer's target sentence:** "Intrinsics turn pixels into bearing rays and are
  resolution-dependent; distortion is a systematic error that grows to the edges; rolling shutter
  is motion distortion in the image; and auto-exposure trades frame rate and appearance stability
  for brightness — so a perception camera runs fixed exposure and, ideally, global shutter."
- **`piros2` line:** the C922 is rolling-shutter and its behaviour was measured rather than
  assumed. `exposure_dynamic_framerate=1` is the camera's power-on default (the driver *reports*
  the default as 0 — it lies) and it costs 18–21 fps instead of 30; **V4L2 controls persist inside
  the camera across processes and reboots**, so a manual exposure left by a benchmark makes every
  later session black — `just camera` prints every control current-vs-default and
  `just camera-reset` restores the baseline. Under that baseline the old "30 fps ceiling" fell:
  42–60 *distinct* frames/s at true 1280×720 MJPG (0 duplicate payloads in 634 messages). Gain is
  never auto-adjusted on Linux, so dim rooms need it raised explicitly.

## 10. Camera calibration (checkerboard, reprojection error)

- **The procedure:** show a target with known geometry (checkerboard, ChArUco — which tolerates
  partial views and is now preferred, or a circle grid) in many poses covering the whole image and
  a range of distances and tilts; detect corners to sub-pixel accuracy; solve for intrinsics,
  distortion and each board pose by minimising **reprojection error** — the pixel distance between
  detected corners and where the model says they should project (Zhang's method for the initial
  estimate, then bundle-adjustment-style refinement).
- **Reading the result:** RMS reprojection error of ~0.1–0.5 px is healthy for a decent camera; a
  low RMS on a poor set of views means overfitting, not accuracy. The diagnostics that matter are
  *coverage* (corners across the whole frame, including the edges where distortion lives), pose
  diversity (tilt the board — a fronto-parallel-only set cannot separate focal length from
  distance), and the residual *pattern* (structured residuals mean the model is wrong, e.g. a
  fisheye fitted with a pinhole model).
- **Why it matters downstream:** K scales the entire point cloud. A 3 % error in `fx` is a 3 %
  error in every X and Y at a given depth — which shows up as a room that doesn't close, and is
  indistinguishable from a depth-scale error unless you separate them deliberately.
- **Interviewer's target sentence:** "Calibration is a least-squares fit minimising reprojection
  error; RMS alone doesn't tell you it's good — you need coverage to the image edges and pose
  diversity, or you've fitted a model that only works in the middle of the frame."
- **`piros2` line:** honest and specific — the checkerboard calibration is **still unfinished**.
  A live session debugged `just calibrate` into working shape (three real recipe bugs fixed: a
  dead `-p camera:=` service remap, the calibrator ignoring window close, and a `kill %N` trap
  orphaning nodes) but ended without a save. What unblocked the pipeline was
  **spec-derived approximate intrinsics** — `c922_720p_approx.yaml`: 78° diagonal FOV →
  fx = fy ≈ 907 px, centred principal point, zero distortion — published via `camera_info_url`
  and verified live on `/camera_info`. Good to a few percent; the measured yaml is an accuracy
  upgrade, not a blocker. That is the right kind of engineering answer: unblock with a stated
  approximation, record the error budget, keep the real measurement on the list.

## 11. Extrinsic calibration between sensors

- **What it is:** the rigid transform (and often a time offset) between sensor frames — camera↔IMU,
  LiDAR↔IMU, LiDAR↔camera, sensor↔vehicle. Errors here are *systematic*: a 1° rotational error
  between LiDAR and IMU injects a consistent bias into every deskew and every fused pose, and no
  amount of filtering removes a bias.
- **How it is done:** target-based (a checkerboard seen by both a camera and a LiDAR; a corner
  reflector), motion-based / hand-eye (`AX = XB` from trajectories observed by both sensors —
  needs rotation about at least two axes to be fully constrained), or **online estimation** — put
  the extrinsics in the state vector and let the estimator refine them (VINS-Mono does this for
  camera↔IMU, including the time offset). Kalibr is the standard camera↔IMU toolbox.
- **Observability matters:** pure translation cannot identify a rotational offset; a robot driving
  in a straight line cannot calibrate itself. This is why calibration procedures ask for a wiggly
  trajectory.
- **Interviewer's target sentence:** "Extrinsic error is a systematic bias, not noise — a degree
  of LiDAR-to-IMU misalignment biases every deskew — so you either calibrate with a target and
  hand-eye motion or estimate it online, and you make sure the motion actually excites the axes
  you're trying to observe."
- **`piros2` line:** the repo has extrinsics as a **hand-declared static TF chain** —
  `base_link → camera_link → camera_optical_frame`, with a placeholder mount pose (5 cm up) and
  the canonical −90/0/−90 optical rotation, verified across the LAN with `tf2_echo`. They were
  *declared*, not measured, and the file says so. `se3.py`'s `BASE_FROM_OPTICAL` is that same
  transform in code, used to conjugate rotations between optical and base axes.

## 12. Time synchronisation, hardware triggering, PTP/NTP

- **Why it dominates:** at 1 m/s, 10 ms of clock skew is 1 cm of position error; at 90°/s of yaw,
  10 ms is 0.9°. Fusing two sensors with a constant unknown offset produces a *consistent* error
  that looks like miscalibration. Time is the second half of the frames-and-time problem.
- **The ladder, best to worst:** (1) **hardware triggering / a shared clock** — one signal fires
  the cameras and stamps the IMU, or the LiDAR's PPS disciplines everything (GNSS PPS + NMEA is
  the classic); (2) **PTP (IEEE 1588)** — sub-microsecond over Ethernet with hardware timestamping
  in the NIC/switch; (3) **NTP/chrony** — milliseconds over a network, fine for logging, not for
  fusion; (4) **software timestamps at receipt** — you are now measuring your own scheduler and USB
  stack, not the sensor.
- **Estimating the offset** is the pragmatic fallback: model a constant time offset `t_d` as a state
  and let the optimiser find it (VINS-Mono does; so do several LiDAR-inertial systems).
- **Trap to name:** a *plausible* timestamp is worse than a missing one. If a driver stamps at
  receipt rather than at exposure, the number looks fine and is systematically late by the
  transport latency.
- **Interviewer's target sentence:** "Hardware trigger or PPS if you can, PTP if you're on
  Ethernet, NTP only for logs — and if you're stuck with software stamps, model the offset as a
  state rather than pretending it's zero."
- **`piros2` line:** the repo's headline sensor bug is exactly this. `/image_raw` header stamps
  **lag wall clock by a steady ~0.73 s** — a UVC/driver timestamping fault, with the frames
  themselves live. It was discovered because a stamp-age freshness gate silently dropped **100 %**
  of frames. The standing rules that came out of it: never gate freshness or report latency
  against `header.stamp` on this camera; measure processing cost against one process's own clock;
  the dashboard's rate and STALE lines are computed on its *receipt* clock. And the constructive
  fix on the fusion side: the depth node republishes the exact frame it inferred on as
  `/depth/rgb` with **stamps identical to `/depth`**, so `message_filters` exact sync pairs every
  depth frame instead of relying on two independently stamped streams agreeing.

## 13. Encoders and wheel odometry

- **Encoders** count shaft rotation — incremental (quadrature A/B channels giving direction, plus
  an index pulse) or absolute (a coded disc, position known at power-on). Resolution is counts per
  revolution times the gear ratio; velocity comes from counting edges in a window (good at speed)
  or timing between edges (good at low speed).
- **Wheel odometry** integrates the kinematic model (differential drive, Ackermann) to a pose. Its
  error sources are systematic before they are random: wheel radius error, wheelbase error, and
  above all **slip** — which is unbounded and unmodelled. On a mine floor, in mud, or on a legged
  robot, wheel odometry is close to useless; on a clean warehouse floor it is excellent and
  effectively free.
- Its real value in a fusion stack is as a **prior with a good short-term velocity estimate and a
  strong non-holonomic constraint** (a car cannot move sideways) — which helps enormously in
  exactly the degenerate cases where LiDAR fails (§17 of [02_SLAM.md](02_SLAM.md)). UMBmark is the
  classic systematic-error calibration procedure.
- **Interviewer's target sentence:** "Wheel odometry is cheap, high-rate and non-holonomically
  constrained, which makes it a great prior — but slip is unbounded and unobservable from the
  encoders alone, so it's an input to a filter, never a truth source."
- **`piros2` line:** not touched — the platform is a static camera on a tripod-and-hands, with no
  wheels and no encoders. The repo's honest position is that it has *no* proprioceptive sensor at
  all, which is precisely why its orientation estimate has nothing to fall back on when features
  vanish (the blackout case, §15).

## 14. Barometers, altimeters, rangefinders

- **Barometer:** pressure → altitude via the atmosphere model, ~10 cm resolution but drifting with
  weather, and badly disturbed by a drone's own propwash and by opening a door indoors. Good for
  *relative* altitude over minutes, useless for absolute height, and it is the standard fallback
  for a drone's Z axis when GNSS is gone. Underground it is subject to ventilation pressure changes.
- **Downward rangefinders:** ultrasonic (cheap, cone-shaped, confused by soft surfaces), infrared
  ToF (VL53L1X-class, cm-accurate to a few metres, sunlight-limited), laser altimeters (tens of
  metres), and radar (works through dust — a real advantage underground). These give height
  *above the surface below you*, which is not the same as altitude: fly over a table and it steps.
- **Optical flow sensors** (PX4Flow, PMW3901) pair with a rangefinder to give horizontal velocity —
  the classic GPS-denied hover aid — and need texture and light, both absent in a dark drift.
- **The composite point:** a drone's altitude estimate in a GPS-denied space is a *fusion* of baro,
  rangefinder and whatever the SLAM says, each with a different failure mode, and the interviewer
  is checking that you know none of them is authoritative.
- **Interviewer's target sentence:** "Baro drifts and is disturbed by the vehicle itself,
  rangefinders measure height above whatever is below rather than altitude, and optical flow needs
  light and texture — so altitude in a GPS-denied environment is a fused estimate with an explicit
  failure story for each input."
- **`piros2` line:** not touched — no barometer, no rangefinder. The one genuinely comparable act
  in the repo is using a **tape measure as the reference instrument** for the depth scale check:
  the cheapest possible absolute rangefinder, used deliberately because everything else in the
  pipeline was relative.

## 15. Sensor noise models and failure modes

- **Noise models** are what let you fuse rather than average: a measurement without a covariance is
  a number without weight. The standard vocabulary — zero-mean Gaussian white noise (the filter's
  `R`), bias/random walk (state, not noise), scale factor and non-linearity (systematic),
  quantisation, and outliers (which are *not* Gaussian, hence robust kernels — Huber, Cauchy — and
  RANSAC).
- **The distinction that matters most:** *bias vs noise*. Averaging kills noise and does nothing to
  bias. Most "we filtered it and it's still wrong" bugs are a bias being treated as noise.
- **Failure modes to enumerate for any sensor:** saturation (out of range, clipped), dropout (no
  data — is the consumer's timeout shorter than the failure?), stale-but-plausible data (the worst:
  a frozen last value that looks live), degradation (dust, rain, fog, sun), interference (two
  LiDARs, two ultrasonics), and *silent mode changes* (auto-exposure, auto-gain, a driver falling
  back to a slower path).
- **The engineering rule:** design so failures are **loud**. A sensor that stops publishing is
  detectable; a sensor that publishes the same value forever is not, unless someone wrote the
  check. Watchdogs, freshness counters, and "did the value change?" tests are cheap.
- **Interviewer's target sentence:** "I enumerate failures per sensor and ask which of them are
  silent — saturation, dropout, stale-but-plausible, degradation and interference — and then I make
  the silent ones loud, because a filter can't fix a failure it can't see."
- **`piros2` line:** this is the repo's explicit doctrine and there are receipts.
  **Camera consumers fail loudly**: `camera.launch.py` pre-flight-checks the device *and names the
  PID holding it* (a leaked usb_cam had fed a whole session unnoticed) because usb_cam logs one
  ERROR on a missing device and then idles forever; the camera node carries `on_exit=Shutdown()`;
  recipes verify a stream exists before recording. The **silent** failures found and fixed:
  the 0.73 s stamp fault (a freshness gate that dropped 100 % of frames without saying so),
  the ONNX **CUDA→CPU silent fallback** (now the winning provider is logged), and — the one caught
  by a scripted gate — a covered lens yielding no descriptors, where `could_estimate` stayed False
  and the tracking-loss counter never moved; the detector now counts a nothing-to-match frame as
  lost once tracking has ever succeeded, with two tests pinning it and a black-fill gate bag that
  reproduces the 19.7° failure when the flag is toggled off.

## What to say if asked "what sensors have you worked with?"

"One, properly: a USB webcam — and I know it in unusual detail because it lied to me. Its header
stamps are 0.73 seconds behind wall clock while the frames are live, so I measure rates on the
receiving node's own clock; its V4L2 controls persist inside the device across reboots, so a
benchmark's leftover exposure blacks out the next session; its real frame rate is 42–60, not the
30 everyone quotes, and only under a known-good control baseline. I ran approximate intrinsics
derived from the FOV spec rather than block on a checkerboard, and I pinned the monocular depth
scale with a tape measure and then measured the residual per-frame wobble at ±4 %. I have not
worked with LiDAR or an IMU — I can talk about deskewing, Allan variance and preintegration as
someone who has read the papers, and that is the boundary."
