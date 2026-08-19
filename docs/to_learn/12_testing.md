# Testing — the study file for section 12 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at hold-a-conversation
depth, the sentence an interviewer is fishing for, and an honest **`piros2` line**. The
syllabus gives section 12 no priority note — it is breadth, but breadth the Emesent ad
names ("test harnesses" in Python, "CI/regression"). Per the honest-claim rule: reading ≠
holding; the `piros2` lines say what has been built, and here the repo has real weight: a
suite at **199 tests on 2026-08-18** needing no hardware, model weights or DDS (CLAUDE.md),
grown overnight by the SLAM plan (the fork alone reports 123; `def test_` under
`src/*/test/` counts 221 today), plus a written doctrine — "gates are closed by scripts,
not eyes" ([docs/info/verification.md](../info/verification.md)) — exercised by `just
gate`, `just snap`, `just run-bag`. Every file named below was checked against the tree;
every number is from the repo's own docs.

## Mental model to carry through the whole file

```
   cheap · fast · many · deterministic  ──────────────►  expensive · slow · few · noisy
   pure fn │ unit (node in-process, │ integration │ system (replay │ HIL (real   │ field trial
   tests   │ fakes for pubs/TF)     │ (real DDS)  │ of a real bag) │ compute)    │ (survey truth)
   piros2: se3, pose_graph, │ FakeSession,     │ none (launch_ │ run-bag,      │ —     │ tape measure,
           mesh_fill        │ CapturingPub,    │ testing off)  │ gate-*        │       │ sweeps recorded
                            │ fake /proc       │               │               │       │ once
```

Push every question as far *left* as it honestly goes; when it must sit right, make the
check **name its evidence** — a number, a threshold, a log line, a bag — so a script can
close it and a person is only needed for the physical world.

## 1. Unit, integration, system and regression levels

- **Unit:** one function or one class, in-process, all collaborators replaced or trivial;
  runs in milliseconds; the failure names the line. In ROS 2 the trap is thinking a *node*
  can't be unit-tested because it needs the graph — it can, if the callback is a method and
  the publishers are attributes you can swap.
- **Integration:** two or more real components talking through the real mechanism (DDS,
  a service, a file) — a launch file with a talker and a listener, `launch_testing`'s
  `ReadyToTest` and post-shutdown asserts. Slower, and now discovery, QoS and timing enter.
- **System (end-to-end):** the whole pipeline on realistic input, checked against an
  external truth or its own earlier output. In robotics the honest system test is a
  **bag replay**: same bytes every run, no hardware, the pipeline decides what it decides.
- **Regression:** any of the above kept *because a bug once lived there*. A regression
  test is a pinned reproduction; the good ones say in their name what broke.
- Rule of thumb: the pyramid — many unit, fewer integration, few system — because cost
  and flakiness rise to the right and a right-hand failure says *that*, not *where*.
  Classic mistake: only end-to-end tests "because that's what matters", then a one-hour
  suite that fails for reasons unrelated to the change.
- **Interviewer's target sentence:** "Unit tests localise, integration tests prove the
  wiring, system tests prove the behaviour on real data, and regression tests are the ones
  that pin a bug in place — the mix should be shaped like a pyramid so the fast tier catches
  most things and the slow tier stays small enough to keep green."
- **`piros2` line:** unit: `test_se3.py`, `test_pose_graph.py`, `test_mesh_fill.py`,
  `test_depth_align.py`, `test_keyframe_store.py` — pure functions and small classes;
  node-in-process unit: `test_depth_estimator.py`, `test_cloud_projector.py`,
  `test_keypoint_detector.py`, `test_edge_detector.py` (callback called directly,
  publishers captured — no DDS); system: `just run-bag` and the `gate-*` recipes replay real
  bags through the whole `world_mesh` launch; regression:
  `test_blackout_after_tracking_counts_as_lost` pins the "covered lens isn't a loss" bug the
  black-fill gate bag found. **No launch-level integration tests** — `pytest.ini` turns the
  `launch_testing` plugin off "until actual launch tests exist"; that tier is the gap.

## 2. Test doubles: mocks, fakes, stubs

- Meszaros's vocabulary, used loosely but expected: **stub** — canned answers, no logic,
  not asserted on ("the TF buffer says this transform"); **fake** — a working, simplified
  implementation (in-memory DB, a session that "runs" a model by returning a fixed array);
  **mock** — records calls so you *assert on the interaction* ("publish was called once
  with stamp X"); plus **spy** (real object, calls recorded) and **dummy** (passed, unused).
- The design point behind all of them: **inject the collaborator**. A node that constructs
  its own `onnxruntime.InferenceSession` in `__init__` cannot be tested without the
  weights; one that accepts `session=` can. Same for the clock, the filesystem root, TF.
- Classic mistake: over-mocking — asserting on every internal call so the test breaks on
  any refactor while proving nothing about outputs. Prefer fakes, assert on the result.
- **Interviewer's target sentence:** "A stub answers, a fake works, a mock verifies. I
  reach for fakes and capture-and-assert on outputs, and I design the code so the
  expensive collaborator — the model, the device, the clock — is injected."
- **`piros2` line:** all three shapes are in the tree, named honestly in their docstrings.
  **Fake:** `FakeSession` in `test_depth_estimator.py` "stands in for onnxruntime; returns
  a fixed vertical gradient" — it implements `get_inputs()` and `run()`, records
  `last_feed`, and the estimator takes it via `DepthEstimator(session=FakeSession())`, so
  "100 MB of weights (plus the venv that loads them) stay out of the test environment";
  what remains under test is decode, ImageNet normalisation, inversion, resize, headers,
  preview. **Stub:** `FakeBuffer` in `test_cloud_projector.py` returns one transform (or
  `None`) for the odom-frame output path — no tf2 listener, no `/tf` traffic. **Spy /
  mock-ish:** `CapturingPublisher` (a list that `publish()` appends to) and
  `CapturingBroadcaster` for TF, so tests assert on the message that would have gone out —
  encoding, stamp identity between `/depth` and `/depth/rgb`, header frame. The pattern is
  copied package to package: it started in `piros2_vision`'s edge-detector test.

## 3. GoogleTest, GMock

- **GoogleTest** is the C++ unit framework ROS 2 itself uses: `TEST(Suite, Name)`;
  `TEST_F` with a fixture deriving `::testing::Test` (`SetUp()`/`TearDown()`); `TEST_P` +
  `INSTANTIATE_TEST_SUITE_P` for parametrised cases; `EXPECT_*` continues on failure,
  `ASSERT_*` returns (use it when continuing would dereference garbage);
  `EXPECT_NEAR(a, b, tol)`/`EXPECT_DOUBLE_EQ` for floats — never `EXPECT_EQ` on doubles;
  `--gtest_filter`, `--gtest_repeat`, `--gtest_shuffle` for flake hunting.
- **GMock:** `MOCK_METHOD(ReturnType, Name, (Args...), (override))` inside a class deriving
  the interface (so the code under test must depend on an abstract interface — the design
  cost of mocking in C++); `EXPECT_CALL(mock, Name(_)).Times(1).WillOnce(Return(x))`,
  matchers (`_`, `Eq`, `Gt`, `Field`, `Pointee`), `NiceMock`/`StrictMock` for uninteresting
  calls, `InSequence`. Uninteresting-call warnings are the usual noise.
- **In ROS 2 (Jazzy):** `ament_cmake_gtest`/`ament_cmake_gmock` — in `CMakeLists.txt`,
  `if(BUILD_TESTING) find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_x test/test_x.cpp) ament_target_dependencies(...) endif()`; run via
  `colcon test --packages-select pkg` and read with `colcon test-result --verbose`.
  For rclcpp nodes the fixture typically does `rclcpp::init` once, spins with a
  `SingleThreadedExecutor` in `spin_some()` loops with a deadline, and the node under
  test is a component you can construct with `NodeOptions`.
- **Interviewer's target sentence:** "GTest for the assertions and fixtures, GMock when a
  dependency is behind an interface; `EXPECT_NEAR` for anything floating; ament wires it
  into `colcon test` under `BUILD_TESTING`."
- **`piros2` line:** **not touched** — every package is `ament_python`, no `CMakeLists.txt`
  in `src/`, so no GTest; that matches the syllabus's C++ boundary. The nearest thing is
  the shape: the pytest fixtures do what a `TEST_F` fixture does (`rclpy.init()` →
  construct → swap publishers → yield → `destroy_node()` → `rclpy.shutdown()`).

## 4. pytest, fixtures, parametrisation

- **Fixtures** (`@pytest.fixture`) are dependency injection by argument name; `yield`
  gives teardown; `scope=` (function/module/session) trades isolation for speed;
  `conftest.py` shares them; built-ins worth naming: `tmp_path`, `monkeypatch`, `capsys`,
  `caplog`. **Parametrisation:** `@pytest.mark.parametrize('phi', [...])` turns one body
  into N cases with N ids; `pytest.approx` for float equality; `pytest.mark.skipif` for
  environment-dependent cases; `-x`, `-k expr`, `--lf` (last-failed), `-p no:plugin`,
  `--import-mode=importlib`.
- **In ROS 2 (Jazzy):** an `ament_python` package declares `python3-pytest` and the
  `ament_*` linters as `<test_depend>`; `colcon test` runs each package's `test/` in its own
  process from the package root and `colcon test-result --verbose` aggregates the JUnit
  XML (`ament_cmake_pytest` for CMake packages). Two common mistakes: teardown before
  `yield`, and cross-package name collisions when one pytest process collects every
  package (two `test_flake8.py` modules → import error).
- **Interviewer's target sentence:** "Fixtures inject and tear down, parametrise for the
  table of cases, `approx` for floats, and keep the collection layout such that both the
  build tool and the IDE can run the same tests."
- **`piros2` line:** the repo's testing language. `just test` = `colcon test && colcon
  test-result --verbose`; the VSCode Testing sidebar runs the *whole* tree in one pytest,
  which needed three accommodations, all present and documented in-file: `pytest.ini`
  (`--import-mode=importlib -p no:launch_testing -p no:launch_ros` — "the default import
  mode cannot hold two modules named test_flake8 at once"), `.vscode/ros.env` (`PYTHONPATH`,
  `LD_LIBRARY_PATH`, `AMENT_PREFIX_PATH` for `/opt/ros/jazzy`, because "the Testing sidebar
  spawns pytest WITHOUT a login shell"), and linter tests anchored on `__file__`
  (`PACKAGE_DIR = str(Path(__file__).resolve().parents[1])`) instead of `ros2 pkg create`'s
  CWD-dependent `argv=[]`. Fixtures are yield-teardown; `tmp_path` hosts the fake `/proc`
  and the `.npz`/`.g2o` round-trips; `parametrize` drives the SO(3)/SE(3) exp–log round
  trips in `test_pose_graph.py`; `pytest.approx` in the projector and PLY tests; `skipif(G2O
  is None, reason='g2o binary not installed')` guards the g2o oracle test.

## 5. Testing without hardware: fake drivers, synthetic data

- Three levers, cheapest first: (1) **synthetic inputs with a known answer** — a white
  rectangle on black has edges exactly where drawn; a plane at 2 m projects back to 2 m; a
  seeded ray bundle rotated by a known R must give R back; (2) **injected doubles for the
  hardware-facing object** — model session, device file, `/proc`, TF buffer, clock;
  (3) **recorded data** — a bag captured once is a deterministic sensor forever ("bag
  everything" is hygiene, not just debugging).
- Design consequence: the hardware boundary must be thin and injectable — opening
  `/dev/video0` in a constructor or reading `/proc` at a hard-coded path is untestable by
  construction. Classic mistakes: synthetic data that is *too* friendly (a chessboard is
  perfect for detection and useless for matching — every corner is a lookalike and the
  cross-check throws lookalikes away); unseeded randomness; a fake driver so elaborate it
  needs its own tests.
- **Interviewer's target sentence:** "Build inputs whose right answer you know, inject the
  hardware-facing collaborator, and treat a recorded bag as the deterministic version of the
  sensor — the code has to be shaped so the boundary is thin enough to swap."
- **`piros2` line:** the suite's premise — CLAUDE.md: "none need hardware or model
  weights". `FakeSession` for ONNX (above); `_fake_proc(tmp_path, pid, fd_targets,
  cmdline)` in `test_camera_launch.py` builds `/proc/<pid>/fd/*` symlinks and a `cmdline`
  file so `_device_holders(device, proc=tmp_path)` — the pre-flight that names the PID
  holding `/dev/video0` — is tested on a fake `/proc`, including the by-id symlink case and
  "own process is not a holder"; **synthetic chessboards** for ORB *detection* and
  **seeded greyscale noise** (`make_textured_frame(seed, tweak)`, "every patch unique, by
  construction", `tweak` nudging one pixel past the CRC dup-skip) for *matching*;
  `unit_rays(n, seed)` for the Kabsch estimator, with planted false matches and a coplanar
  reflection guard; synthetic depth planes for the projector ("flat wall comes back flat at
  two metres"); `synthetic_view()` for relocalisation; punched-plane / pinched-hole /
  debris fixtures for `mesh_fill.py`; a drifted 24-node circle for the pose graph. And the
  bags — `static1`, `sweep3`, the gate bags — recorded once, replayed by `just run-bag` and
  every `gate-*` recipe.

## 6. Property-based testing

- Instead of hand-picked examples, state a **property** that must hold for *all* inputs
  and let a generator search for a counterexample, then **shrink** it to the smallest
  failing case. Python: **Hypothesis** (`@given(st.floats(...))`, strategies, `assume`,
  `@settings(max_examples=…)`, the example database that replays past failures). C++:
  RapidCheck; also `--gtest_shuffle`/`--gtest_repeat` as poor cousins.
- Where it shines in robotics maths: round-trips (`log(exp(ξ)) == ξ` for small ξ,
  quaternion ↔ matrix ↔ quaternion up to sign), group axioms (`R·Rᵀ = I`, `det R = 1`,
  `inv(T)·T = I`), invariance (a transform applied to points then inverted returns the
  points), monotonic or bounded outputs (a filter's output within input range),
  idempotence (`complete(complete(m)) == complete(m)`), serialisation round-trips.
- Rules of thumb: properties near singularities need explicit strategies (angles near π,
  near-zero rotations, degenerate/coplanar point sets) or the generator never finds them;
  keep generated tests deterministic in CI (Hypothesis derandomises there); a property
  test that only ever asserts "no exception" is weak.
- Classic mistake: writing the oracle by re-implementing the code under test.
- **Interviewer's target sentence:** "Property tests state the invariant — round-trip,
  group law, idempotence — and let a generator hunt for the counterexample and shrink it;
  they earn their keep on Lie-group and serialisation code where the space of inputs is
  large and the singularities are known."
- **`piros2` line:** **no Hypothesis** (grep finds none) — but the suite is full of
  hand-rolled properties on seeded generators, the same idea without the shrinker:
  `test_so3_exp_log_round_trip`/`test_se3_exp_log_round_trip` (parametrised),
  `test_optimised_poses_stay_on_se3`, `test_quaternion_round_trip_all_branches` (all four
  branches of matrix→quaternion), `test_completion_is_idempotent` in `test_mesh_fill.py`,
  `test_ply_coordinates_round_trip`, save/load round trips for the keyframe store and the
  pose graph's `.g2o`. Adding Hypothesis to `test_se3.py` is the honest "next step".

## 7. Golden-file and snapshot testing for pipelines

- Run the pipeline on a fixed input, store the output ("golden"), and diff future runs
  against it. Snapshot frameworks (`pytest-regressions`, `syrupy`; Jest snapshots on the
  web side) automate the store/compare/regenerate cycle. Strengths: catches *any* change
  cheaply, ideal for serialisation, rendered images, config dumps. Weaknesses: it tests
  *sameness*, not *correctness* — a wrong golden is a locked-in bug; floating point and
  non-determinism make byte-equality brittle (use tolerances, or compare a *metric* — ATE,
  triangle count, plane residual — instead of the bytes); regeneration discipline matters
  ("I updated the golden" needs a reviewer to look at the diff, not just re-bless).
- In robotics the golden is usually **the pipeline's own earlier output on the same bag** —
  compare a trajectory to last week's trajectory (evo-style ATE/RPE), a mesh to last week's
  mesh (surface gap), a keypoint count to a range — and only for a few *reference* datasets
  with published truth (TUM RGB-D, KITTI, EuRoC) do you compare against ground truth.
- **Interviewer's target sentence:** "Golden tests are cheap change-detectors — I use them
  on serialisation and rendered outputs, and for pipelines I compare a metric with a
  tolerance against the pipeline's own previous run on a fixed bag, plus ground-truth
  datasets where they exist."
- **`piros2` line:** `verification.md`'s doctrine is exactly this — "compare against the
  pipeline's *own* earlier output rather than against a truth nobody has". `gate_check.py`:
  a gate bag replays window A, a disturbance, then A′ (part of A again); "whatever pose the
  pipeline reported for a source frame during A is the reference for that same frame
  during A′", so the verdict is median/p90 rotation (and translation in rgbd mode) over
  A′'s tail under `gate.json` thresholds *plus* the expected log lines (`tracking lost` →
  `relocalized against keyframe` → `snapping odometry`), written as `report.json`,
  `poses.csv`, `poses.png`. Measured 2026-08-18: flick PASS (65.3° correction, tail
  0.48°), occlude PASS (18.4° snap, tail 0.95°/3 cm). The SLAM gates add a **yardstick**
  (the same replay through RTAB-Map) and a **ground truth** (`gate-tum` on TUM fr1/desk:
  ATE RMSE 0.089 m own backend, 0.096 m RTAB-Map, 0.163 m raw odometry); `gate-mesh` scores
  a paired OUT/BACK surface gap (median 5.7 cm corrected vs 7.8 cm). Picture-goldens:
  `just snap` dumps every image topic and rviz/rqt window to files, `just mesh-views`
  renders a PLY from fixed viewpoints — "the RViz window is a viewer, not the evidence". No
  byte-diff snapshot library, deliberately: the outputs are metrics with tolerances.

## 8. Hardware-in-the-loop testing

- The X-in-the-loop ladder: **MIL** (model in the loop — controller model against plant
  model), **SIL** (compiled controller software against a simulated plant, e.g. PX4 SITL,
  Gazebo/Isaac), **PIL** (the software on the *target processor*, plant still simulated —
  catches timing, word size, compiler differences), **HIL** (the real embedded computer,
  real buses/IO, a real-time simulator feeding sensor signals and consuming actuator
  commands — PX4 HITL, dSPACE/Speedgoat rigs in automotive/aero). Each rung buys realism
  in one dimension — timing, IO, drivers — at the price of setup cost and reproducibility.
- What HIL uniquely catches: driver and bus behaviour (UVC quirks, CAN timing, serial
  framing), real CPU load and latency, thermal throttling, boot/update paths, watchdogs —
  none of which a laptop container shows. What it does *not* replace: the physical world
  (lighting, dust, rotor wash on a LiDAR). For a Hovermap-style product the interesting
  HIL is Cortex on its onboard computer, the flight controller (DJI OSDK) real or emulated,
  replayed LiDAR + IMU, loop closed — "does the velocity command go out on time when SLAM
  slips?" is a HIL question.
- **Interviewer's target sentence:** "HIL puts the real compute and real interfaces in the
  loop with a simulated plant, so you test timing, drivers and failure paths that
  simulation-only misses; it sits between SIL and the field, and it should be automated
  and gated on the same numbers as the field would be."
- **`piros2` line:** structural rather than a rig: the **Pi is the real embedded computer
  running the real driver** (`usb_cam` on `/dev/video0`), the dev box the compute; the
  Wi-Fi watchdog's escalation ladder was drilled on real hardware — the drill "reproduced
  incident 1's `status_code=16` AP rejection and recovered unaided at T+426 s" — which is
  HIL-shaped verification of a recovery path. No simulated plant, no automated rig; the
  hardware findings (0.73 s stamp lag, `exposure_dynamic_framerate` stealing fps,
  BEST_EFFORT dropping 2.7 MB frames) came from running on hardware and were then pinned
  as rules and, where possible, tests.

## 9. Field trial methodology and validation criteria

- A field trial is an experiment: **acceptance criteria written before** the flight
  ("loop-closure error < X cm over Y m; no SLAM-slip abort in Z minutes of corridor"), a
  **reference** better than the sensor (total-station targets, a prior scan, RTK where GNSS
  exists, tape measure at minimum), **repeats** (a single pass measures the weather),
  **everything bagged** (raw sensors + TF + params + git hash, so the software can change
  and be re-scored offline), a pre-flight checklist, and a debrief that turns each surprise
  into a bag-backed regression.
- Validation criteria are *metrics with thresholds*: ATE/RPE against truth, drift as
  % of distance, target error (mean/σ), coverage %, processing time per mission minute,
  operator-facing events (aborts, RTH). Wildcat's paper is the template — 63 surveyed
  targets at QCAT, mean absolute error per target, two other systems on the same data;
  Emesent's ST-X accuracy white paper and spec (±15/±10/±5 mm; ±0.03 % drift) are the
  productised version. Classic mistakes: no baseline, truth no better than the sensor
  under test, tuning on the trial data, trials that produce opinions instead of bags.
- **Interviewer's target sentence:** "Write the pass criteria first, bring a reference
  better than the sensor, repeat, bag everything so the trial becomes a regression
  dataset, and score with the same metrics the customer will — target error, drift per
  distance, coverage."
- **`piros2` line:** small-scale but real: the **tape-measure scale check** pinned
  `depth_scale: 2.69` (a 2.50 m wall read 9.30 m at scale 10; re-export verification
  +0.1 %); the ±4 % per-frame depth-scale wobble was *measured on a static scene* before
  the aligner was designed; the "lit-sweep experiment" fused the same 44 s capture under two
  pose files and got two different, named failure signatures. And the discipline in
  `verification.md`: "'Needs a human' is reserved for the physical world … say what one
  recording would turn it into" — one human recording session (`just record 45 sweep3`)
  converted the hand-sweep, flick and occlude gates into replayable `just gate …` runs.

## 10. Coverage, and its limits

- **Line/statement coverage:** fraction of lines executed. **Branch coverage:** both arms
  of every conditional. **MC/DC** (modified condition/decision): every boolean sub-condition
  shown to independently affect the outcome — DO-178C Level A / ISO 26262 ASIL D territory,
  relevant if the drone stack ever chases certification. Tools: `gcov`/`lcov`/`gcovr` for
  C++ (`--coverage` flag, `-O0`), `coverage.py`/`pytest-cov` for Python
  (`--cov=pkg --cov-branch --cov-report=term-missing`); `colcon` has a `coverage-pytest`
  mixin and `ament_cmake` can wire `lcov`.
- **The limits:** coverage measures *execution*, not *assertion* — a test that calls the
  function and asserts nothing covers 100 % of it; it says nothing about untested *inputs*
  (the singularity you never generated), timing, or integration. **Mutation testing**
  (`mutmut`, `cosmic-ray`; Mull for C++) is the honest complement: mutate the code, see
  whether a test dies. Rule of thumb: coverage is a tool for *finding untested code*, not
  a score; 70–85 % of the logic-bearing code with real assertions beats 100 % of glue.
- **Interviewer's target sentence:** "I use coverage to find what isn't tested, not to
  score the suite — it can't see missing assertions or missing inputs; mutation testing
  and property tests are how you find those."
- **`piros2` line:** no coverage is measured or gated (no `pytest-cov`, no `lcov`). What
  stands in: tests written from the behaviour contract —
  `test_completion_parameters_declare_the_contract` pins the mesher's completion knobs, the
  dashboard's "stale feed gets flagged on receipt time only" pins the 0.73 s stamp rule —
  and linter tests (`flake8`, `pep257`, `copyright`) in every package. Adding `--cov` would
  be trivial; that it hasn't been done is a choice worth stating.

## 11. Flaky tests and non-determinism in robotics tests

- **Sources, roughly by frequency in ROS 2:** discovery timing (a subscriber created after
  the first message; a client that doesn't `wait_for_service`); executor threading; wall
  clock vs `use_sim_time`; drops under BEST_EFFORT or tiny history depth; unseeded
  randomness (RANSAC — an estimator with unseeded RANSAC passes 99 % of the time);
  floating-point order dependence (parallel reductions, GPU kernels — cuDNN picks
  algorithms non-deterministically unless told not to); unordered containers iterated for
  output; CPU load changing which of two racing messages arrives first; leaked processes
  from the previous test holding a device or port.
- **Mitigations:** seed everything and *pass the seed in*; test the pure maths without an
  executor at all; when a graph is needed use `ReadyToTest`, `wait_for_service`,
  `wait_for_message`-style barriers with deadlines, never `sleep(2)`; RELIABLE + adequate
  depth in tests; run under `use_sim_time` with the bag's clock; make output ordering
  explicit; quarantine and *fix* flakes rather than retrying them (`pytest-rerunfailures`
  and `--gtest_repeat` are diagnostic tools, not a policy — a retry loop is a flake with a
  hat on); teardown that kills what setup started.
- The subtler point interviewers like: **the pipeline itself is non-deterministic under
  load** — dropped frames change which frames pair, which changes the estimate — so a
  system-level gate must be written with a *tolerance band* and its run-to-run spread
  measured, not asserted equal to a golden.
- **Interviewer's target sentence:** "Flakiness in ROS tests is almost always discovery,
  timing or unseeded randomness; I fix it by testing the maths without an executor, using
  barriers instead of sleeps, seeding, and giving system-level checks a measured tolerance
  — and I treat a retry as a symptom, never a fix."
- **`piros2` line:** (1) the unit tier avoids the problem structurally — no DDS,
  callbacks called directly, publishers captured, every random input seeded
  (`default_rng(seed)`, `unit_rays(seed=…)`, `make_textured_frame(seed=…)`). (2) The
  system tier *measured* its non-determinism: `verification.md` records raw-odometry loop
  gap "6.1 cm / 1.9° (run-to-run 3.8–14.6 cm / 1.4–19° — the pipeline is not deterministic
  under load)", so the gates are thresholds with `--settle`/`--pair-tolerance`/`--timeout`,
  not equalities, and warm-up is explicit (`just gate` waits for the estimator's "inference
  provider" log line; `run-bag` sleeps 12 s — "frames played before that are simply
  lost"). (3) Teardown is a contract: every session/gate recipe carries an EXIT trap that
  `pkill -f`s each node pattern (because `kill %N` orphaned nodes twice on 2026-07-27),
  `just stragglers` sweeps both machines, and CLAUDE.md records "rviz2 sometimes needs two
  SIGTERMs". A pairing flake found and pinned: `message_filters` exact sync at the default
  queue of 5 made RTAB-Map pairing "a coin toss — 0–6 vs a deterministic 24 odometry
  updates on the same replay"; the fix was 30-deep queues.

## What to say if asked "how do you test robotics software?"

"Push everything into pure functions and node-in-process tests with injected doubles — in
`piros2` a fake ONNX session, captured publishers, a fake `/proc` tree, seeded synthetic
frames and ray bundles; ~200 tests needing no hardware, weights or DDS. Above that, the
system test is a bag replay with a scripted verdict: a gate bag re-orders views we already
recorded, the checker compares the return-pass pose to the pipeline's own first-pass pose
under a threshold, checks the log says *why*, and writes the plot; TUM fr1/desk for ground
truth, RTAB-Map as a yardstick. I haven't written GoogleTest or launch_testing integration
tests and I don't measure coverage — those are the gaps." Then stop.
