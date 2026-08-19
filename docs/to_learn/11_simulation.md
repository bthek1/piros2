# Simulation — the study file for section 11 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist bullet: the concept, the sentence an
interviewer is fishing for, and an honest **`piros2` line**.

This is a breadth section with an unusual twist: **`piros2` has no simulator at all, and
that is a defensible position rather than a gap to apologise for** — because the repo
solved the same problem a different way. Its last bullet, "record and replay as an
alternative to simulation", is the one item here where the repo is genuinely strong and
has receipts: a whole verification layer
([docs/info/verification.md](../info/verification.md)) that replays real bags through the
real stack headlessly and exits 0/1. Everything above that bullet is reading. Reading a
section does not tick its box.

Emesent relevance: their current engineering ad lists a **simulation pipeline** with
Gazebo and Isaac Sim as nice-to-haves, and a company flying autonomous drones into places
where a crash is expensive and a re-test costs a mine shift has an obvious appetite for
both simulation *and* replay. Knowing which question each answers is the point.

## Mental model: three ways to test a robot without the robot

| | What is real | What is fake | Answers |
| --- | --- | --- | --- |
| **Replay (bags)** | sensor data, all your code, the messaging layer | the world (it can't react to you) | "does my perception/estimation still produce the same answer on data we trust?" |
| **Simulation** | your code, a physics/sensor model that *reacts* | sensors, physics, appearance | "does my control/planning/autonomy behave when the world responds?" |
| **HIL / field** | everything | nothing | "does it actually work?" |

The decisive difference: **a bag cannot close the loop.** If your code decides to turn
left, the recorded world does not turn. So anything that only *observes* — perception,
depth, SLAM front-ends, mapping, meshing — can be tested to a very high standard on bags,
while anything that *acts* — planners, controllers, autonomy state machines, failsafes —
needs a simulator or the real thing. Say that sentence and you have placed every tool
correctly.

## 1. Gazebo / Ignition / gz-sim

- **The naming mess, which interviewers use as a shibboleth:** "Gazebo Classic" (the
  original, `gazebo11`, **end-of-life January 2025**) → rewritten as "Ignition Gazebo"
  (Citadel, Edifice, Fortress) → renamed back to "Gazebo" with the binaries and libraries
  prefixed **`gz-`** (`gz sim`, `gz topic`, `gz service`) after a trademark dispute. Modern
  releases are named alphabetically: Fortress, Garden, Harmonic, Ionic. **Harmonic is the
  LTS pairing for ROS 2 Jazzy**, bridged by `ros_gz` (`ros_gz_bridge`, `ros_gz_sim`).
  Getting this right signals you have used it since 2023; saying "gazebo_ros_pkgs" and
  `gazebo11` signals a tutorial from 2019.
- **Architecture:** an entity-component-system (ECS) core with plugins — a physics plugin
  (DART by default), sensor plugins rendering through `gz-rendering` (OGRE 2), and system
  plugins for controllers and joints. The server can run **headless** (`gz sim -s -r`), which
  is what makes it usable in CI; the GUI is a separate process.
- **The bridge is the thing to understand for ROS 2:** Gazebo has its own transport
  (`gz-transport`, its own discovery), so `ros_gz_bridge` translates message types both
  ways, per topic, with a declared mapping. Every bridged topic is a copy and a conversion
  — a real cost for images and point clouds, and the usual first performance surprise.
- **The number that matters:** the **real-time factor** (RTF). Physics stepping at 1 kHz with
  a few LiDARs will drop RTF below 1 and your controller's notion of time diverges from
  wall clock — which is why simulated runs must use `use_sim_time:=true` and the
  `/clock` topic, and why forgetting that is the classic "my TF lookups all fail in sim" bug.
- **Interviewer's target sentence:** "gz-sim Harmonic with ROS 2 Jazzy through `ros_gz`;
  headless server for CI, `use_sim_time` and `/clock` so every node shares simulated time,
  and I watch the real-time factor because a sim that can't keep up changes the behaviour
  you're testing."
- **`piros2` line:** not used. The repo is deliberately hardware-first — a real Pi, a real
  camera, real Wi-Fi — and its "unreal" runs are bag replays rather than a physics world.
  The one place sim would have paid: there is no simulated camera, so every camera bug
  (the 0.73 s stamps, the persistent V4L2 exposure state, a leaked usb_cam holding the
  device) had to be found on hardware.

## 2. Isaac Sim, Isaac ROS

- **Isaac Sim** is NVIDIA's Omniverse-based simulator: USD scene description, RTX
  ray-traced rendering (so camera and LiDAR sensor models are *rendered*, not approximated),
  PhysX for dynamics, and a Python/`omni.isaac` scripting layer. Its selling points are
  photorealism for training perception, **synthetic data generation with automatic ground
  truth** (segmentation masks, depth, bounding boxes, poses — free labels), and domain
  randomisation. Its costs are a large GPU requirement and a heavyweight install.
- **Isaac Lab** (successor to Isaac Gym/OmniIsaacGymEnvs) is the RL training layer —
  thousands of parallel environments on the GPU, which is how modern locomotion and
  manipulation policies get trained.
- **Isaac ROS** is a different thing and the distinction is worth stating: it is a set of
  **GPU-accelerated ROS 2 packages** ("GEMs" — visual SLAM/`cuVSLAM`, nvblox for 3D
  reconstruction, AprilTag detection, stereo depth, DNN inference) built on NITROS, which
  passes GPU buffers between composed nodes **without copying to host memory**. It runs on
  real Jetsons, not in a simulator. Confusing Isaac Sim with Isaac ROS is a common slip.
- **When to reach for it:** perception that needs photoreal appearance or free labels, RL,
  or an NVIDIA-centric edge stack. When not to: pure kinematics/controls testing, where
  gz-sim starts in seconds.
- **Interviewer's target sentence:** "Isaac Sim for photoreal synthetic data and RL at
  scale on USD scenes; Isaac ROS is the separate, GPU-accelerated ROS 2 package set with
  zero-copy NITROS pipelines for Jetson — related brand, different job."
- **`piros2` line:** not used, though the repo lives on the edge of it: the depth network
  is ONNX Runtime with the CUDA/cuDNN execution providers on a GTX 1660 SUPER (72–79
  ms/frame, versus 280–305 ms on CPU), and the *silent* CPU fallback when the NVIDIA pip
  libraries or `preload_dlls()` are missing is documented. That is the same "GPU path is
  fast and fails quietly" territory Isaac ROS optimises, reached from the plain-ONNX side.

## 3. Physics engines: ODE, Bullet, PhysX

- **What a rigid-body engine does each step:** integrate free motion, detect collisions
  (broadphase → narrowphase), then solve contact and joint constraints — the solver being
  where accuracy, stability and speed trade off. Most robotics engines use an iterative
  LCP-style solver: more iterations = stiffer, more stable contacts = slower.
- **ODE** — old, fast, forgiving, the Gazebo Classic default; contacts are soft with
  ERP/CFM parameters you tune by feel. **Bullet** — widely used, good general collision,
  strong in games and in `pybullet` for RL. **PhysX** — NVIDIA's, GPU-accelerated, the Isaac
  backend, very good at scale. **DART** — accurate articulated-body dynamics (Featherstone),
  the modern `gz-sim` default. **MuJoCo** — soft-constraint contact model, extremely fast
  and smooth-gradient, now open source and dominant in RL research.
- **Where sim lies about physics, and it is always the same places:** friction (Coulomb
  models with a single coefficient vs real stick-slip), contact stiffness and restitution,
  actuator dynamics and backlash, deformables, and aerodynamics. A quadrotor sim without
  rotor drag, ground effect, prop wash and motor lag will fly better than the real thing —
  which is exactly the gap that bites at §6.
- **The practical rule:** the timestep must be small enough for the stiffest element in
  the system (contacts, high-gain controllers), and if you have to raise solver iterations
  to stop jitter, you have usually mismodelled a mass or an inertia by an order of
  magnitude.
- **Interviewer's target sentence:** "Engines differ mostly in the contact solver — ODE
  fast and soft, DART accurate for articulated bodies, PhysX for GPU scale, MuJoCo for
  smooth contact in RL — and the honest limits are always friction, contact and actuator
  dynamics."
- **`piros2` line:** not touched. There is no dynamics anywhere in this repo: the camera is
  carried by hand, so there is no plant, no actuation and nothing to integrate.

## 4. Sensor simulation and noise modelling

- **A simulated sensor that returns perfect data is a trap** — it validates that your code
  runs, and hides every failure your code will actually meet. Useful sensor sim means
  modelling the error, not just the geometry.
- **Cameras:** rendering gives you appearance; you then add Gaussian/shot noise, lens
  distortion, exposure and motion blur, and — the one most often skipped — **rolling
  shutter**. Photoreal (Isaac/RTX) matters when the consumer is a learned network trained
  on real images; a Canny edge detector barely cares.
- **LiDAR:** ray-cast per beam gives ranges. To be useful you add range noise (often
  range-dependent), incidence-angle dropouts, missing returns on dark or specular surfaces,
  intensity, and — for realism that changes SLAM behaviour — **per-point timestamps and the
  motion of the sensor during the sweep**, so deskewing is actually exercised. Dust and
  smoke are essentially never modelled well, which is precisely the underground case.
- **IMU:** this one is easy to do properly and worth doing — white noise plus a random-walk
  bias with the same parameters an Allan variance plot would give you (see
  [04_sensors.md](04_sensors.md) §5), so your estimator's tuning transfers.
- **GNSS:** position noise plus, if you are honest, multipath and dropout — the point being
  to test the *fallback*, not the happy path.
- **Interviewer's target sentence:** "Simulated sensors are only useful with their noise and
  failure modes modelled — IMU bias random walk, LiDAR dropouts on dark surfaces and
  per-point timestamps so deskew is exercised, camera rolling shutter and exposure —
  otherwise you're testing against a sensor you'll never own."
- **`piros2` line:** the repo does exactly this **at the unit-test level rather than in a
  simulator**, which is the same idea one layer down: a **fake ONNX session** for the depth
  estimator, **synthetic depth planes** for the projector, **synthetic chessboards** for the
  keypoint detector, seeded-noise ray bundles for the rotation geometry, a **fake `/proc`
  tree** for the camera launch's busy-device pre-flight, and — the sharpest one — gate bags
  whose injected frames model two *specific* sensor failures: `--fill noise` (coarse blobs
  giving hundreds of ORB keypoints that match nothing, the way motion blur behaves) and
  `--fill black` (a near-black frame with faint noise, the way a covered lens behaves — and
  deliberately not pure black, because the detector CRC-skips byte-identical frames whole).

## 5. URDF, SDF, xacro

- **URDF** — ROS's robot description: a tree of `<link>`s (visual, collision, inertial) and
  `<joint>`s. Deliberately limited: **a tree only**, so no closed kinematic chains; no
  worlds, no sensors natively (they arrive via `<gazebo>` extension tags), no multiple
  robots.
- **SDF** — Gazebo's format and a superset: worlds, lights, physics settings, sensors,
  plugins, nested models, closed chains. `gz-sim` speaks SDF natively and converts URDF on
  import.
- **xacro** — the XML macro preprocessor that makes either bearable: properties, maths,
  `<xacro:macro>` for a repeated leg or wheel, and conditionals. Run `xacro model.urdf.xacro`
  to see what actually gets loaded — the standard first debugging step. In ROS 2 launch, the
  usual idiom is `Command(['xacro ', path])` into the `robot_description` parameter, consumed
  by `robot_state_publisher`, which then publishes the TF tree from joint states.
- **The detail that catches people:** *inertials*. Everyone gets the visual mesh right and
  leaves a 1 kg unit-inertia placeholder on every link, then wonders why the sim is unstable
  or the arm sags. Also: collision geometry should be primitives or a convex hull, never the
  full visual mesh, or collision checking crawls.
- **Interviewer's target sentence:** "URDF is a tree with no worlds or sensors, SDF is
  Gazebo's superset that has both, xacro is how you keep either maintainable — and the thing
  that actually breaks sims is wrong inertials and using visual meshes for collision."
- **`piros2` line:** **not used at all, deliberately.** There is no URDF in the repo; the
  frames it needs (`base_link → camera_link → camera_optical_frame`) are published as a
  static TF chain from `camera.launch.py` with a placeholder mount pose and the canonical
  −90/0/−90 optical rotation. For a fixed camera on a tripod that is the honest minimum —
  a URDF would be ceremony around three numbers. I would need one the moment there was a
  moving joint or a second sensor to place.

## 6. The sim-to-real gap, and what transfers

- **What the gap is made of:** unmodelled dynamics (friction, backlash, aerodynamics,
  flex), sensor appearance and noise mismatch, latency and jitter that a synchronous sim
  hides, and — the sneakiest — **the world's variety**. Sim scenes are tidy; real mines have
  dust, cabling, water, mesh, glare and people.
- **What transfers well:** software integration (does the graph wire up, do the frames
  agree, does the state machine reach the state), geometry and kinematics, planner logic and
  the coverage of edge cases, failure-injection behaviour (what happens on comms loss),
  and anything algorithmic where the input is geometric rather than photometric.
- **What transfers badly:** anything tuned against contact dynamics or aerodynamics,
  perception trained purely on rendered images, timing/latency margins, and estimator tuning
  based on synthetic noise that doesn't match the real Allan variance.
- **The techniques that narrow it:** **domain randomisation** (randomise textures, lighting,
  masses, latencies so the policy must be robust to a distribution that contains reality),
  **system identification** (measure the real plant and fit the sim to it), **residual /
  hybrid models** (learn the difference between sim and real), and simply **validating on
  real logs** — which loops back to §9.
- **Interviewer's target sentence:** "Integration, geometry, planning logic and failure
  handling transfer; contact dynamics, aerodynamics, photometric perception and timing
  margins don't — so I use sim for the first set and recorded real data plus field trials
  for the second."
- **`piros2` line:** the repo's whole verification doctrine is a statement about this gap —
  it decided that for an observe-only pipeline, **replaying real sensor data is strictly
  better evidence than simulating fake sensor data**, and built the tooling for it. The
  cases it can't cover are named honestly: the tape-measure scale check, exposure in a real
  room, and a motion that was never recorded — those are still "needs a human", and the file
  says what one recording would turn each into.

## 7. Headless simulation in CI

- **The mechanics:** run the simulator with no GUI (`gz sim -s -r --headless-rendering`),
  usually inside a container, with `use_sim_time` everywhere; drive it from a test script;
  assert on outcomes; tear it down. Rendering sensors still need a GPU or software GL
  (`LIBGL_ALWAYS_SOFTWARE=1`, or EGL headless), which is the first thing that breaks on a
  cloud runner.
- **The requirements that make it survive:** **determinism or, failing that, seeded runs with
  tolerances** (physics with variable timesteps and multi-threading is rarely bit-exact —
  chasing exact reproducibility is usually the wrong fight; bounded assertions are the right
  one); **timeouts on everything** so a hung sim fails instead of occupying the runner;
  **artifacts on failure** (log, bag, screenshot) so the failure is diagnosable without a
  re-run; and **hermetic teardown** so a crashed process doesn't poison the next job.
- **The value:** a nightly job that flies 200 scripted missions and reports which failed is
  a different quality bar from "we flew it and it seemed fine". For an autonomy company this
  is the regression net around the state machine and the failsafes.
- **Interviewer's target sentence:** "Headless server in a container, sim time everywhere,
  bounded assertions rather than bit-exact determinism, hard timeouts and artifacts on
  failure — otherwise you get a flaky job people learn to ignore."
- **`piros2` line:** no simulator, but **the headless discipline is built and proven** on
  replay: `just gate flick` and `just gate occlude` launch the full pipeline with no GUI,
  play a gate bag once, and **exit 0 or 1** from `tools/verify/gate_check.py` — which
  asserts both a numeric threshold (the A′-vs-A pose error over the tail) *and* the presence
  of the log lines the plan promised, writing `report.json`, `poses.csv`, `poses.png` and
  `launch.log` into `captures/verify/`. That is CI-shaped in every respect except being
  wired to a CI service — and there is no `.github/` in this repo, which is the honest gap.

## 8. Scenario-based and regression testing in simulation

- **Scenario-based testing** = define situations the robot must handle (a corridor
  narrowing to 2.4 m, comms lost at waypoint 3, a battery threshold hit mid-mission, a
  degenerate tunnel, a moving obstacle), express each as a repeatable scenario file, and
  assert on outcomes (reached the rally point; never came within X of a wall; returned home
  within Y seconds). Automotive formalises this (OpenSCENARIO/OpenDRIVE); robotics usually
  rolls its own YAML.
- **Coverage thinking:** you cannot enumerate the world, so you parameterise scenarios
  (gap width, light level, comms-loss timing) and sweep, then look for the *boundary* where
  behaviour changes — that boundary is your real spec. Fuzzing and adversarial search
  (find the parameters that break it) are the grown-up version.
- **Regression** = pin the scenarios that once failed. Every field incident should end as a
  scenario in the suite, which is the mechanism that stops the same bug arriving twice.
- **Metrics, not eyeballs:** success rate, minimum clearance, time to complete, path length,
  energy, estimator error against sim ground truth. Sim's unfair advantage is that ground
  truth is free — use it.
- **Interviewer's target sentence:** "Parameterised scenarios with assertions on outcomes,
  swept to find the boundary where behaviour changes, and every field failure pinned as a
  regression scenario — with metrics rather than someone watching a viewport."
- **`piros2` line:** the repo does this shape on bags. The two relocalization gates are
  *scenarios* — "flick away and back" (`A → noise → B → noise → A′`, kp mode) and "cover the
  lens and uncover on a known view" (`A → noise → A′`, rgbd mode) — cut from a real sweep by
  `make_gate_bag.py` with thresholds in a `gate.json` beside the bag. Both PASS with numbers
  (65.3° correction, 0.48° tail; 18.4° snap, 0.95°/3 cm tail). And the regression instinct is
  there: the black-fill variant **failed first**, exposed a real bug (a blackout wasn't being
  counted as tracking loss), and the fix shipped with two unit tests plus the gate bag that
  reproduces the 19.7° failure when the flag is toggled off.

## 9. Record and replay as an alternative to simulation

- **The argument:** for any component that only consumes sensor data and produces an
  estimate or a map, recorded real data beats simulated data on every axis that matters —
  it has the real noise, the real timing, the real driver bugs, and the real world's mess.
  You cannot test a controller with it, but you can test perception and estimation to a
  standard a simulator will never reach.
- **What makes it work in practice:** a bag format that stores *what the sensor actually
  sent* (compressed images, per-point fields) with original stamps; the ability to replay at
  a controlled rate; and — the subtle one — **not looping**, because a looped bag teleports
  the odometry back to the start and makes the pipeline diverge in ways the real world never
  would.
- **The limits, stated honestly:** no closed loop; the bag ages (a `/camera_info` recorded
  before intrinsics existed is useless); and you can only replay motions someone recorded.
  The clever middle ground is **editing bags into new scenarios** — re-ordering, cutting and
  splicing views you already have into a motion nobody performed.
- **Interviewer's target sentence:** "Replay is the right tool for anything open-loop —
  perception, depth, SLAM front-ends, mapping — because it carries the real sensor's noise
  and bugs; simulation is for anything closed-loop, where the world has to react. And bags
  can be *edited* into scenarios, which covers more ground than people expect."
- **`piros2` line:** **this is the repo's answer to the whole section, and it is built.**
  `just record [seconds] [name]` bags the compressed stream + `camera_info` + `/tf_static`
  on the Pi and fetches it; `just replay` runs a bag through `image_transport republish`
  into the edge detector entirely on the dev box; **`just run-bag [bag]` runs the entire
  `world_mesh` session from a bag with no Pi and no camera** — same nodes, same topics, same
  services, so `just snap` and `just mesh-save` work against it. `just gate-bags` *edits* a
  real 44 s sweep into the two gate bags, preserving each frame's header→receive offset so
  the camera's 0.73 s stamp fault survives the edit on purpose. The known trap is recorded
  too: **looping a bag teleports odometry**, so `just map` plays once; and a bag whose
  `/camera_info` predates the intrinsics (K all zeros) cannot feed mapping, which is why
  `bags/static1` exists as the plumbing bag. The most recent SLAM work leaned on the same
  layer: a **palindrome** bag (`sweep3` played forward then reversed, header timeline
  mirrored) manufactures a loop closure out of a sweep that never looped, giving every
  return frame its own outbound pose as reference — a motion nobody performed, assembled
  from data we had.

## What to say if asked "have you used Gazebo / Isaac?"

"No — I've read about gz-sim Harmonic with Jazzy through `ros_gz`, and I know Isaac Sim
from Isaac ROS, but I haven't shipped either. What I did instead was build the replay half
properly: the whole session runs from a recorded bag with no hardware, and the gates that
used to say 'a human waves the camera and watches RViz' are now scripts that replay an
edited bag headlessly and exit 0 or 1 with a number and a plot. I know the boundary of that
approach — a bag can't react to what my code decides, so the moment there's a controller or
an autonomy state machine in the loop I need a simulator, and that's the piece I'd be
learning on the job."
