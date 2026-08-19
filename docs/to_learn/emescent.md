# Robotics Syllabus

**Every technical topic on the Emesent Cortex JD, expanded.** Built 2026-08-18 from
[conversations/emesent.md](conversations/emesent.md), the day before the 08-19 panel. It is a
**syllabus, not a to-do list**: tick what is already held, and the untickeds are the ramp.

**Scope note.** This is the robotics and real-time lane specifically. It does not cover the
web/backend stack (already strong, see [skills.md](skills.md)) or the orchestration gap
(Kubernetes/OpenShift, tracked in [goals.md](goals.md)).

**Priority is already set in [goals.md](goals.md) and does not change here: C++ inside ROS
first, real SLAM second, RL/behavioural cloning third.** Sections 8 and 2 below are therefore
the ones that matter; the rest is breadth.

**Honest-claim rule while working through this.** Reading about a topic does not move it into
[skills.md](skills.md). A tick here means "can hold a technical conversation"; a skills.md
entry means "have built with it". **The C/C++ and SLAM boundaries in
[interview-answers.md](interview-answers.md) stay exactly as written until a project changes
them, not until a tutorial does.**

## 1. Company and product domain

**Researched 2026-08-18 → [01_emesent-company-domain.md](01_emesent-company-domain.md)** — every item
below has a sourced section there, plus a "could not verify" list and one-liners for the room.
Boxes stay unticked until the material is *held*, per the honest-claim rule above.

- [ ] Emesent history: CSIRO Data61 spinout, 2018, what was commercialised
- [ ] Hovermap product line and what it does
- [ ] Cortex: software and control architecture, handheld through to autonomous
- [ ] Handheld vs drone vs ground-robot mapping modes
- [ ] Mining and tunnel inspection as a domain: what customers buy and why
- [ ] GPS-denied environments: what breaks, and why it matters commercially
- [ ] Point cloud deliverables and downstream spatial data workflows
- [ ] Competitors: NavVis, Leica BLK, Exyn, Flyability

## 2. SLAM

**The second-priority gap in [goals.md](goals.md), and the one most likely to be tested by a
robotics employer.** `piros2` *was* rotation-only and NOT SLAM until
2026-08-18/19, when its fork grew a real backend — loop-closure
detection, a hand-written pose-graph optimiser owning `map → odom`, a
TSDF that follows the graph, a persistent graph — all gated by scripts
against RTAB-Map and TUM ground truth
([slam-plan.md](../plans/completed/slam-plan.md)); the honest wording
now is "monocular RGB-D-style SLAM in one room, hand-written backend";
see
[projects.md](projects.md).

**Study file, written 2026-08-18 → [02_SLAM.md](02_SLAM.md)** — one section per item below (concept,
the sentence an interviewer wants, and an honest `piros2` line), plus a Wildcat section read
from the paper and the boundary answer to "have you built SLAM?". Boxes stay unticked until held.

- [ ] Definition: simultaneous localization and mapping, and why the two are coupled
- [ ] Frontend vs backend
- [ ] Odometry vs SLAM vs localisation in a known map
- [ ] Pose graph optimisation
- [ ] Loop closure detection and correction
- [ ] Drift, revisit, global consistency
- [ ] Scan matching: ICP (point-to-point, point-to-plane), NDT, GICP
- [ ] Feature-based vs direct methods
- [ ] Visual SLAM: ORB-SLAM, VINS
- [ ] LiDAR SLAM: LOAM, LeGO-LOAM, FAST-LIO, KISS-ICP
- [ ] LiDAR-inertial odometry, tightly vs loosely coupled
- [ ] Continuous-time SLAM and trajectory representation (the CSIRO Wildcat lineage)
- [ ] Submaps and map merging
- [ ] Factor graphs: GTSAM, Ceres, g2o
- [ ] Bundle adjustment
- [ ] Degeneracy: why long featureless tunnels break scan matching

## 3. State estimation and maths

**Study file → [03_state-estimation-and-maths.md](03_state-estimation-and-maths.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] SE(3), SO(3), rigid transforms
- [ ] Quaternions vs Euler vs rotation matrices, gimbal lock
- [ ] Lie groups, Lie algebra, manifold optimisation
- [ ] Kalman filter, EKF, UKF
- [ ] Complementary filter
- [ ] Particle filter / Monte Carlo localisation
- [ ] Covariance, uncertainty propagation, observability
- [ ] Least squares, robust cost functions (Huber, Cauchy)
- [ ] RANSAC
- [ ] Kabsch / Procrustes, Umeyama alignment
- [ ] Pose interpolation and extrapolation (SLERP)

## 4. Sensors

**Study file → [04_sensors.md](04_sensors.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] LiDAR: spinning vs solid state, channels, range, FOV, returns, intensity
- [ ] Motion distortion and scan deskewing
- [ ] Time-of-flight vs phase-shift ranging
- [ ] IMU: accelerometer, gyroscope, magnetometer; 6 vs 9 axis; MARG; AHRS
- [ ] IMU bias, random walk, Allan variance, temperature drift
- [ ] IMU preintegration
- [ ] Depth cameras: stereo, structured light, time of flight
- [ ] Monocular depth estimation and its scale ambiguity
- [ ] Cameras: intrinsics, distortion, rolling vs global shutter, exposure
- [ ] Camera calibration (checkerboard, reprojection error)
- [ ] Extrinsic calibration between sensors
- [ ] Time synchronisation, hardware triggering, PTP/NTP
- [ ] Encoders and wheel odometry
- [ ] Barometers, altimeters, rangefinders
- [ ] Sensor noise models and failure modes

## 5. Perception and point clouds

**Study file → [05_perception-and-point-clouds.md](05_perception-and-point-clouds.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] Point cloud data structures: PCL, Open3D, PointCloud2
- [ ] Voxel grids, octrees, OctoMap, KD-trees, nearest-neighbour search
- [ ] Downsampling, filtering, outlier removal
- [ ] Normal estimation
- [ ] Registration and alignment
- [ ] TSDF fusion, surfels, meshing (Poisson, marching cubes)
- [ ] Occupancy mapping: free vs unknown space
- [ ] Ray casting
- [ ] Segmentation: ground plane, clustering, semantic
- [ ] 3D object detection
- [ ] Computer vision: features (ORB, SIFT), optical flow, epipolar geometry, essential and
      fundamental matrices, triangulation
- [ ] Deep learning inference on edge: ONNX, TensorRT, quantisation

## 6. Navigation and planning

**Study file → [06_navigation-and-planning.md](06_navigation-and-planning.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] Global vs local planning
- [ ] A*, Dijkstra, D* Lite, hybrid A*
- [ ] RRT, RRT*, PRM, sampling-based planning
- [ ] Costmaps, inflation, obstacle layers
- [ ] Local planners: DWA, TEB, MPPI
- [ ] Trajectory optimisation and smoothing
- [ ] Frontier exploration and autonomous exploration
- [ ] Coverage planning
- [ ] Collision checking
- [ ] Nav2 architecture and behaviour trees
- [ ] Recovery behaviours and failsafes
- [ ] Kinodynamic constraints, holonomic vs non-holonomic

## 7. Control

**Study file → [07_control.md](07_control.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] PID: tuning, windup, derivative filtering
- [ ] Cascaded control loops
- [ ] Feedforward vs feedback
- [ ] State space, LQR
- [ ] MPC
- [ ] Drone flight control: attitude, rate and position loops
- [ ] Actuator saturation and rate limits
- [ ] Control loop timing and sample rates

## 8. C++

**Study file → [08_cpp.md](08_cpp.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

**THE FIRST-PRIORITY GAP.** Four processes in one week turned on it (Emesent, Arista, Anduril,
the defence lane). The standing claim is **embedded firmware C/C++, not large-scale
application or STL C++**, and that wording does not change until something is built.

- [ ] C++14/17/20 feature sets and what each added
- [ ] RAII
- [ ] Smart pointers: unique_ptr, shared_ptr, weak_ptr
- [ ] Move semantics, rvalue references, perfect forwarding
- [ ] Rule of 0/3/5
- [ ] Const correctness
- [ ] Templates, specialisation, SFINAE, concepts
- [ ] STL containers and their complexity guarantees
- [ ] STL algorithms and iterators
- [ ] Lambdas and std::function
- [ ] std::optional, variant, string_view, structured bindings
- [ ] Exceptions and error-handling strategy, expected
- [ ] Threading: std::thread, mutexes, condition variables, atomics
- [ ] Memory model, data races, false sharing
- [ ] Lock-free structures, ring buffers, SPSC queues
- [ ] Allocation: heap vs stack, custom allocators, memory pools
- [ ] Undefined behaviour
- [ ] Build systems: CMake, targets, linking
- [ ] Package management: Conan, vcpkg
- [ ] Debugging: gdb, valgrind, sanitizers (ASan, TSan, UBSan)
- [ ] Profiling: perf, flame graphs, cachegrind
- [ ] Compiler optimisation, inlining, LTO
- [ ] Static analysis: clang-tidy, cppcheck
- [ ] Python interop: pybind11

## 9. Real-time and performance

**Study file → [09_real-time-and-performance.md](09_real-time-and-performance.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] Hard vs soft real time
- [ ] Determinism, worst-case execution time, jitter
- [ ] Scheduling policies, priorities, priority inversion
- [ ] PREEMPT_RT and kernel latency
- [ ] Why allocation, locks and exceptions are avoided in hot paths
- [ ] Cache locality and data-oriented design
- [ ] Latency vs throughput trade-offs
- [ ] Zero-copy and shared memory
- [ ] Benchmarking methodology

## 10. ROS and ROS 2

**Study file → [10_ros-and-ros2.md](10_ros-and-ros2.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

**The strongest section, and the one `piros2` already covers in Python.** The untickeds worth
attention are rclcpp, composition and lifecycle nodes.

- [ ] ROS 1 vs ROS 2 differences, and why ROS 2 exists
- [ ] Nodes, topics, services, actions, parameters
- [ ] DDS, the RMW layer, CycloneDDS vs FastDDS
- [ ] QoS: reliability, durability, history, deadline, liveliness
- [ ] Executors, callback groups, spin behaviour
- [ ] Node composition and intra-process zero-copy
- [ ] Lifecycle (managed) nodes
- [ ] TF2: frames, transform trees, static vs dynamic
- [ ] Message and interface definitions
- [ ] Launch files and launch composition
- [ ] rosbag2, MCAP, record and replay
- [ ] RViz, rqt, the ros2 CLI
- [ ] Real-time ROS 2 considerations
- [ ] **rclcpp specifically, not just rclpy**
- [ ] Multi-machine ROS 2: discovery, domain IDs, network tuning

## 11. Simulation

**Study file → [11_simulation.md](11_simulation.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] Gazebo / Ignition / gz-sim
- [ ] Isaac Sim, Isaac ROS
- [ ] Physics engines: ODE, Bullet, PhysX
- [ ] Sensor simulation and noise modelling
- [ ] URDF, SDF, xacro
- [ ] The sim-to-real gap, and what transfers
- [ ] Headless simulation in CI
- [ ] Scenario-based and regression testing in simulation
- [ ] Record and replay as an alternative to simulation

## 12. Testing

**Study file → [12_testing.md](12_testing.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] Unit, integration, system and regression levels
- [ ] Test doubles: mocks, fakes, stubs
- [ ] GoogleTest, GMock
- [ ] pytest, fixtures, parametrisation
- [ ] Testing without hardware: fake drivers, synthetic data
- [ ] Property-based testing
- [ ] Golden-file and snapshot testing for pipelines
- [ ] Hardware-in-the-loop testing
- [ ] Field trial methodology and validation criteria
- [ ] Coverage, and its limits
- [ ] Flaky tests and non-determinism in robotics tests

## 13. CI/CD, deployment, infrastructure

**Study file → [13_ci-cd-deployment-infrastructure.md](13_ci-cd-deployment-infrastructure.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] GitHub Actions, runners, matrix builds
- [ ] Build caching for C++
- [ ] Docker, multi-stage builds, image size
- [ ] Containerised development, devcontainers
- [ ] Cross-compilation and toolchains
- [ ] Artifact and release management, versioning
- [ ] Over-the-air and field update strategies
- [ ] Rollback and recovery
- [ ] AWS: EC2, S3, IAM, deployment patterns
- [ ] Cloud ingest of large spatial datasets
- [ ] Git workflow, branching, code review practice

## 14. Embedded and edge

**Study file → [14_embedded-and-edge.md](14_embedded-and-edge.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] ARM targets, Jetson platforms, single-board computers
- [ ] Compute and power budgets
- [ ] GPU acceleration on edge, CUDA basics
- [ ] Cross-compilation and sysroots
- [ ] Serial, I2C, SPI, CAN
- [ ] Device drivers and kernel interfaces
- [ ] Boot, init and systemd on embedded Linux
- [ ] Thermal and resource constraints
- [ ] Firmware update paths

## 15. Linux

**Study file → [15_linux.md](15_linux.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

- [ ] Processes, threads, scheduling
- [ ] Memory management, page cache, OOM
- [ ] Filesystems and I/O
- [ ] Networking: sockets, UDP fragmentation, MTU, multicast
- [ ] Network tuning for DDS
- [ ] systemd services and journald
- [ ] Performance tools: top, htop, perf, strace, ltrace, iotop
- [ ] Shell scripting

## 16. Agentic engineering

**Study file → [16_agentic-engineering.md](16_agentic-engineering.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

**Listed as a nice-to-have on the Emesent JD and largely already held** via the Stock Market
Analyser's ten agent workflow patterns ([projects.md](projects.md)).

- [ ] Agent workflow patterns: routing, chaining, parallelisation, ReAct,
      evaluator-optimizer, plan-and-execute, orchestrator-workers, multi-agent
- [ ] Tool use and function calling
- [ ] Local model serving (Ollama) vs hosted APIs
- [ ] Retrieval and vector search
- [ ] Durable long-running agent runs, cancellation
- [ ] Evaluating agent output
- [ ] AI coding agents in the development workflow, and where they fail

## 17. Software engineering practice

**Study file → [17_software-engineering-practice.md](17_software-engineering-practice.md)** — one section per item below: the concept, the sentence an interviewer wants, and an honest `piros2` line.

**Section 17's diagram items are the named Andromeda feedback and are already promoted to
short-term in [goals.md](goals.md).**

- [ ] Design reviews, and how to run one
- [ ] Architecture documentation, ADRs
- [ ] **Sequence, container and component diagrams (C4)**
- [ ] API and module boundary design
- [ ] Managing technical debt in a growing stack
- [ ] Debugging methodology for field-reported issues
- [ ] Logging, telemetry and observability on robots
- [ ] Root cause analysis and postmortems
- [ ] Working with hardware and field teams
- [ ] Translating customer requirements into software requirements
- [ ] Communicating trade-offs to non-technical stakeholders
- [ ] Support escalation handling
