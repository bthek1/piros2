# ROS and ROS 2 — the study file for section 10 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at conversation depth,
the sentence an interviewer is fishing for, and a **`piros2` line** — what this repo
actually does that touches the item, verified against the code before it was written.
The syllabus's note for this section: **"The strongest section, and the one `piros2`
already covers in Python. The untickeds worth attention are rclcpp, composition and
lifecycle nodes."** Per the honest-claim rule: reading ≠ holding. The `piros2` lines say
what has actually been built (Jazzy Jalisco, Ubuntu 24.04, `rclpy` throughout); the
rclcpp / composition / lifecycle / real-time items are explained properly here and marked
untouched, because they are.

## Mental model to carry through the whole file

```
 your code ──► rclcpp / rclpy  ──► rcl (C) ──► rmw_<impl> ──► DDS (Fast DDS | CycloneDDS) ──► UDP/SHM
   nodes         client library      common core   middleware       discovery, QoS, RTPS wire
```

QoS, discovery, domain IDs and interface binding are DDS; executors and callback groups
are the client library; TF, launch, rosbag2 and the CLI are tools on top of nodes. Knowing
which layer owns a symptom is most of debugging ROS 2.

## 1. ROS 1 vs ROS 2 differences, and why ROS 2 exists

- ROS 1 (2007–Noetic, EOL May 2025): a single `roscore` master, TCPROS, one node per
  process, `catkin`, XML launch, no QoS, no security, no real-time story — designed for one
  robot in a lab on a wired network.
- ROS 2 (2017→) exists because production wanted: **no single point of failure**
  (peer-to-peer discovery via DDS — no master), **multi-robot and lossy networks** (QoS
  policies to trade reliability for latency), **real-time and embedded** (a C core `rcl`,
  deterministic executors, micro-ROS), **security** (SROS2 — DDS-Security: authenticated,
  encrypted participants), **composition** (many nodes per process, intra-process
  transport), **lifecycle** (deterministic bring-up), Windows/macOS, and a build system
  (`ament` + `colcon`) that isolates packages.
- Day-to-day: `ros2 <verb>` CLI, Python launch, per-node typed parameters declared before
  use, `rclcpp`/`rclpy` mirroring each other, `ros1_bridge` for migration.
- **Interviewer's target sentence:** "ROS 2 replaced the master and TCPROS with DDS so
  discovery is peer-to-peer and transport has QoS; the rest — lifecycle, composition,
  security, real-time executors — are the things you need to ship a robot rather than run
  a lab demo."
- **`piros2` line:** Jazzy end to end, ROS 1 never touched — which is why the docs carry
  warnings like "`camera_frame_id` is a ROS 1 name usb_cam silently ignores; the Jazzy
  param is `frame_id`" and "usb_cam's exposure params use ROS 1-era control names the
  kernel renamed". Migration residue is a real category of bug when you read old answers.

## 2. Nodes, topics, services, actions, parameters

| Primitive | Pattern | Use when | CLI |
| --- | --- | --- | --- |
| **Topic** | pub/sub, many-to-many, anonymous, typed `.msg` | continuous data: images, clouds, TF, state | `ros2 topic list/echo/hz/bw/delay/info -v` |
| **Service** | request/response, one server, blocking or async client | short, must-complete queries: reset, save, get_state | `ros2 service call /n/reset std_srvs/srv/Trigger` |
| **Action** | goal → feedback stream → result, cancellable, one server | long tasks with progress: navigate, dock, record 30 s | `ros2 action send_goal --feedback` |
| **Parameter** | per-node typed key/value, declared, YAML-loadable, change callbacks | tunables and configuration | `ros2 param get/set/dump/load` |

- A **node** is the unit of the graph: it owns publishers, subscriptions, services,
  timers, parameters and a logger; several nodes can share one process (§6). Names are
  namespaced; `~/reset` expands to `/<node_name>/reset` — the idiom for a node's own
  services.
- The classic deadlock: a *synchronous* service call from inside a callback on a
  single-threaded executor blocks the executor that must deliver the response — use
  `call_async` / a future callback, or a Reentrant group. Actions are topics + services
  underneath; Nav2's `NavigateToPose` is the canonical one — anything a person might cancel
  or watch is an action.
- Parameters must be declared (`declare_parameter('depth_scale', 1.0)`) — undeclared
  ones are rejected unless `allow_undeclared_parameters`; setting a wrong type is
  refused; `add_on_set_parameters_callback` validates, `add_post_set_parameters_callback`
  (Iron+) reacts. YAML shape: `node_name: {ros__parameters: {...}}`, `/**:` as a wildcard.
- **Interviewer's target sentence:** "Topics for streams, services for short atomic
  requests, actions for anything long or cancellable, parameters for configuration — and
  never block an executor waiting for a service response."
- **`piros2` line:** all four in the fork. Topics: `/image_raw/compressed` → relay →
  `/depth`, `/depth/rgb`, `/points`, `/world/mesh_live`, `/keypoints/count` as a plain
  `std_msgs/Int32` (the comment says why: a custom message would need its own rosidl
  `ament_cmake` package). Services: `~/reset` and `~/save_map` on the detector, `~/reset`
  and `~/save` on the mesher (`std_srvs/Trigger`; `just mesh-save` / `just map-save` call
  them), and a `create_client(ResetPose, '/reset_odom_to_pose')` whose `call_async` fires
  from inside a callback — the non-blocking form, on purpose. Parameters: every node
  declares its knobs (`tsdf_mesher` declares 21) and reads `config/world_mesh.yaml`. No
  action server has been written — the nearest thing is `just record <secs> <name>`, which
  is *exactly* the shape (a long task with a duration) done as a `timeout -s INT` instead.

## 3. DDS, the RMW layer, CycloneDDS vs FastDDS

- **DDS** (OMG standard) is a data-centric pub/sub middleware: participants, topics with
  types, writers/readers, QoS, and the **RTPS** wire protocol over UDP (multicast for
  discovery, unicast for data). Discovery is two-phase: **SPDP** (participants announce
  on the multicast address, port `7400 + 250·domain` — domain 42 → 17900) then **SEDP**
  (endpoints exchange topic/type/QoS and match). No master; every participant knows the
  whole graph.
- **rmw** is ROS 2's abstraction over vendors: `rmw_fastrtps_cpp` (default in Jazzy),
  `rmw_cyclonedds_cpp` (Tier 1), `rmw_connextdds`, and `rmw_zenoh_cpp` (Zenoh, not DDS,
  packaged for Jazzy). Chosen at runtime by `RMW_IMPLEMENTATION`; all nodes on a domain
  must agree, and the ROS daemon caches discovery so it must be restarted after any change.
- **Fast DDS**: multi-threaded, shared-memory transport on by default, data-sharing, a
  *Discovery Server* mode that replaces multicast. **CycloneDDS**: single-threaded, small,
  an XML/`CYCLONEDDS_URI` config that is easy to reason about, no shared memory without
  iceoryx, a reputation for behaving on lossy Wi-Fi; the default in Galactic only. The
  practical differences are discovery flakiness, interface selection and socket buffers.
- The trap every multi-homed host hits: DDS binds *all* interfaces and advertises every
  address; a peer picks one it can't route to. Fix by pinning the interface (Cyclone:
  `<NetworkInterface name="…">`; Fast DDS: an XML transport whitelist).
- **Interviewer's target sentence:** "rmw makes the vendor swappable but not invisible —
  discovery, interface binding, buffer sizes and shared-memory behaviour differ, so you
  pick one per fleet, pin it in the environment, and pin the interface it may use."
- **`piros2` line:** `rmw_cyclonedds_cpp` on both machines, chosen and set once in
  Ansible `group_vars/all.yml`; the `ros2_env` role renders `cyclonedds.xml.j2` per host
  (`dds_interface: enp6s18` on the dev box, `wlan0` on the Pi) and exports
  `CYCLONEDDS_URI=file://…` from `~/.profile`. The reason it exists is measured: the dev
  box has three Docker bridges, `tailscale0` and a WireGuard `laptop` at `10.8.0.3`, and
  DDS advertised those. `ROS_STATIC_PEERS` / `ROS_AUTOMATIC_DISCOVERY_RANGE` are
  documented in `docs/info/networking.md` as the fallback if the Wi-Fi AP eats
  multicast — not needed so far.

## 4. QoS: reliability, durability, history, deadline, liveliness

| Policy | Values | Meaning |
| --- | --- | --- |
| Reliability | RELIABLE / BEST_EFFORT | retransmit until acked vs fire-and-forget |
| Durability | VOLATILE / TRANSIENT_LOCAL | late joiners get nothing vs get the writer's history ("latching") |
| History + depth | KEEP_LAST n / KEEP_ALL | how many samples the writer/reader queues hold |
| Deadline | period | writer promises, reader expects, a sample at least every *T*; miss → event callback |
| Lifespan | duration | a sample older than this is dropped unread |
| Liveliness | AUTOMATIC / MANUAL_BY_TOPIC + lease | how a writer proves it's alive; `assert_liveliness()` for manual |

- **Compatibility is request-vs-offer**: a subscription may not ask for *more* than the
  publisher offers. RELIABLE sub + BEST_EFFORT pub → **no match, silently** (Jazzy logs an
  incompatible-QoS event if you subscribe to it). TRANSIENT_LOCAL sub + VOLATILE pub →
  no match. Depth is per side and doesn't need to agree.
- Defaults: RELIABLE, VOLATILE, KEEP_LAST 10; `SensorDataQoS` = BEST_EFFORT, KEEP_LAST 5.
  `/tf_static` is TRANSIENT_LOCAL — why a transform published once is seen by nodes started
  later. The doctrine "sensor data = BEST_EFFORT" assumes messages that fit in a datagram;
  a fragmented message needs *every* fragment, and BEST_EFFORT never retransmits.
- **Interviewer's target sentence:** "QoS is a contract matched at discovery — a
  subscription can't demand more than the publisher offers, and a mismatch is a silent
  non-match, not an error — so the first question about a topic nobody receives is
  `ros2 topic info -v`."
- **`piros2` line:** three measured lessons, all in code comments. (1) `BIG_FRAME_QOS` =
  RELIABLE / KEEP_LAST / depth 1 on every image consumer: a 720p rgb8 frame is 2.7 MB →
  ~1800 UDP fragments, one always drops with default socket buffers, so BEST_EFFORT
  received **zero** frames while RELIABLE received instantly (`ros2 topic echo
  --qos-reliability best_effort` reproduces it). Publishers are RELIABLE too, because
  RViz and rqt_image_view subscribe RELIABLE and would never match. (2) `LATCHED_QOS` =
  RELIABLE + TRANSIENT_LOCAL + KEEP_LAST 1 for the live-mesh Marker, the keyframe/graph
  markers and every "current state" topic a late-started RViz must see. (3) The gate-bag
  builder copies `offered_qos_profiles` through `rosbag2_py.TopicMetadata` because a bag
  that plays `/tf_static` VOLATILE never matches a tf2 listener. Deadline and liveliness
  are understood, not used — the dashboard measures staleness against its own receipt
  clock instead (a deadline event would be the idiomatic form).

## 5. Executors, callback groups, spin behaviour

- An **executor** owns a wait set of every entity (subscription, timer, service, client,
  waitable) of the nodes added to it; `spin()` waits for readiness and dispatches
  callbacks. rclcpp: `SingleThreadedExecutor` (default of `rclcpp::spin`),
  `MultiThreadedExecutor` (a thread pool), `StaticSingleThreadedExecutor`, and the
  `rclcpp::experimental::executors::EventsExecutor` (event-queue instead of a wait set,
  lower overhead). rclpy: `SingleThreadedExecutor` default, `MultiThreadedExecutor
  (num_threads=…)`. `spin_some` / `spin_once` / `spin_until_future_complete` for manual loops.
- **Callback groups** decide what may run *concurrently* under a multi-threaded executor:
  `MutuallyExclusive` (the default group of every node — callbacks in it are serialised)
  and `Reentrant` (anything may overlap, including the same callback with itself). Put a
  slow subscription and a heartbeat timer in different groups or the timer starves; put a
  service client's response in a different group from the caller or a synchronous call
  deadlocks.
- rclcpp code shape:
  ```cpp
  auto cg = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions opts; opts.callback_group = cg;
  sub_ = create_subscription<sensor_msgs::msg::Image>("depth", rclcpp::SensorDataQoS(), cb, opts);
  rclcpp::executors::MultiThreadedExecutor exec; exec.add_node(node); exec.spin();
  ```
- rclpy's `MultiThreadedExecutor` overlaps I/O, not CPU — a call holding the GIL blocks
  every other callback.
- **Interviewer's target sentence:** "The executor decides *when* callbacks run and
  callback groups decide *whether two may overlap*; the default is one thread and one
  mutually-exclusive group, so a blocking call in a callback stalls the whole node — the
  fixes are async calls, groups, or a multi-threaded executor."
- **`piros2` line:** every node runs the default single-threaded rclpy executor —
  `grep` finds no `MultiThreadedExecutor` or callback group in `src/`. The executor
  lesson was learnt the hard way in the mesher instead: a 12–21 s mesh refresh run inline
  starved integration (~50 frames of an 88 s bag), moved to a *thread* it still starved
  it (Open3D holds the GIL — zero frames for 20 s at a time, a loop bag's memory came out
  90 outbound / 13 return frames), and only a separate interpreter freed the node —
  `mesh_worker.py`'s `MeshFinisher`, a spawn-context `multiprocessing.Process` with a
  non-blocking poll. That is the executor problem solved outside the executor, honestly.

## 6. Node composition and intra-process zero-copy

- **Composition** = several nodes in one process. Write the node as a class taking
  `rclcpp::NodeOptions`, build it as a shared library, register it with
  `RCLCPP_COMPONENTS_REGISTER_NODE(ns::MyNode)` and
  `rclcpp_components_register_node(target PLUGIN "ns::MyNode" EXECUTABLE my_node)` in
  CMake (that macro also emits a standalone executable). Load into a container at runtime
  (`ros2 run rclcpp_components component_container` + `ros2 component load /ComponentManager
  pkg ns::MyNode`) or from launch (`ComposableNodeContainer(package='rclcpp_components',
  executable='component_container_mt', composable_node_descriptions=[ComposableNode(...)])`).
  Containers: `component_container` (one single-threaded executor),
  `component_container_mt` (multi-threaded), `component_container_isolated` (an executor
  per node).
- **Intra-process communication** (`NodeOptions().use_intra_process_comms(true)`, or
  `extra_arguments=[{'use_intra_process_comms': True}]` in the launch description): a
  publisher and subscriber in the same process exchange the message through an in-process
  buffer instead of serialising through DDS. Publish a `std::unique_ptr<Msg>` and, with
  one subscriber, ownership *moves* — genuinely zero-copy; with several, rclcpp copies or
  hands out shared const pointers depending on what the callbacks take. Some QoS
  combinations aren't supported (KEEP_ALL, transient-local was added late) and it only
  helps for large messages — images, clouds — where serialisation was the cost.
- **Loaned messages** (`borrow_loaned_message()`) go further — the middleware owns the
  memory (Fast DDS data-sharing / iceoryx), so same-host inter-process avoids the copy too;
  off by default since Iron (`ROS_DISABLE_LOANED_MESSAGES`), POD types only.
- **Interviewer's target sentence:** "Composition is how a perception pipeline gets its
  latency back — camera driver, rectify, detector in one container with intra-process
  comms, publishing `unique_ptr`s so a 2 MB image is moved, not copied — Isaac ROS's
  whole NITROS story is this idea taken to the GPU."
- **`piros2` line:** untouched — every node is a Python process, and it shows: the
  2026-08-16 transport rework found five readers each pulling their own unicast copy of
  the compressed stream over Wi-Fi and fixed it with `camera_relay` (a *process* that
  fans the stream out on loopback) plus `/depth/rgb` (the estimator republishing the
  exact frame it inferred on). In C++ that pipeline would be one composed container with
  intra-process comms and the problem would not exist. Being able to say that — and why
  Python couldn't do it — is the honest position.

## 7. Lifecycle (managed) nodes

- `rclcpp_lifecycle::LifecycleNode` (rclpy: `rclpy.lifecycle.LifecycleNode`) exposes a
  fixed state machine — primary states **Unconfigured → Inactive → Active → Finalized**
  — driven by transitions `configure`, `activate`, `deactivate`, `cleanup`, `shutdown`.
  Each transition calls a hook (`on_configure(const State&)` … `on_shutdown`,
  `on_error`) returning `CallbackReturn::SUCCESS | FAILURE | ERROR`. Convention: allocate
  and declare in `on_configure`, start publishing/timers in `on_activate`, stop in
  `on_deactivate`, free in `on_cleanup`. A `LifecyclePublisher` drops messages unless
  activated.
- Externally the node offers `~/change_state`, `~/get_state`, `~/get_available_transitions`
  services and a `~/transition_event` topic; `ros2 lifecycle set /n configure`,
  `ros2 lifecycle get /n`. Launch: the `LifecycleNode` action plus `EmitEvent(ChangeState)`
  or `RegisterEventHandler(OnStateTransition)`. Nav2's `lifecycle_manager` brings its
  servers up in order and bonds to them.
- Why: deterministic bring-up and recovery — drivers configure (open device, check
  calibration) *before* anything depends on them, come up in dependency order, reconfigure
  without a restart. A mission that must not start until LiDAR, IMU and SLAM report
  *active* is the Cortex-shaped use case.
- **Interviewer's target sentence:** "Managed nodes turn 'is the sensor stack ready?' into
  a queryable state instead of a sleep in a launch file — configure allocates, activate
  publishes, and a supervisor drives the transitions in dependency order."
- **`piros2` line:** untouched — no `LifecycleNode` anywhere. The nearest things are
  hand-rolled: `camera.launch.py`'s `OpaqueFunction` pre-flight (device exists and isn't
  held — names the holder PID) with `on_exit=Shutdown()` on usb_cam, `run-bag`'s 12 s
  sleep and `gate`'s wait for the estimator's `inference provider` log line before
  playing a bag. Those *are* the bring-up-ordering problem; lifecycle is the idiomatic
  answer I would reach for in C++.

## 8. TF2: frames, transform trees, static vs dynamic

- tf2 keeps a time-indexed **tree** of coordinate frames (one parent per frame, no
  loops); every transform is a stamped `parent → child` on `/tf` (dynamic, VOLATILE,
  10 s buffer by default) or `/tf_static` (TRANSIENT_LOCAL, latched, published once).
  Consumers run a `Buffer` + `TransformListener` and call
  `lookup_transform(target, source, time, timeout)`; time `0` / `Time()` means "latest
  common time"; asking for a stamp the buffer can't bracket raises an extrapolation
  exception. tf2 interpolates between stamps.
- **REP-105** frames: `map` (globally consistent, may jump) → `odom` (continuous, drifts)
  → `base_link` (the body); the SLAM node owns `map → odom`, odometry owns
  `odom → base_link`, and the robot's URDF/static publishers own everything below.
  **REP-103** axes: body x forward / y left / z up; *optical* frames z forward / x right /
  y down — the fixed rotation between them is rpy (−90°, 0, −90°), and every projection
  assumes the image header names the optical frame.
- Tools: `tf2_echo a b`, `tf2_tools view_frames` (→ `frames.pdf`), `static_transform_publisher
  --x … --frame-id … --child-frame-id …`, RViz's TF display, `tf2_ros::MessageFilter`.
  Classic mistakes: two publishers for one child (the tree flickers), TF and data on
  different clocks, looking up at `header.stamp` for a transform that arrives late,
  forgetting the optical rotation (clouds lie on their side).
- **Interviewer's target sentence:** "TF is a time-indexed tree with one parent per
  frame; `map → odom → base_link` splits the continuous estimate from the corrected one,
  and lookups are at a stamp, so the two clocks and the arrival order of transforms are
  the whole debugging story."
- **`piros2` line:** the full REP-105 chain is live: `camera.launch.py` publishes
  `base_link → camera_link` (5 cm up, placeholder) and `camera_link →
  camera_optical_frame` (the −90/0/−90 rotation, its comment calls it "pure
  bookkeeping"), verified with `tf2_echo` across the LAN; the fork's `keypoint_detector`
  broadcasts `odom → base_link` (or hands it to `rgbd_odometry` via `publish_tf` — one
  parent per frame) and, since the SLAM build, `map → odom` on a timer from its pose
  graph. Consumers look up *latest* (`lookup_transform(target, source, Time())` in the
  mesher) because the camera's header stamps lag wall clock by a steady 0.73 s — a
  stamp-exact lookup dropped everything; and `cloud_projector` publishes `/points`
  already in `odom` because RViz's own wait-for-TF-at-stamp raced the always-late odom
  transform and flapped. `se3.py` carries the optical conjugation.

## 9. Message and interface definitions

- Three IDL kinds: `.msg` (fields; primitives, `string<=N`, fixed/bounded/unbounded
  arrays, constants `int32 X=1`, defaults), `.srv` (request `---` response), `.action`
  (goal `---` result `---` feedback). Generated by `rosidl` into C++/Python types (plus
  the type-support DDS needs); Iron+ attaches a **type hash** so mismatched definitions
  fail to match instead of decoding garbage — `ros2 topic info -v` shows it.
- Interfaces live in an `ament_cmake` package: `rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Foo.msg" DEPENDENCIES std_msgs)`, `<buildtool_depend>rosidl_default_generators`,
  `<exec_depend>rosidl_default_runtime`, `<member_of_group>rosidl_interface_packages`.
  Convention: a separate `*_msgs`/`*_interfaces` package, so C++ and Python users share
  it and Python packages (which can't run rosidl) can depend on it. `ros2 interface show
  sensor_msgs/msg/PointCloud2` prints the definition.
- Know cold: `std_msgs/Header`, `sensor_msgs` (`Image`, `CompressedImage`, `CameraInfo`
  with `k`/`d`/`p`, `PointCloud2` = a byte blob described by `fields[]` + `point_step`,
  `Imu`), `geometry_msgs/TransformStamped`, `nav_msgs/Odometry`, `visualization_msgs/Marker`,
  `std_srvs/Trigger`.
- **Interviewer's target sentence:** "Interfaces are code-generated contracts with a type
  hash — keep them in their own package, prefer the standard types so tools like RViz
  and rosbag understand your topics, and reach for a custom message only when no
  standard one carries the meaning."
- **`piros2` line:** no custom interface — deliberately. `/keypoints/count` is an `Int32`
  and the mesh is a `Marker` TRIANGLE_LIST because the standard types are what RViz,
  rqt and `just snap` already read. `PointCloud2` is hand-built as a numpy structured
  array whose dtype *is* the wire layout (`point_step` = the record size) — the most
  concrete interface knowledge in the repo. `CameraInfo`'s `k` was zeros in the first
  bag (pre-intrinsics), which is why that bag can't feed mapping.

## 10. Launch files and launch composition

- A Python launch file returns a `LaunchDescription` of actions: `Node` (package,
  executable, name, namespace, `parameters=[yaml_path, {...}]`, `remappings`,
  `arguments`), `ExecuteProcess` (any command), `IncludeLaunchDescription
  (PythonLaunchDescriptionSource(path), launch_arguments={...}.items())`,
  `DeclareLaunchArgument` + `LaunchConfiguration` (resolved lazily — a *substitution*,
  not a string; `ParameterValue(LaunchConfiguration('x'), value_type=int)` to type it),
  `OpaqueFunction` (run Python after arguments resolve), `GroupAction` + `PushRosNamespace`,
  event handlers (`RegisterEventHandler(OnProcessExit(...))`), `Shutdown()`, and
  `ComposableNodeContainer` for §6. XML/YAML launch exist for simple cases.
- Launch composition = a system launch *including* the sensor launch and passing
  arguments through — but an include runs on the machine doing the launching; ROS 2 has no
  remote launch (ROS 1's `machine` tag is gone), so multi-machine bring-up is SSH/systemd.
  `on_exit=Shutdown()` makes one death end the session; by default the rest carry on,
  silent.
- **Interviewer's target sentence:** "Launch is a Python DSL of lazily-resolved
  substitutions — the file describes a process tree, and the two things people forget are
  that includes run locally and that a dead node doesn't stop the launch unless you say so."
- **`piros2` line:** all of the above in anger. `vision.launch.py` includes
  `camera.launch.py` (`IncludeLaunchDescription`); `camera.launch.py` declares
  resolution/framerate/gain arguments, types them with `ParameterValue`, pre-flights the
  device in an `OpaqueFunction`, and puts `on_exit=Shutdown()` on usb_cam;
  `world_mesh.launch.py` runs the two PyPI-dependent nodes as `ExecuteProcess([VENV_PYTHON,
  '-m', 'piros2_perception.depth_estimator', …])` because colcon's hardcoded shebang misses
  the venv (a launch_ros `Node` would exec the system entry point) and deliberately does
  *not* include the camera launch — that would open `/dev/video0` on the dev box. Found and
  fixed: `just world` passed args to the camera launch only, so `odom:=rgbd` never reached
  `world.launch.py`; the fork routes args to both.

## 11. rosbag2, MCAP, record and replay

- `ros2 bag record -o name /a /b` (or `-a`), `ros2 bag play name [--loop --rate r
  --start-offset s --topics … --clock]`, `ros2 bag info`, `ros2 bag convert`. Storage
  plugins: **MCAP** (default since Iron; self-describing — message schemas embedded —
  chunked, indexed, zstd/lz4 per chunk, readable by Foxglove and the `mcap` CLI without
  ROS) vs `sqlite3` (the Humble default). Bags store each topic's **offered QoS** so replay
  matches subscribers; `--qos-profile-overrides-path` when it doesn't.
- `--clock` publishes `/clock`; nodes with `use_sim_time:=true` follow it, which is what
  makes TF lookups and `message_filters` behave on replay. `--loop` restarts stamps — any
  odometry integrating them teleports.
- API: `rosbag2_py.SequentialReader/Writer`, `StorageOptions(uri, storage_id='mcap')`,
  `ConverterOptions('cdr','cdr')`, `TopicMetadata` incl. QoS, `deserialize_message`.
- **Interviewer's target sentence:** "A bag is the reproducible fixture — record the raw
  sensor topics with their QoS, replay with `--clock` and sim time, and you can rerun the
  whole pipeline on a desk; MCAP made that portable outside ROS."
- **`piros2` line:** `just record <secs> <name>` bags `/image_raw/compressed`,
  `/camera_info` and `/tf_static` on the Pi (`timeout -s INT` bounds the recorder;
  24 s ≈ 36 MiB MCAP) and fetches it; `just replay` loops it through `image_transport
  republish` into the edge detector with no Pi; `just run-bag` plays **once** (the
  teleport finding) into the whole world_mesh session. `make_gate_bag.py` re-cuts a sweep
  with `rosbag2_py` — MCAP in, MCAP out, timelines re-stitched so header stamps stay
  continuous, `offered_qos_profiles` copied so `/tf_static` stays latched. Jazzy trap
  recorded in troubleshooting: `image_transport republish` takes transports as
  *parameters* (`-p in_transport:=compressed -p out_transport:=raw`), not positional args.

## 12. RViz, rqt, the ros2 CLI

- **CLI**: `ros2 topic list|echo|hz|bw|delay|info -v|pub`, `ros2 node list|info`,
  `ros2 service list|call`, `ros2 param get|set|dump|load`, `ros2 action`, `ros2 bag`,
  `ros2 launch`, `ros2 run`, `ros2 pkg`, `ros2 interface`, `ros2 component`,
  `ros2 lifecycle`, `ros2 doctor --report`. `ros2 topic echo --qos-reliability
  best_effort` etc. to match a publisher's QoS. The **ROS daemon** caches discovery for the
  CLI — `ros2 daemon stop && ros2 daemon start` after any RMW/domain/DDS change or it
  reports the stale graph.
- **rqt**: `rqt_graph`, `rqt_image_view`, `rqt_plot`, `rqt_console`, `rqt_reconfigure`,
  `rqt_tf_tree`. **RViz2**: a Fixed Frame, per-display topic + QoS, `.rviz` configs
  (`rviz2 -d`), displays for `PointCloud2`, `Marker`, `Image`, `TF`, `Odometry`, `Path`.
- **Interviewer's target sentence:** "`ros2 topic info -v` and `hz` answer 'is anyone
  publishing, at what QoS, is anyone matching' — most 'RViz shows nothing' bugs are a QoS
  mismatch, a stale daemon, or the wrong fixed frame, in that order."
- **`piros2` line:** the CLI found real bugs: `ros2 topic delay /image_raw` exposed the
  0.73 s stamp fault; `ros2 param get /usb_cam frame_id` caught the ignored-parameter
  name; `ros2 topic echo --qos-reliability` proved the BEST_EFFORT zero-delivery;
  `just topics` restarts the daemon before listing. RViz runs from `world_mesh.rviz` with
  Depth3D / LiveMesh / TF / Keyframes displays and image panels; on this Wayland box it
  needs `QT_QPA_PLATFORM=xcb` (OGRE is GLX-only) and, since an NVIDIA userspace/kernel
  mismatch, `LIBGL_ALWAYS_SOFTWARE=1`. And the repo's stance in one line: "the RViz
  window is a viewer, not the evidence" — `just snap` dumps topics and X windows to files.

## 13. Real-time ROS 2 considerations

- Real-time = **bounded latency**, not speed. Ingredients: a `PREEMPT_RT` kernel,
  threads at `SCHED_FIFO` priority, `mlockall` + pre-faulted stacks/heaps, **no dynamic
  allocation, locks, or logging on the RT path** (rclcpp's logging is not RT-safe;
  `std::cout` isn't either), CPU isolation/affinity, and a DDS configured without
  surprises (Cyclone and Fast DDS both have RT-oriented settings; Connext Micro and
  Zenoh-pico for the tiny end).
- What ROS 2 gives you: `rcl` in C; executors on your own RT thread with a custom
  allocator (the TLSF demo), the events executor, intra-process comms, loaned messages,
  `ros2_control`'s 1 kHz-class controller manager (RT loop + non-RT thread, lock-free
  buffers), micro-ROS on MCUs, `rclcpp::WaitSet` for hand-rolled loops. What it does *not*
  give: RT guarantees through the default executor or over a network. The pattern (pendulum
  demo, `ros2_control`): the RT thread steps from a real-time-safe buffer; a normal thread
  does ROS I/O and swaps buffers.
- **Interviewer's target sentence:** "ROS 2 can host a real-time loop but isn't one by
  default — you put the RT step on a `SCHED_FIFO` thread with pre-allocated memory and
  no logging, feed it lock-free from a normal ROS thread, and keep DDS out of the
  critical path; the middleware is for telemetry and commands, not the 1 kHz loop."
- **`piros2` line:** untouched — Python, wall timers, and a Wi-Fi hop rule it out by
  construction, and nothing pretends otherwise. The nearest measured discipline is the
  timing honesty: every cost is measured against one process's own clock (never
  `header.stamp`), the estimator paces the pipeline at `max_rate: 5` so the odom TF stays
  current instead of trailing a queue, and the mesher's refresh moved to another process
  when it was measured stealing 20 s at a time from integration.

## 14. **rclcpp specifically, not just rclpy**

- The mirror of rclpy, but with ownership and templates you must get right:
  ```cpp
  class Detector : public rclcpp::Node {
  public:
    explicit Detector(const rclcpp::NodeOptions & o) : Node("detector", o) {
      declare_parameter<int>("max_features", 500);
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();          // or SensorDataQoS()
      sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        "image_raw/compressed", qos,
        [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) { on_frame(msg); });
      pub_ = create_publisher<std_msgs::msg::Int32>("keypoints/count", qos);
      timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
      srv_ = create_service<std_srvs::srv::Trigger>("~/reset",
        [this](auto req, auto res) { reset(); res->success = true; });
    }
  private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  };
  int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Detector>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
  }
  ```
- Idioms to name: entities are `SharedPtr`s kept alive as members (an unstored
  subscription is destroyed immediately — the classic silent bug); no `shared_from_this()`
  in the constructor; `ConstSharedPtr` vs `UniquePtr` (the intra-process move);
  `RCLCPP_INFO`/`_THROTTLE`; `get_parameter("x").as_int()`; `tf2_ros::Buffer` +
  `TransformListener` + `tf2::doTransform`; `cv_bridge::toCvShare`; `image_transport`
  (C++-only — a Python publisher gets no `/compressed` for free);
  `message_filters::Synchronizer<ExactTime<A,B>>`. Build: `ament_cmake`, `find_package`,
  `ament_target_dependencies` (fine in Jazzy; modern form `target_link_libraries(t
  rclcpp::rclcpp)`), `install(TARGETS …)`, `ament_package()`.
- **Interviewer's target sentence:** "rclcpp is the same graph model with explicit
  ownership: keep every handle alive, take messages by `ConstSharedPtr` (or `UniquePtr`
  when you want intra-process to move them), and build as a component so the node can be
  composed — that's where the C++ pays off."
- **`piros2` line:** honestly untouched as an author. Every C++ that runs in the repo is
  someone else's binary — `usb_cam`, `static_transform_publisher`, `image_transport
  republish`, `rgbd_odometry`/`rtabmap`, `demo_nodes_cpp talker` (`just chatter`, the
  milestone-0 test) — configured, remapped and traced but never written. What transfers
  is everything above the language: I have hit the exact problems rclcpp idioms answer
  (the ownership question is the Python `self.sub = …` I already write; the C++-only
  `image_transport` is why the edge detector hand-publishes `/compressed`). The C++ study
  file (section 8) is where the language itself is worked.

## 15. Multi-machine ROS 2: discovery, domain IDs, network tuning

- Same `ROS_DOMAIN_ID` (0–101 safe on default UDP ports; each domain is 250 ports), same
  `RMW_IMPLEMENTATION`, both hosts on one L2/L3 segment for multicast SPDP — or
  `ROS_STATIC_PEERS=ip` + `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET|LOCALHOST|OFF` (Iron+;
  `ROS_LOCALHOST_ONLY=1` is the deprecated spelling of LOCALHOST) / Fast DDS Discovery
  Server. Then `ros2 daemon` restart. No hostnames, no master URI — DDS is IP-only, so a
  multi-homed host must pin its interface (§3).
- **Time**: TF and `message_filters` compare stamps across machines — run chrony/NTP or a
  PTP grandmaster; a 100 ms skew is a TF extrapolation error.
- **Bandwidth**: raw 720p RGB at 30 fps is ~83 MB/s — never on Wi-Fi; every RELIABLE
  reader is its own **unicast** copy, so *N* readers = *N*× the link; large messages
  fragment and need `net.core.rmem_max` and the vendor's socket buffer raised (Cyclone
  `<SocketReceiveBufferSize>`); APs rate-limit multicast; radio power-save adds latency.
- **Interviewer's target sentence:** "Discovery is the easy part — same domain, same
  RMW, an interface pinned, daemon restarted; the hard parts are that every reader is a
  unicast copy, that big messages fragment on default buffers, and that two clocks are now
  in every stamp."
- **`piros2` line:** the whole repo is two machines — Pi on `wlan0`, dev box on
  `enp6s18`, `ROS_DOMAIN_ID=42`, Cyclone, one definition in Ansible `group_vars`, and a
  measured 4.2–19.1 ms ping. Learnt live: five dev-box readers each pulled their own
  unicast copy of the compressed stream and collapsed the link at 14+ MiB/s (~2 frames/s
  each) → `camera_relay` so it crosses the Wi-Fi once; usb_cam's *raw* topic was being
  pulled by rgbd's remap (2.7 MB frames over Wi-Fi) → fixed; a leaked usb_cam holds
  `/dev/video0` so sessions carry EXIT traps and `just stragglers` sweeps both hosts;
  the Pi's link dies while the OS lives (twice) → the Ansible `wifi` watchdog and
  `ssh -tt` + keepalives so a dead link reaps the camera in ~60 s. Not done: no NTP/chrony
  in the playbook — the two clocks were never explicitly disciplined (the 0.73 s
  camera stamp fault dominated anyway), which is the honest gap to name.

## What to say if asked "how deep is your ROS 2?"

"Deep in `rclpy` and the graph — QoS, TF, launch, rosbag2/MCAP, parameters, services,
multi-machine DDS on Wi-Fi with CycloneDDS pinned, all built and measured in `piros2`, with
a scripted verification layer that replays bags and gates on numbers. The gaps I know I
have: I have never written an `rclcpp` node, a composable component or a lifecycle node,
and I've never run a real-time loop — I know what each is for and where my Python pipeline
would have been better as a composed C++ container. Ask me anything about the DDS layer or
TF; grade me as a beginner on rclcpp." Then stop.
