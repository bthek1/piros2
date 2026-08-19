# Real-time and performance — the study file for section 9 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at
"can hold a technical conversation" depth, the sentence an interviewer is fishing for, and
an honest **`piros2` line**. Section 9 carries no priority note in the syllabus — it is
breadth behind C++ and SLAM — but it is the section where the JD's "real-time C++ on an
embedded Linux box" lives, and it is where `piros2` has the most *measured* material to
draw on: per-node ms/frame figures, a QoS/fragmentation finding, rates measured on the
receiver's own clock, a Wi-Fi saturation diagnosis and fix, GPU-vs-CPU inference numbers.
Per the honest-claim rule: none of it is hard real-time — everything in the repo is Python
under the default single-threaded rclpy executor on stock kernels — so the `piros2` lines
say what was measured and where the real-time discipline was *not* applied, rather than
dressing a soft pipeline up as an RT system.

## Mental model to carry through the whole file

```
            deadline ─────────────────────────────────┐
  event ──► wake-up latency ──► execution time ──► done│
            (scheduler, kernel,  (WCET, cache,       ▼
             other tasks)         allocation, locks) margin or miss
```

Real-time is about **bounding the worst case** of that whole span, not shrinking the
average; performance engineering is about the average and the throughput. The two pull
in different directions (a batch that raises throughput lengthens the worst latency), and
most of the items below are one lever on one of the three boxes: scheduling policy and
PREEMPT_RT attack wake-up latency; no-allocation/no-lock rules and cache locality attack
execution time and its variance; zero-copy attacks the cost of moving data between the
boxes; benchmarking is how you find out which box you're actually in.

## 1. Hard vs soft real time

- **Hard real time:** a missed deadline is a system failure — the value of a late result is
  negative (motor commutation, a flight controller's rate loop, an airbag). You design to
  the worst case with proof or measurement-plus-margin. **Firm:** a late result is useless
  but not catastrophic (a video frame past its display slot, a stale LiDAR scan the
  odometry has already moved past — drop it). **Soft:** a late result still has value,
  degrading with lateness (a map update, a UI, most of a ROS graph).
- **The engineering consequence:** hard RT means bounded everything — bounded execution
  time, bounded memory, no unbounded waits, no dynamic allocation in the loop, a scheduler
  that guarantees the task runs. Soft RT means "fast enough on average and rarely late",
  and you can use Linux, dynamic memory and a DDS. The mistake is treating a soft-RT stack
  as if it were hard (claiming guarantees Linux does not give) *or* treating a hard-RT
  requirement as soft (running the rate loop on the companion computer over a USB link).
- **Where the line falls in a Hovermap-shaped system:** the flight controller (DJI's
  autopilot, or a PX4/ArduPilot MCU) is hard RT and owns rate/attitude; the payload's
  Linux box (Wildcat odometry at ~15 Hz, ~63 ms per loop on a laptop; a Jetson-class board
  in the field) is soft/firm — a late pose is a *worse* pose to feed the velocity loop, and
  the correct response to a very late or degenerate one is to stop trusting it ("SLAM
  slip" cancels position hold), not to pretend it arrived on time.
- **Interviewer's target sentence:** "Hard means a miss is a failure and you design for the
  worst case; soft means late still has value and you design for the distribution. Linux
  plus ROS 2 is soft/firm territory even with PREEMPT_RT — the hard loops belong on the
  flight controller."
- **`piros2` line:** everything is soft. The one *firm* pattern in the repo is
  latest-wins: `KEEP_LAST` depth-1 subscriptions and the dashboard's three latest-wins
  feeds, where a frame that arrives after the next one is simply never looked at — a stale
  frame has zero value, so it is dropped rather than queued. The estimator's `max_rate`
  pacing is the same idea from the source side.

## 2. Determinism, worst-case execution time, jitter

- **Determinism:** the same inputs produce the same outputs *and the same timing*. Timing
  non-determinism on a modern CPU comes from caches, branch predictors, TLBs, DVFS
  (frequency scaling), SMIs on x86, interrupts, page faults, allocator behaviour, lock
  contention, and other tasks — none of it visible in the source.
- **WCET (worst-case execution time):** the bound you schedule against. Static analysis
  (abstract interpretation over the binary and the pipeline model — aiT, Bound-T) is what
  avionics does on simple cores; on a Cortex-A or x86 with caches and out-of-order
  execution it is intractable, so practice is **measurement-based**: run the workload
  under load for a long time, take the high-water mark, add a margin (often 2× or a fixed
  µs headroom), and keep watching. **Never budget on the mean** — a loop with a 3 ms mean
  and a 30 ms tail is a 30 ms loop as far as the deadline is concerned.
- **Jitter:** the variation in period or latency (release jitter = variation in when the
  task starts; response jitter = variation in when it finishes). It matters more than
  constant delay for control (the control file, section 7's §8) and for anything that
  differentiates or integrates with a nominal `dt`. Report it as p50/p99/p99.9/max, and say
  over how many samples — a p99 from 200 samples is two events.
- **Sources you can remove:** frequency scaling and C-states on the RT cores, unpinned
  threads (`isolcpus` + affinity), page faults (`mlockall`, pre-touch), allocation, and a
  working set that falls out of cache (§6).
- **Interviewer's target sentence:** "Real-time is about the tail, not the mean: measure
  WCET as a high-water mark under load, budget against that with margin, and treat jitter
  as a first-class metric — a loop that is usually fast and occasionally 10× late is a
  slow loop."
- **`piros2` line:** the repo reports its costs as ranges from a node's own clock —
  `edge_detector` ~30–45 ms/frame on the Pi (16–20 fps), `keypoint_detector` ~14 ms/frame
  at the 500-feature cap, `cloud_projector` ~12 ms per 33k–57k-point cloud,
  `cloud_mapper` 27–30 ms per 45k-point cloud, `depth_estimator` 72–79 ms/frame in-node on
  the GPU (57 ms of that is inference), `tsdf_mesher` ~52–78 ms/frame integrate on CUDA —
  and one distribution that changed a design: the RViz Depth3D flap was measured as
  per-cloud TF wait **p50 813 ms with 30 % dropped**, and after pacing **p50 15 ms, max
  551 ms, none dropped** (`docs/info/troubleshooting.md`). Nothing here is a WCET; they
  are typical ranges on a bag or a live scene, and the honest word is "measured", not
  "bounded".

## 3. Scheduling policies, priorities, priority inversion

- **Linux policies:** `SCHED_OTHER` (the default fair scheduler — CFS, replaced by EEVDF
  from kernel 6.6 — with `nice` −20…19 as a *weight*, not a guarantee); `SCHED_FIFO` and
  `SCHED_RR` (static priorities 1–99, always pre-empt `SCHED_OTHER`; FIFO runs until it
  blocks or a higher priority arrives, RR time-slices among equals); `SCHED_DEADLINE`
  (earliest-deadline-first with a `runtime/deadline/period` reservation per task — the
  most principled, least used). Setting an RT policy needs `CAP_SYS_NICE` or an `rtprio`
  limit in `limits.conf`; the kernel's **RT throttling** (`sched_rt_runtime_us` = 950000 of
  1000000 µs by default) reserves 5 % for non-RT tasks so a runaway FIFO thread can't wedge
  the box. Verified on the dev box: `kernel.sched_rt_runtime_us = 950000`; and `ulimit -r`
  is 0 on both machines — no user here could take an RT priority without configuration.
- **Priorities as a design:** the highest priority goes to the shortest-deadline task
  (rate-monotonic: shorter period → higher priority; schedulable if utilisation ≤ n(2^(1/n)−1)
  ≈ 69 % for many tasks, or 100 % under EDF). Give IRQ threads and the driver thread the
  sensor depends on a priority *above* the consumer. Don't put everything at 99.
- **Priority inversion:** a high-priority task blocks on a lock held by a low-priority
  task, and a *medium*-priority task pre-empts the low one, so the high task waits for the
  medium — unbounded. Mars Pathfinder 1997: the bus-management task (high) waited on a
  mutex held by the meteorological task (low) while communications (medium) ran; the
  watchdog reset the spacecraft until JPL enabled *priority inheritance* on the VxWorks
  mutex from Earth. **Fixes:** priority inheritance (the holder temporarily inherits the
  waiter's priority — `PTHREAD_PRIO_INHERIT`, all kernel mutexes under PREEMPT_RT),
  priority ceiling (the lock has a fixed high priority any holder gets), or simply not
  sharing a lock across priority levels (lock-free queues, §5).
- **Interviewer's target sentence:** "SCHED_FIFO with a priority per deadline, RT
  throttling and mlockall so you don't wedge or page-fault, and every shared lock either
  priority-inheriting or gone — priority inversion is the Pathfinder bug and it still
  happens."
- **`piros2` line:** not touched — no `chrt`, `taskset` or `nice` anywhere in the repo, every
  node runs `SCHED_OTHER` under a single-threaded rclpy executor, and no callback group or
  `MultiThreadedExecutor` is used, so no priorities and no locks to invert; the mesher's
  10–15 s re-mesh runs on a timer in the *same* thread as its integrate callback and
  simply blocks it while it works. That is the honest state; a real-time port would start
  by naming which callback deserves a thread and a priority.

## 4. PREEMPT_RT and kernel latency

- **What it is:** the Linux real-time patch set — mainlined in **6.12 (late 2024)** after
  ~20 years out of tree — that makes almost the whole kernel pre-emptible: spinlocks
  become sleeping `rt_mutex`es with priority inheritance, interrupt handlers run as
  threads (schedulable, prioritisable), and the remaining non-pre-emptible sections are
  short. Enabled with `CONFIG_PREEMPT_RT`; Ubuntu ships it as the `linux-realtime` flavour
  (via Ubuntu Pro on 24.04). Below it in strength: `CONFIG_PREEMPT` ("low-latency
  desktop", pre-emptible kernel but spinlocks still spin) and `PREEMPT_VOLUNTARY`.
- **What it buys:** bounded scheduling latency for an RT-priority thread — worst-case
  wake-up latency from ~ms on a stock kernel to **tens of µs on x86, ~100–200 µs on a
  Raspberry Pi-class ARM board**, measured with `cyclictest` from `rt-tests`
  (`cyclictest -m -p 90 -i 1000 -h 400` under a stress load, and read the *max*, not the
  average). What it does **not** buy: it makes nothing faster on average (often slightly
  slower), it does not fix a program that allocates, page-faults, or takes contended
  locks in its loop, and it does not make DDS or Python real-time.
- **The layered recipe** for an RT thread on Linux, in order: PREEMPT_RT kernel → thread
  with `SCHED_FIFO` and a chosen priority → `mlockall` and pre-touched stack/heap → CPU
  affinity away from housekeeping cores → no allocation, no blocking syscalls, no
  contended locks in the loop → verify with `cyclictest`-style measurement of *your*
  loop. Skip any layer and the guarantee is gone.
- **Interviewer's target sentence:** "PREEMPT_RT bounds the kernel's scheduling latency —
  threaded IRQs, sleeping spinlocks with priority inheritance — so an RT-priority thread
  wakes within tens of microseconds; it's mainline since 6.12, you measure it with
  cyclictest, and it's necessary but nowhere near sufficient — the application still has
  to behave."
- **`piros2` line:** neither machine runs it. Verified over SSH while writing this: the Pi
  is `6.8.0-raspi` with `CONFIG_PREEMPT=y` (the low-latency desktop model, not `_RT`;
  `/sys/kernel/realtime` absent), the dev box is a stock `7.0.0-generic`. Nothing in the
  repo needs it — the shortest deadline in the system is a 16 ms camera poll, and the
  camera itself already jitters by design (auto-exposure varies the frame interval).

## 5. Why allocation, locks and exceptions are avoided in hot paths

- **Allocation:** `malloc`/`new` are non-deterministic in time — a fast-path bump today, a
  lock, a free-list walk, an `mmap`/`brk` syscall and a page fault tomorrow — and
  fragmentation makes a long-running process's heap behave differently at hour 40 than at
  minute 1. Rules: allocate everything at start-up (pre-sized vectors, object pools,
  ring buffers), never in the loop; if you must, use a bounded-time allocator (TLSF —
  O(1) alloc/free — is what the ROS 2 real-time work uses through `rclcpp`'s
  allocator template parameters and the `realtime_tools` package's helpers); pre-fault and
  `mlockall` so a first touch is not a page fault mid-loop; and watch for hidden
  allocations — `std::string`, `std::function` captures over the small-buffer size,
  `std::vector` growth, logging with formatting, `std::shared_ptr` control blocks, DDS
  serialisation buffers.
- **Locks:** a mutex is a potential *unbounded* wait plus a priority-inversion hazard
  (§3) plus a cache-line ping-pong. Hot paths use single-producer/single-consumer
  lock-free ring buffers, atomics with the right memory order, seqlocks for
  "publish latest state", or message passing with the wait on the *consumer* side only
  (`realtime_tools::RealtimeBuffer` / `RealtimePublisher` exist precisely to hand data
  from an RT control thread to a non-RT ROS publisher without the RT side ever blocking).
- **Exceptions:** C++'s zero-cost model is free when nothing throws and *unboundedly*
  expensive when something does — the unwinder walks tables, may take a global lock, and
  allocates the exception object; and the presence of exceptions makes WCET reasoning
  across the call graph impossible in practice. So: `noexcept` on hot-path functions,
  error codes / `std::expected` (C++23) / `std::optional` inside the loop, exceptions only
  at the boundaries where a throw means "abandon this frame". Same category: no I/O, no
  `printf`, no blocking syscalls in the loop — log to a lock-free ring and drain it elsewhere.
- **Interviewer's target sentence:** "No allocation, no locks, no exceptions, no I/O in the
  loop — allocate up front, hand data across threads with lock-free buffers, and keep
  exceptions for the boundary — because each of those is an unbounded wait hiding in a
  one-line call."
- **`piros2` line:** the anti-pattern, honestly: every node is Python, allocating on every
  callback (numpy temporaries, `cv2.imdecode`, a fresh `PointCloud2` per cloud). The one
  place the *shape* of the rule appears is `cloud_projector`, which builds the
  `PointCloud2` as a numpy structured array whose dtype *is* the wire format
  (`POINT_DTYPE` x/y/z/rgb float32, offsets 0/4/8/12) — one buffer, no per-point Python
  objects — and `edge_detector`'s note that `cv_bridge` hands back a numpy view where it
  can. Measured, not designed: 12 ms for a 33k–57k-point cloud.

## 6. Cache locality and data-oriented design

- **The numbers to keep in your head:** L1 hit ~1 ns (4 cycles), L2 ~4 ns, L3 ~10–20 ns,
  DRAM ~80–100 ns; a cache line is 64 bytes; a branch mispredict ~15–20 cycles; a TLB miss
  another memory walk. A cache-missing loop is 50–100× slower per element than a
  streaming one, and *that* — not the arithmetic — is where most robotics inner loops
  spend their time (point clouds, voxel grids, KD-tree traversal, feature matching).
- **Locality:** *spatial* — touch memory in order so the hardware prefetcher streams it;
  *temporal* — reuse what is already in cache before moving on. **AoS vs SoA:** an array of
  `struct Point {x,y,z,rgb,normal…}` drags every field through cache when you only need
  `z`; a struct of arrays (`xs[]`, `ys[]`, `zs[]`) streams the one you need and vectorises
  (SIMD wants SoA). PCL's `PointXYZ` is padded to 16 bytes so it aligns for SSE — a
  deliberate AoS compromise. **False sharing:** two threads writing different variables
  on the same 64-byte line serialise on it — pad or align hot per-thread data.
  **Hot/cold splitting:** keep the fields the loop touches together and the rest elsewhere.
- **Data-oriented design** (Acton, "Data-Oriented Design and C++", CppCon 2014): design
  around *the data and its transforms* — what is the shape, how much of it, what is the
  access pattern — instead of around object hierarchies with virtual calls per element.
  Entity-component systems in games; in robotics, "a point cloud is a big flat array you
  run passes over", voxel hashing with contiguous blocks (Open3D's `VoxelBlockGrid`,
  Wildcat's surfels stored per voxel), and Eigen's fixed-size matrices sitting on the
  stack.
- **Interviewer's target sentence:** "Memory access dominates: keep the working set
  contiguous and streamed, prefer SoA where you vectorise, watch false sharing across
  threads, and design around what the data looks like — that's why point-cloud and voxel
  code lives in flat arrays and hash-addressed blocks, not per-point objects."
- **`piros2` line:** the design is numpy-flat where it counts — the projector's structured
  array; `piros2_world`'s mapper, whose weighted-average rewrite (2026-08-10) keeps means
  and weights in preallocated numpy arrays and collapses each cloud with `np.unique` +
  `np.add.at` — measured 27–30 ms per realistic 45k-point cloud against ~25 ms for the old
  loop, 74 ms in an all-distinct-voxel stress case, so the rewrite bought fusion semantics,
  not speed; and the TSDF in Open3D's hash-addressed `VoxelBlockGrid` on CUDA. No cache profiling was ever done; the honest
  claim is "vectorised numpy and a GPU grid", not "cache-tuned".

## 7. Latency vs throughput trade-offs

- **Definitions:** latency = time from one input to its output; throughput = inputs per
  second. In a pipeline, throughput is set by the *slowest* stage, latency by the *sum* of
  the stages plus every queue between them. Little's law, `L = λ·W`: with arrival rate λ
  and average time-in-system W, the average number in the system is L — so a queue that is
  usually full is a queue adding `L/λ` of latency, on purpose or not.
- **The trade:** batching (GPU inference on 8 frames, sending one big DDS sample instead of
  many small ones, coalescing writes) raises throughput and lengthens latency; deep queues
  smooth bursts (throughput) at the cost of stale data (latency); tighter queues and
  drop-oldest policies protect latency and lose data. Control and odometry are
  latency-bound — the *freshest* sample matters and a queued one is worse than a dropped
  one; mapping and logging are throughput-bound — every sample matters and lateness is
  fine.
- **Where the tail comes from:** back-pressure that isn't — a fast producer feeding a slow
  consumer through an unbounded or deep queue does not slow down, it just makes the
  consumer permanently late by the queue depth. The disciplined answers: bound the queue,
  drop the oldest (`KEEP_LAST` small depth in ROS 2), pace the *producer* to the
  consumer's rate, or split the work so the latency-critical part runs first and the
  heavy part is deferred (integrate now, re-mesh on a timer).
- **Interviewer's target sentence:** "Throughput is the slowest stage, latency is the sum
  plus the queues; batching and deep queues buy throughput with latency, so for control
  and odometry you keep queues shallow, drop old data, and pace the source — a queued
  sample is worse than a dropped one."
- **`piros2` line:** this item *was* the 2026-08-16 live-debug session. Symptom: RViz's
  Depth3D flickered between a TF error and rendering. Cause, measured: the unpaced depth
  pipeline (~10 Hz) outran `rgbd_odometry` (~4–5 Hz), the odom TF stamps trailed the
  clouds by ~0.8 s median while rgbd chewed queue backlog. **Shrinking rgbd's sync queues
  first made it worse** — under bursty processing the two topics dropped different stamps
  and exact sync starved (the same finding as the mapping launch's 5-vs-30 sync queue:
  0–6 pairings versus a deterministic 24). The fix: pace the *source* — `max_rate: 5` on
  the estimator (what rgbd sustains; the GPU also does half the work) plus a 10-deep
  display queue; per-cloud TF wait p50 813 ms/30 % dropped → p50 15 ms/none. And the
  mesher's split — integrate every synced frame (~52–78 ms), re-mesh every 10–15 s on a
  timer — is the "latency-critical first, heavy work deferred" shape.

## 8. Zero-copy and shared memory

- **Why copies cost:** a 1280×720 RGB8 image is 2.7 MB; at 30 fps that is ~83 MB/s *per
  copy*, and a default ROS 2 pub/sub path copies more than once (serialise to CDR, the
  transport's buffer, the socket, deserialise per subscriber) — and DDS sends each
  RELIABLE reader its *own* unicast copy over the network unless multicast is in play.
  On an embedded ARM box that is a real fraction of memory bandwidth.
- **ROS 2's tools, in order of how much they change:**
  - **Intra-process communication** — publisher and subscriber in the *same process*
    (composed nodes in a `component_container`, `use_intra_process_comms(true)`): a
    `unique_ptr` message is *moved* to a single subscriber, no serialisation, no copy;
    with several subscribers, one shared copy. This is the main practical reason
    composition exists.
  - **Loaned messages** — `publisher->borrow_loaned_message()` lets the middleware hand
    you a buffer it owns; with a shared-memory transport (CycloneDDS + iceoryx, Fast DDS
    data-sharing) and a *fixed-size* (POD, bounded) message type the data is written once
    into shared memory and readers map it — true zero-copy across processes. Variable-size
    types (`PointCloud2` with an unbounded `data` field, `Image`) fall back to a copy into
    the shared segment; that copy still beats the loopback socket path.
  - **Shared-memory transports** as a whole (Fast DDS SHM is on by default for same-host
    peers; CycloneDDS needs the iceoryx plugin) — cheaper than UDP loopback even when
    they copy. Docker's `ipc: host` note in `docs/info/setup.md` is this requirement.
- **The rule:** for large, high-rate data — images, LiDAR clouds — keep producer and
  consumer in one process (compose), or on one host with a shared-memory transport, and
  design the message fixed-size if you want the real zero-copy path. Across a link, send
  the smallest representation (compressed images, downsampled clouds, Wildcat's
  100–170 KB submaps over the mesh radio) and send it *once*.
- **Interviewer's target sentence:** "Compose nodes so intra-process moves the pointer,
  use loaned messages with a shared-memory transport for fixed-size types to go
  zero-copy across processes, and never let a large topic cross a link once per
  subscriber — the copies, not the compute, are what saturate an embedded box."
- **`piros2` line:** none of the zero-copy machinery is used — every node is its own
  Python process, `rclpy` has no intra-process path, and the message types are
  variable-size. What the repo *did* do is measure the per-subscriber copy cost the hard
  way and route around it: five dev-box readers of `/image_raw/compressed` = five unicast
  copies over the Pi's Wi-Fi, each *completing* ~2 frames/s while the link burned
  14+ MiB/s (one reader: full rate at ~1.3 MiB/s); plus a raw-topic collision where
  `rgbd_odometry`'s `rgb/image` had been remapped to `/image_raw` and was silently pulling
  2.7 MB raw frames across the Wi-Fi. `camera_relay` (in `piros2_world_mesh`) is the fix:
  the stream crosses the link *once* and is fanned out on loopback as
  `/camera_relay/compressed`; the estimator republishes the exact frame it inferred on as
  `/depth/rgb` so exact sync pairs every depth frame. Diagnostic that needs no ROS:
  `ssh pi 'cat /sys/class/net/wlan0/statistics/tx_bytes'` twice, 10 s apart — one clean
  compressed stream is ~1–3 MiB/s. And the older finding that sits underneath: a 2.7 MB
  raw frame fragments into ~1800 UDP datagrams, at least one drops with default socket
  buffers, and BEST_EFFORT never retransmits — so a best-effort subscriber to `/image_raw`
  received **zero** frames even on loopback while RELIABLE worked instantly
  (`edge_detector.py`'s `BIG_FRAME_QOS`; the "sensor data = BEST_EFFORT" rule assumes
  small messages).

## 9. Benchmarking methodology

- **Measure the right span with the right clock:** decide what you are timing (wake-up
  latency? end-to-end? one function?) and use a monotonic clock read at both ends in the
  same process (`std::chrono::steady_clock`, `clock_gettime(CLOCK_MONOTONIC)`); never
  subtract a timestamp written by another clock or another device unless you have proven
  they agree.
- **Warm up, then sample enough:** first iterations pay for page faults, cache fill,
  JIT/CUDA context creation, allocator growth — discard them. Then take enough samples to
  see the tail (thousands for a p99, more for a p99.9) and report the **distribution**
  (p50/p95/p99/max, or a histogram), never a lone mean.
- **Control the conditions and state them:** CPU governor (`performance`), frequency
  pinned or at least noted, thermal state (a Pi throttles), other load, power supply;
  input data fixed (a recorded bag, a fixed dataset) so runs are repeatable; the build
  (`-O2/-O3`, LTO, sanitizers *off*); the config that matters (resolution, rate, GPU vs
  CPU). A number without its conditions is not a measurement.
- **Isolate one variable at a time; beware the observer:** profilers and tracers cost;
  printing in the loop costs; a viewer on the same box costs. Tools by layer:
  `cyclictest` for scheduling latency; `perf stat`/`perf record` for where the cycles go
  (cache-miss counters answer §6); `ros2_tracing` (LTTng) for callback and executor
  latency inside a ROS 2 graph; `ros2 topic hz/bw/delay` for rates and transport;
  Google Benchmark / `hyperfine` for micro and whole-command timing; `valgrind --tool=
  massif` or heaptrack for allocation in the loop.
- **Make it repeatable and able to fail:** the useful benchmark is a script that runs the
  same input, prints the numbers, and exits nonzero on regression — a gate, not a screenshot.
- **Interviewer's target sentence:** "Warm up, use a monotonic clock in one process,
  report the distribution not the mean, state the conditions, change one thing at a time,
  and put the measurement in a script that can fail — otherwise it's an anecdote."
- **`piros2` line:** the repo's measurement discipline is its strongest real-time-adjacent
  claim, and every rule above was learned from a specific mistake:
  - *Right clock:* `/image_raw` header stamps lag wall clock by a steady ~0.73 s (a
    UVC/driver fault, `ros2 topic delay` shows it); the first edge detector gated on
    stamp age and dropped 100 % of frames. Since then every span is between two reads of
    the receiving node's own clock (`entry = self.get_clock().now()` in `on_frame`;
    `time.monotonic()` in the mesher), and the dashboard's rates and STALE lines are
    computed from receipt times only (`rates()` and `stats_lines()` are pure functions
    tested without ROS).
  - *Warm-up and conditions:* the CPU depth figure is "280–305 ms per frame steady, ~1.3 s
    first-inference warm-up" and the GPU figure "~57 ms inference, 72–79 ms/frame in-node
    against `bags/static1`" — the input bag is named, and the CPU→GPU move
    (`onnxruntime-gpu[cuda,cudnn]` on a GTX 1660 SUPER, ~13 fps vs ~3 fps) is only
    trustworthy because the node logs the winning provider — the CUDA path *silently*
    falls back to CPU if `preload_dlls()` isn't called. Frame rate is never quoted without
    its exposure mode (18–21 fps with `exposure_dynamic_framerate` on; 30.00 fixed;
    42–60 fps under the `camera-reset` baseline).
  - *Observer effect:* RViz's own load pushed `rgbd_odometry`'s `delay=` to 1.6–2.6 s and
    brought the Depth3D flap back after the transport fix; the structural answer was to
    publish clouds already in the fixed frame so RViz never waits on TF.
  - *Repeatable, failing checks:* `just gate flick|occlude` replay gate bags and exit 0/1
    on a pose-error threshold; `just run-bag` runs the whole session from a bag with no
    Pi. Not speed benchmarks — but the same shape.

## What to say if asked "have you done real-time work?"

"Not hard real-time on this project — `piros2` is Python on stock kernels, single-threaded
executors, no RT priorities, and I've checked that's true rather than assumed it. What I
have is the measurement half: per-node costs off the node's own monotonic clock, a
timestamp fault that broke a freshness gate, a p50/p99-style TF-wait distribution that
drove a pacing fix, a QoS fragmentation finding where best-effort delivered zero frames,
and a Wi-Fi saturation diagnosis from the interface's TX counter that led to
copy-once-fan-out. I know the RT recipe — PREEMPT_RT, SCHED_FIFO, mlockall, no
allocation/locks/exceptions in the loop, cyclictest — and where in a Cortex-like stack it
belongs versus where the flight controller owns the hard loop." Then stop.
