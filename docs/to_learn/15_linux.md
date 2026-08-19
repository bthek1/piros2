# Linux — the study file for section 15 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist bullet: the concept, the sentence an
interviewer is fishing for, and an honest **`piros2` line**.

This section is breadth on paper and depth in practice: a field-robotics company runs
embedded Linux on the payload and Linux on the developer's desk, and almost every hard bug
in this repo turned out to be a *systems* bug rather than an algorithm bug — a device held
by a leaked process, a UDP datagram lost in fragmentation, an environment file not read by
a non-interactive shell, a driver whose userspace and kernel module disagreed. Those are
the stories to tell. Reading a section does not tick its box.

## Mental model: where a robotics bug actually lives

```
your node ── process/thread scheduling ── memory ── syscalls
     │              │                        │         │
     │              │                        │         └─ devices (/dev/video0, tty, CAN)
     │              │                        └─ page cache, mmap, OOM killer
     │              └─ priorities, cgroups, CPU affinity, RT
     └─ sockets ── UDP fragmentation ── MTU ── multicast ── the network
```

The lesson `piros2` learned repeatedly: **the layer below the one you're debugging is
usually the one that's lying.** A node that "receives no messages" was a UDP fragmentation
problem; a camera that "won't open" was a process nobody knew was running; a GUI that
"crashes on start" was a driver version mismatch.

## 1. Processes, threads, scheduling

- **Process vs thread:** a process owns an address space, file descriptors and a PID; threads
  share all of that and have their own stack and registers. Linux implements both with
  `clone()` and schedules *threads* (tasks). Fork semantics worth knowing: `fork()` is
  copy-on-write, and forking a multi-threaded process is a well-known way to inherit a locked
  mutex nobody will unlock.
- **Scheduling classes:** `SCHED_OTHER` (CFS — the default, fair-share, `nice` from −20 to 19
  weighting the share), and the real-time classes `SCHED_FIFO` and `SCHED_RR` (priorities
  1–99, *always* preempting normal tasks — an RT task in a busy loop can lock a core, which is
  what `sched_rt_runtime_us` exists to prevent). `SCHED_DEADLINE` gives EDF with explicit
  runtime/period. `chrt` sets them; `taskset` pins CPU affinity; cgroups (v2) bound CPU, memory
  and I/O per group — which is how you stop a mesher starving a control loop.
- **What actually shows up in robotics:** priority inversion (see
  [09_real-time-and-performance.md](09_real-time-and-performance.md)), a node with more
  threads than cores thrashing, and CPU affinity used to isolate a hot loop from the
  interrupt-handling core.
- **Inspection:** `ps -eLo pid,tid,psr,pri,rtprio,comm` shows threads with their CPU and RT
  priority; `top -H`; `/proc/<pid>/status` and `/proc/<pid>/task/`; `pidstat`.
- **Interviewer's target sentence:** "Linux schedules threads; CFS is fair-share with nice as
  a weight, and FIFO/RR preempt it entirely — so a hot loop gets an RT priority, a CPU pin and
  a cgroup bound, and you check with `chrt` and `ps -eLo … rtprio` rather than hoping."
- **`piros2` line:** no RT scheduling here — but the repo has a hard-won lesson about
  **process trees**: justfile background jobs are `bash -lc` wrappers, so `kill %N` kills the
  wrapper and *orphans* the actual `ros2 run` grandchildren, which then sit silent until a
  live stream feeds their subscriptions again (it bit twice in one day). The standing
  convention is that every session recipe's `trap … EXIT` must `pkill -f` the node patterns,
  and `just stragglers` sweeps both machines. There is a matching trap for me as an assistant,
  recorded in CLAUDE.md: never put a node's source path on the same command line as a session
  recipe, because the EXIT trap's `pkill -f` will match *my own shell* — exit 144.

## 2. Memory management, page cache, OOM

- **Virtual memory:** each process sees a virtual address space mapped to physical pages by
  the MMU; pages fault in on demand. `RSS` is resident (physically present) memory, `VSZ` is
  the whole address space (meaningless as a "usage" number), and **shared pages are counted in
  every process's RSS**, which is why summing RSS overcounts. `PSS` (in `/proc/<pid>/smaps`)
  is the honest per-process share.
- **Page cache:** free RAM is used to cache file contents; "low free memory" on Linux is
  normal and healthy — read `available`, not `free`, in `free -h`. Writes go to dirty pages
  and are flushed by writeback; `fsync` forces it. For robotics this matters when bagging: a
  burst of writes fills dirty pages and stalls when writeback can't keep up.
- **Overcommit and the OOM killer:** Linux lets processes allocate more than exists
  (`vm.overcommit_memory`), so `malloc` rarely fails; instead, when memory actually runs out
  the **OOM killer** picks a victim by `oom_score` (roughly, biggest memory hog, adjustable
  via `oom_score_adj`) and kills it. Symptom: a node vanishes with no log and no core dump —
  the evidence is in `dmesg`/`journalctl -k` ("Out of memory: Killed process …"). On an
  embedded target this is a *design* consideration: no swap, a small RAM budget, and a
  deliberate choice about who is expendable.
- **Other tools of the trade:** `mlockall()` to pin pages for real-time (no page faults in the
  hot loop), huge pages for large working sets, and `mmap` for zero-copy file access.
- **Interviewer's target sentence:** "Read `available` not `free` because page cache is doing
  its job; Linux overcommits, so out-of-memory shows up as the OOM killer silently removing a
  process — you find it in `dmesg` — and on an embedded target you set `oom_score_adj`
  deliberately and `mlockall` the real-time path."
- **`piros2` line:** the memory pressure in this repo is real but bounded: the TSDF
  `VoxelBlockGrid` at 1.5 cm and the mesh extraction are the heavy consumers (marching cubes
  **OOMs on the 6 GB GPU below ~8 mm voxels and falls back to CPU** in the offline pipeline —
  a measured, documented boundary), and the live mesher hard-caps triangles rather than
  growing without limit. The voxel map is likewise array-backed with a hard cap. Those are
  deliberate "bound the thing that can grow" decisions rather than tuning after an OOM.

## 3. Filesystems and I/O

- **The basics that matter:** everything is a file descriptor; buffered I/O goes through the
  page cache while `O_DIRECT` bypasses it; `fsync`/`fdatasync` are the only durability
  guarantees; and a rename within a filesystem is atomic — which is the standard trick for
  "write to a temp file and swap it in" so a reader never sees a half-written file.
- **Filesystems in embedded reality:** ext4 (journalled, the default), F2FS (flash-aware),
  overlayfs (containers and read-only rootfs with a writable layer), tmpfs (RAM), and
  read-only root with a small read-write partition — the standard robustness pattern for
  devices that lose power abruptly. SD cards and eMMC wear out; write amplification is real,
  and logging at high rate to flash is a genuine hardware-lifetime decision.
- **I/O performance:** `iostat`/`iotop` for throughput and wait; `%iowait` in `top`; queue
  depth and scheduler (`mq-deadline`, `bfq`, `none` for NVMe). A robot writing a bag at
  50 MB/s to an SD card will hit the card's sustained-write limit long before the CPU notices.
- **Identifying devices stably:** device node names (`/dev/sda`, `/dev/video0`) are
  enumeration-order-dependent and therefore *not stable*; use `/dev/disk/by-uuid`,
  `by-partuuid`, `/dev/v4l/by-id/…` or udev rules with a persistent attribute.
- **Interviewer's target sentence:** "Write-temp-then-rename for atomicity, `fsync` when you
  actually need durability, read-only root with a writable overlay for power-loss robustness,
  and never trust enumeration-order device names — use by-id or a udev rule."
- **`piros2` line:** two concrete instances. **Storage identity:** the Pi's SD card is pinned
  by **PARTUUID** (`5ec0ffee-01`/`-02`) in both `cmdline.txt` and `/etc/fstab`, deliberately
  *not* by label — because Ubuntu's Pi image labels every copy `system-boot`/`writable`, so a
  second copy of the image collides on label, filesystem UUID *and* PARTUUID; the standing
  instruction is not to "simplify" these back to `LABEL=`. **Device identity:** the camera is
  referenced through its `/dev/v4l/by-id` symlink — but `usb_cam` mangles the symlink, so it
  has to be passed through `readlink -f` first. And `/dev/video1` is not a second camera at
  all: it is the C922's UVC *metadata* node, capture is `/dev/video0` only.

## 4. Networking: sockets, UDP fragmentation, MTU, multicast

- **The stack in one line:** application → socket → transport (TCP reliable ordered stream /
  UDP unreliable datagram) → IP → link. DDS — hence ROS 2 — runs on **UDP** by default, which
  makes the next three bullets robotics-critical rather than trivia.
- **MTU and fragmentation:** Ethernet's MTU is typically 1500 bytes, so an IP datagram larger
  than that is **fragmented** into MTU-sized pieces which are reassembled by the receiver. The
  killer property: **if any one fragment is lost, the entire datagram is discarded** — there is
  no partial delivery and no per-fragment retransmission. So a large UDP message's effective
  loss probability is `1 − (1 − p)^n` for n fragments, which for large n approaches certainty.
  Wi-Fi makes p non-trivial. Mitigations: keep messages small (compress!), raise the socket
  and kernel buffers (`net.core.rmem_max`, `net.ipv4.ipfrag_high_thresh`), use jumbo frames on
  a controlled wired link, or use a transport with its own fragmentation and NAK-based
  retransmission — which is exactly what DDS RELIABLE does.
- **Multicast:** one sender, many receivers, using IGMP for group membership. DDS uses it for
  **discovery** by default. Failure modes: switches that don't do IGMP snooping (flooding or
  dropping), Wi-Fi APs that handle multicast badly (often sending it at the lowest basic
  rate), routers that don't forward it, and interfaces that silently don't join the group.
- **Interface selection is the classic multi-homed bug:** a host with several interfaces may
  bind and advertise an address the peer cannot route to — VPN interfaces are the nastiest
  because they look routable and aren't.
- **Tools:** `ip addr`/`ip route`, `ss -tulpn`, `tcpdump`/Wireshark, `iperf3`, `ping -M do -s`
  to find the path MTU, `ethtool`.
- **Interviewer's target sentence:** "DDS is UDP, so MTU matters: one lost fragment discards
  the whole datagram, which is why big images over Wi-Fi appear to vanish entirely rather than
  degrade — you compress, you size the buffers, and you pin the interface so discovery doesn't
  advertise a VPN address the peer can't route to."
- **`piros2` line:** **both of these bit this repo, with numbers.** (1) **Fragmentation:** a
  1280×720 RGB8 image is ~2.7 MB, which fragments into roughly **1800 UDP datagrams** — over
  Wi-Fi one always drops, so **BEST_EFFORT receives zero large frames** while RELIABLE
  reassembles them; the vision node subscribes RELIABLE/KEEP_LAST-1 on purpose, and the
  standing rule is never to stream raw images across the LAN (1280×720 RGB8 at 30 fps is
  ~83 MB/s) — use `image_transport` compressed topics. (2) **Interface pinning:** the dev box
  has three Docker bridges (172.17–172.19), `tailscale0` and a WireGuard interface named
  `laptop` at 10.8.0.3; DDS will happily pick one and advertise an address the Pi cannot
  route to, so CycloneDDS is pinned via `CYCLONEDDS_URI` to `enp6s18` on the dev box and
  `wlan0` on the Pi. (3) The **bandwidth** lesson, measured live: five dev-box readers each
  pulling their own unicast copy of the compressed stream plus a node accidentally subscribed
  to raw `/image_raw` collapsed the link to ~2 frames/s each at 14+ MiB/s; the fix was a
  `camera_relay` so the stream crosses the Wi-Fi **once** and is fanned out locally.

## 5. Network tuning for DDS

- **The knobs that actually matter**, roughly in order: kernel socket buffers
  (`net.core.rmem_max`, `wmem_max`, and `rmem_default`) — DDS wants these in the megabytes
  because a burst of fragments must be buffered before the reader drains them; IP fragment
  reassembly thresholds (`net.ipv4.ipfrag_high_thresh`, `ipfrag_time`); the transmit queue and
  txqueuelen; and the DDS implementation's own buffer settings (Cyclone's
  `MaxMessageSize`/`FragmentSize`, receive buffer, and whether it uses multicast for discovery).
  Cyclone will log a warning when the OS buffer is smaller than it asked for — worth reading.
- **Discovery configuration:** for a known set of hosts, disabling multicast discovery and
  listing peers explicitly (Cyclone's `Peers`, or ROS 2's discovery server with Fast DDS) is
  more robust than multicast over Wi-Fi, and it removes the "works on the bench, fails on the
  site network" class of bug.
- **Domain isolation:** `ROS_DOMAIN_ID` maps to UDP port ranges, so two projects on the same
  LAN with the same domain will discover each other's nodes — usually the cause of "there's a
  ghost node in my graph".
- **The environment trap unique to ROS 2:** the daemon caches discovery state, so after
  changing any `ROS_*` or DDS variable you must `ros2 daemon stop && ros2 daemon start` or the
  CLI reports a stale view — which masks fixes that actually worked.
- **Interviewer's target sentence:** "Raise `rmem_max` and the fragment reassembly thresholds,
  pin the interface, and prefer explicit peers over multicast discovery on a flaky or
  site-managed network — then restart the ROS daemon, because it caches discovery and will
  happily show you the old world."
- **`piros2` line:** all of this is configuration the repo actually carries: `ROS_DOMAIN_ID=42`
  (non-default on purpose — 0 is shared with every other project on the LAN),
  `ROS_LOCALHOST_ONLY=0` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` identical on both hosts,
  defined once in Ansible `group_vars` rather than in a `.bashrc`, plus the `CYCLONEDDS_URI`
  interface pinning above. The daemon-restart rule is written down precisely because a stale
  daemon view once masked a working fix.

## 6. systemd services and journald

- **Units:** `service`, `timer` (cron's replacement, with `OnBootSec`/`OnUnitActiveSec`,
  `Persistent=true` for missed runs, and randomised delay), `socket` (activation on connect),
  `target` (grouping), `path`, `mount`. Dependencies are declared (`After=`, `Requires=`,
  `Wants=`) rather than sequenced by number, which is the actual improvement over init scripts.
- **Service behaviour that robotics cares about:** `Restart=on-failure|always` with
  `RestartSec` and a burst limit (`StartLimitIntervalSec`/`StartLimitBurst`) so a crash-looping
  node doesn't hammer the machine; `WatchdogSec` with `sd_notify` so a *hung* process is killed
  rather than left alive; resource limits and hardening (`MemoryMax`, `CPUQuota`,
  `PrivateTmp`, capability bounding); and `Type=notify` for services that must announce
  readiness.
- **journald:** structured, indexed logs with metadata — `journalctl -u <unit>`,
  `-t <identifier>` for a syslog tag, `-b` for this boot and **`-b -1` for the previous boot**
  (the single most useful flag after a crash or a reboot), `-f` to follow, `-p err` by
  priority, `--since`/`--until`, `-o json` for machine reading. Persistence must be enabled
  (`Storage=persistent`) or logs die with the boot; on flash, `SystemMaxUse` bounds the wear.
- **Interviewer's target sentence:** "systemd units with `Restart=on-failure`, a burst limit
  so it can't crash-loop, and `WatchdogSec` with `sd_notify` so a hang is caught as well as a
  crash — and `journalctl -b -1` is where you look after an unexplained reboot."
- **`piros2` line:** the Wi-Fi watchdog is a real systemd design and worth walking through:
  the Ansible `wifi` role installs a service on a **60-second timer** that runs an escalation
  ladder — reassociate → reload the `brcmfmac` driver → reboot, with the reboot **guarded by a
  10-minute uptime floor and a 1-hour cooldown** precisely so it cannot boot-loop — and
  **`journalctl -t wifi-watchdog` is its flight recorder**, with thresholds in
  `group_vars/robot.yml`. It was drilled rather than assumed: the drill reproduced the AP's
  `status_code=16` rejection and recovered unaided at T+426 s via the driver-reload rung,
  proving reassociation alone could not clear that failure. The related diagnostic rule in
  CLAUDE.md is pure journald: **never diagnose an unreachable Pi as "crashed" without
  evidence — ping first, and after recovery read `journalctl -b -1`, because the truth is in
  the previous boot** (twice the link died while the OS ran on undisturbed: journal alive,
  load ~0, no undervoltage). The Pi's sshd also runs `ClientAlive` so a dead link reaps the
  camera session in ~60 s instead of orphaning it.

## 7. Performance tools: top, htop, perf, strace, ltrace, iotop

- **The triage order:** is it CPU, memory, I/O, or waiting? `top`/`htop` (per-core load,
  `%wa` for I/O wait, `%sy` for kernel time — high system time means syscalls or contention),
  `free -h`/`vmstat`, `iostat`/`iotop`, then network (`ss`, `iftop`, `nload`).
- **`perf`** is the one that separates people who profile from people who guess:
  `perf top` for live hotspots, `perf record -g ./prog && perf report` for a call-graph
  profile (needs frame pointers or DWARF — `--call-graph dwarf` when the binary is optimised),
  `perf stat` for hardware counters (instructions, IPC, **cache misses**, branch misses),
  and `perf sched`/`perf trace` for scheduling and syscall latency. Flame graphs are just
  `perf script` folded and rendered — the standard way to make a profile readable.
- **`strace`** traces syscalls (`-c` for a summary count and time per syscall, `-f` to follow
  forks, `-e trace=openat` to filter) and is the fastest way to answer "what file/device/socket
  is it actually touching, and what is it blocking on?". **`ltrace`** does the same for library
  calls. Both slow the target substantially — they are diagnostic, not measurement, tools.
- **Also worth naming:** `bpftrace`/eBPF for production-safe kernel tracing, `valgrind`
  (callgrind/massif) for instruction-level and heap profiling, `lsof` and `fuser` for "who has
  this file/device open", `dmesg` for kernel-side truth, `ss -s` for socket summaries.
- **Interviewer's target sentence:** "Triage CPU vs memory vs I/O vs wait first, then `perf
  record -g` and a flame graph for hotspots, `perf stat` for cache behaviour, and `strace -c`
  when the question is which syscall it's stuck in — `lsof`/`fuser` when something is holding a
  device."
- **`piros2` line:** the repo's profiling is mostly application-level (per-node ms/frame
  measured against each process's own clock — see
  [09_real-time-and-performance.md](09_real-time-and-performance.md)), but the *"who has the
  device"* problem was solved properly: `camera.launch.py` pre-flight-checks `/dev/video0` and
  **names the PID holding it** by walking a `/proc` tree — with a **fake `/proc` tree in the
  unit tests** so the check is verified without hardware. That came from a real incident: a
  leaked `usb_cam` had fed a whole session unnoticed, and because capture is exclusive, every
  later session died with `Device or resource busy`.

## 8. Shell scripting

- **Correctness first:** `set -euo pipefail` (exit on error, error on unset variable, and —
  the one people miss — make a failing command in a *pipeline* fail the pipeline); quote every
  expansion (`"$var"`, `"$@"`) because word-splitting on unquoted variables is the single most
  common shell bug; `[[ ]]` over `[ ]` in bash; `trap … EXIT` for cleanup that must run on both
  normal exit and Ctrl-C (bash fires EXIT on SIGINT, so one trap covers both).
- **Robustness patterns for robotics scripts:** `timeout -s INT <n> <cmd>` to bound anything
  that can hang; `command </dev/null` so a background process can't steal stdin; explicit
  `ssh -o BatchMode=yes -o ConnectTimeout=…` so a remote call fails fast instead of hanging;
  `mktemp -d` for scratch space; writing to a temp file and `mv`-ing into place; and checking
  that a thing you started is *still alive* after warm-up rather than assuming it launched.
- **What not to do in shell:** parse `ls`, use `kill %N` on job specs when the job is a wrapper
  (kill the process *group* or match the real pattern), or grow past ~100 lines of logic — at
  that point it wants to be Python.
- **`just` vs `make`:** `just` is a command runner (recipes, arguments, no build-graph
  semantics), which is why it suits "run this session" better than make's file-dependency model.
- **Interviewer's target sentence:** "`set -euo pipefail`, quote everything, `trap … EXIT` for
  cleanup that also covers Ctrl-C, `timeout` around anything that can hang, and kill by process
  pattern rather than job spec — because job specs kill the wrapper and orphan the real
  process."
- **`piros2` line:** the justfile is the repo's shell surface and every one of those rules is
  there for a reason that was learned: the **teardown contract** (`trap … EXIT` + `pkill -f`
  every node pattern the recipe started, viewer in the foreground so closing the window ends
  the session, `just stragglers` to verify `clean` on both hosts) exists because `kill %N`
  orphaned grandchildren twice; every scripted `ssh pi` must carry
  `-o BatchMode=yes -o ConnectTimeout=5` because a bare ssh hangs ~2 minutes against a dead
  link and wedges any trap it sits in; camera launchers additionally need `ssh -tt` +
  ServerAlive + `</dev/null` so a link death reaps the remote session; and ad-hoc background
  runs are bounded with `timeout -s INT 30 …`. There is also a Wayland/X wrinkle that is pure
  environment scripting: rviz2 and Qt5 apps need `QT_QPA_PLATFORM=xcb` to be visible to X
  tools at all (a Qt5 app otherwise opens a native Wayland surface and `xwininfo` sees
  nothing), GLFW apps like Open3D's viewer need `XDG_SESSION_TYPE=x11` as well as unsetting
  `WAYLAND_DISPLAY`, GUI tools need `PATH="/usr/bin:$PATH"` because the dev box's `python3` is
  PlatformIO's venv and shadows the system one, and a non-interactive `ssh pi '…'` does **not**
  read the ROS environment (the exports live in `~/.profile`, since `.bashrc`'s interactivity
  guard would hide them) — so verification over SSH must use `ssh pi "bash -lc '…'"` or it
  silently runs on domain 0 with the default RMW.

## What to say if asked "how comfortable are you on Linux?"

"Comfortable as a systems debugger rather than as a kernel developer. The bugs I've chased in
my own robotics work were nearly all below the application: a 2.7 MB image fragmenting into
about 1800 UDP datagrams so BEST_EFFORT delivered exactly zero frames while RELIABLE
delivered all of them; DDS binding to a WireGuard interface the other machine couldn't route
to; a non-interactive SSH silently running on the wrong ROS domain because the exports were in
`~/.profile`; a leaked process holding a V4L2 device so every later session failed; and a
Wi-Fi link that died while the OS stayed up, which I fixed with a systemd-timer watchdog with
an escalation ladder and anti-boot-loop guards, and drilled until it recovered unaided. I know
`perf`, `strace` and cgroups at working level; I haven't written kernel drivers, and I'd say
so."
