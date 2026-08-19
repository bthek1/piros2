# Embedded and edge — the study file for section 14 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at conversation depth, the
sentence an interviewer is fishing for, and an honest **`piros2` line** — what the repo
actually does on its embedded target, checked against `docs/info/hardware.md`,
`docs/info/networking.md`, `docs/info/troubleshooting.md`, the `ansible/` roles and the
`justfile` before being written down. The syllabus files this section under breadth ("C++
inside ROS first, real SLAM second … the rest is breadth"). Honest-claim rule: reading this
does not move anything into skills.md; the `piros2` lines say what has been *built* — a
Raspberry Pi 5 on Ubuntu 24.04 aarch64 as a sensor head, provisioned by Ansible, with every
heavy computation deliberately pushed to an x86 box with a GPU. The Emesent frame for the
whole section: Hovermap is a compute-constrained payload on a drone (the JD: "considering
compute constraints of the hardware platform", "embedded or edge compute platforms"); the
compute module itself is not public — **don't assert Jetson**.

## Mental model to carry through the file

```
             power in (battery / USB-C PD)         heat out (fins, fan, airflow)
                     │                                       ▲
   sensors ─► SoC ───┼── CPU cores (ARM Cortex-A)  ──────────┤   ← "compute budget"
   (USB/CSI/         │── GPU / NPU / DLA (if any)            │     = W you can spend
    serial/CAN)      │── I/O blocks: USB, PCIe, UART, I2C,   │       × ms you have per frame
                     │   SPI, CAN, GPIO, ISP, codecs         │
                     ▼
              kernel drivers ─► /dev nodes, sysfs, sockets ─► your ROS nodes
```

Every embedded decision is "which box does the work, at what wattage, and how does software
reach it through the kernel". `piros2`'s answer: the camera compresses on its own silicon
(MJPG), the Pi forwards and does light CPU vision, the depth network runs on a desktop GPU
across the LAN — a split that exists only because the Pi has no usable accelerator.
Hovermap can't split like that in a tunnel with no link; everything runs on the payload.

## 1. ARM targets, Jetson platforms, single-board computers

- **ARM in robotics** means AArch64 application processors (Cortex-A cores running Linux),
  as opposed to Cortex-M microcontrollers running bare metal or an RTOS (the flight
  controller's world: STM32/PX4). Nearly every robot "edge computer" is one of three
  things: a **Pi-class SBC** (Pi 5 = BCM2712, 4× Cortex-A76 @ 2.4 GHz, up to 8 GB LPDDR4X,
  one PCIe 2.0 lane, the RP1 chip for USB 3/GbE/GPIO — no GPU compute worth using), an
  **NVIDIA Jetson** (ARM cores + an NVIDIA GPU on one physical memory), or a **fanless x86
  box** (NUC-class, 15–65 W, faster per core, no cross-compile pain).
- **Jetson family** (rules of thumb): Nano (Maxwell, 128 CUDA cores, 4 GB, 5–10 W,
  JetPack 4, EOL); Xavier NX (Volta, ~21 INT8 TOPS, 10–20 W); AGX Xavier (~32 TOPS,
  10–30 W); the **Orin** generation on JetPack 5/6 — Orin Nano (Ampere, ~40 TOPS, 7–15 W;
  the 2024 "Super" refresh ~67 TOPS at 25 W), Orin NX (~100 TOPS, 10–25 W), AGX Orin (up to
  ~275 TOPS, 15–60 W); and Jetson Thor (Blackwell, 2025) for humanoids. **JetPack** is the
  whole stack: L4T (Ubuntu userland + NVIDIA kernel/drivers), CUDA, cuDNN, TensorRT, VPI,
  multimedia API, flashing tools; JetPack 5 = Ubuntu 20.04, JetPack 6 = Ubuntu 22.04. Two
  consequences: your ROS distro is pinned to JetPack's Ubuntu (JetPack 6 → Humble natively;
  Jazzy means a container or source build), and TensorRT engines are built *per device* and
  don't move between GPU generations.
- **Why Jetson wins for perception payloads:** the GPU shares the CPU's memory (no PCIe
  copy), and TensorRT runs INT8/FP16 engines at a fraction of a desktop card's power. Why
  it loses: single-vendor toolchain with slow Ubuntu tracking, `nvpmodel` power modes that
  silently cap performance, and CPU-bound work (SLAM front ends, planning) that runs slower
  than on an x86 laptop — Wildcat's published odometry rate is "~15 Hz on a laptop".
- **Interviewer's target sentence:** "Pick the compute for the bottleneck: a neural network
  justifies a Jetson's shared-memory GPU and TensorRT despite the lock-in; CPU-bound SLAM
  and planning favour an x86 module — and you size either in watts, because on a drone the
  payload competes with the motors for the battery."
- **`piros2` line:** the ARM target is real — Pi 5, `aarch64`, Ubuntu 24.04.4, kernel
  `6.8.0-1047-raspi`, 4 cores, 7.8 GiB — running `ros-jazzy-ros-base` (no Qt on the sensor
  head, per `group_vars/robot.yml`), `usb_cam`, and the Canny node at ~16–20 fps /
  30–45 ms per frame. Neural depth (Depth Anything V2 Small, ONNX) runs on the dev box's GTX
  1660 SUPER at 72–79 ms/frame in-node vs 280–305 ms on that box's CPU — never on the Pi.
  No Jetson has been touched: JetPack, TensorRT, `nvpmodel` are reading knowledge.

## 2. Compute and power budgets

- **Power budget** is the first-class constraint on a flying payload: every watt of compute
  is a watt not spent hovering, plus its heat. Rules of thumb: Pi 5 ~3–4 W idle, ~8–12 W
  loaded (its 5 V/5 A USB-C supply is mostly headroom for peripherals); Jetson Orin NX
  10–25 W selectable; NUC-class x86 15–28 W TDP with 40 W+ peaks; a GTX 1660 SUPER is a
  125 W card — which is why `piros2`'s split cannot be a payload design. A DJI M300 lifts
  ~2.7 kg of payload and loses flight time with every gram and watt; Hovermap's ~1.6 kg
  already takes most of that, and Cortex "budgets battery" during autonomous exploration.
- **Compute budget** is the same idea in time: at 10 Hz you have 100 ms per LiDAR sweep for
  deskew + matching + map update + planning; if odometry needs 60 ms on this SoC you have
  40 ms for everything else, and a thermal throttle that halves the clock turns a working
  system into a drifting one. State budgets per stage (ms per frame at the target rate) and
  measure them on the *target*, at *thermal steady state* — not on a laptop at bench temperature.
- **Levers:** algorithmic (voxel downsample, subset of rings, no global optimisation —
  Emesent's onboard-vs-Aura trade-off: 0.1 m voxels onboard, a 15-min mission in ~80 s vs
  ~1,200 s offline), scheduling (pace producers, drop rather than queue), precision
  (FP16/INT8), offload (GPU/DLA/ISP), and not doing it (compress in the camera, decimate the
  mesh, cap the map).
- **Interviewer's target sentence:** "I budget milliseconds per stage at the target rate and
  watts per box, measure both on the real hardware at steady state, and make overload
  degrade gracefully — pace or drop at the source — instead of queueing into stale results."
- **`piros2` line:** budgets were measured, per-process-clock because the camera's
  `header.stamp` is faulted (~0.73 s lag): Canny 30–45 ms on the Pi; ORB ~14 ms/frame; cloud
  projection ~12 ms; TSDF integrate 52–78 ms on CUDA; `rgbd_odometry` sustains ~4–5 Hz, so
  the depth estimator's `max_rate: 5` paces the *whole* pipeline (GPU does half the work,
  the odom TF stays current — unpaced, TF lagged the clouds by ~0.8 s median). Frame rate
  itself is a budget lesson: the C922 gives 18–21 fps under `exposure_dynamic_framerate=1`,
  30 at fixed exposure, 42–60 under the reset baseline at 720p60 — "never quote fps without
  the exposure mode". Watts were never metered; `throttled=0x0` and no undervoltage in the
  journal is the only power evidence recorded.

## 3. GPU acceleration on edge, CUDA basics

- **CUDA model:** a *kernel* is a function run by thousands of threads; threads form
  *blocks* (which share fast on-chip *shared memory* and can `__syncthreads()`), blocks form
  a *grid*; hardware schedules threads in *warps* of 32 on *streaming multiprocessors*.
  Memory: registers → shared (~100 KB/SM) → L2 → global device DRAM, plus host memory over
  PCIe on a discrete card. Most kernels are **memory-bound**: coalesce accesses (adjacent
  threads, adjacent addresses), minimise host↔device copies, keep intermediates on the
  device, overlap copy and compute with *streams*; warp divergence (one warp, two branches)
  is the other classic cost.
- **On Jetson** host and device share one DRAM, so "zero-copy" (mapped pinned / unified
  memory) removes the PCIe tax entirely — the architectural reason a 15 W Orin can beat a
  100 W discrete-GPU pipeline that copies every frame.
- **What robotics engineers actually use:** rarely hand-written CUDA. TensorRT (ONNX →
  optimised FP16/INT8 engine), CUDA-backed libraries (cuBLAS/cuDNN, `cv::cuda`, Open3D's
  tensor backend, Isaac ROS with NITROS zero-copy between nodes), ONNX Runtime's CUDA/TensorRT
  execution providers. The discipline is knowing *whether you're on the GPU at all* — every
  stack has a silent CPU fallback.
- **Trade-off:** GPUs win dense, regular, data-parallel work (conv nets, image ops, TSDF
  integration, ray casting) and lose branchy pointer-chasing work (KD-tree search, sparse
  Cholesky) unless restructured; a 10 ms kernel is worthless behind a 15 ms copy.
- **Interviewer's target sentence:** "Edge GPU acceleration is a memory-traffic problem
  before it's a FLOPs problem — keep data resident, use unified memory on Jetson, build
  TensorRT engines on the device, and verify the provider actually selected, because every
  framework degrades to CPU silently."
- **`piros2` line:** CUDA via libraries, not kernels: `onnxruntime-gpu`
  (`CUDAExecutionProvider`, the pip `[cuda,cudnn]` extras) for depth, and Open3D 0.19's
  CUDA wheel for the `VoxelBlockGrid` TSDF (marching cubes OOMs on the 6 GB card below ~8 mm
  voxels and falls back to CPU). The silent-fallback lesson was learned twice: the estimator
  drops to CPU with no error if the nvidia pip libs are missing or `preload_dlls()` isn't
  called, so it *logs the winning provider*; and NVIDIA userspace 595.84 against loaded
  module 595.71 broke *all* GL until reboot (`__GLX_VENDOR_LIBRARY_NAME=mesa
  LIBGL_ALWAYS_SOFTWARE=1` was the stopgap — `LIBGL_ALWAYS_SOFTWARE` alone isn't enough
  because GLVND still dispatches GLX to the broken vendor library). All on x86.

## 4. Cross-compilation and sysroots

- **Cross-compiling** = building on host A (x86 build server) binaries that run on target
  B (aarch64). Needs a *toolchain* for the target triple (`aarch64-linux-gnu-gcc`, or clang
  `--target=aarch64-linux-gnu`) and a **sysroot**: a tree holding the *target's* headers and
  libraries (`/usr/include`, `/usr/lib/aarch64-linux-gnu`, the ROS install) so compiler and
  linker resolve against the target's glibc, Eigen, PCL — not the host's; produced by
  copying a provisioned board's rootfs or installing `:arm64` multiarch packages. In CMake
  it's a **toolchain file** (`CMAKE_SYSTEM_NAME/PROCESSOR`, `CMAKE_C/CXX_COMPILER`,
  `CMAKE_SYSROOT`, and `CMAKE_FIND_ROOT_PATH_MODE_*` so `find_package` never picks host
  libraries); colcon passes it with `--cmake-args -DCMAKE_TOOLCHAIN_FILE=…`.
- **What people actually do** (ROS 2 sysroots are painful — every `package.xml` dependency
  must exist for the target): build natively on the board (zero mismatch, slow — fine on a
  Pi 5, hopeless on a Nano); build in an **arm64 Docker image under QEMU**
  (`qemu-user-static` + binfmt: 5–10× slower than native but exactly the target's userland,
  cache-friendly in CI); an arm64 build farm (Graviton, spare Jetsons); or cross-compile
  only the hot C++ and ship Python unchanged.
- **Classic mistakes:** host libraries leaking into the link; build image glibc newer than
  the target's (`GLIBC_2.38 not found` at run time); forgetting Python C extensions
  (`rclpy`, `cv_bridge`) are architecture-specific too.
- **Interviewer's target sentence:** "A sysroot is the target's root filesystem seen from
  the host compiler; the toolchain file points CMake at it and *only* it. For ROS 2 I'd
  reach for an emulated arm64 container in CI before a hand-maintained sysroot, and pin the
  target's glibc to the build image's."
- **`piros2` line:** not cross-compiled — the Ansible `workspace` role rsyncs the source
  tree to the Pi (excluding `build/ install/ log/` and the 99 MB depth model) and runs
  `colcon build --symlink-install` *natively* on the board as the login user; one repo
  builds on x86 and aarch64. Docker for the Pi was considered and rejected. The nearest
  mismatch lesson is interpreter-level: colcon hardcodes `#!/usr/bin/python3` into entry
  points, so the venv-only `onnxruntime` node runs as `python -m` under the venv interpreter.

## 5. Serial, I2C, SPI, CAN

| Bus | Wires | Topology / speed | Linux face | On a robot |
| --- | --- | --- | --- | --- |
| **UART/serial** | TX, RX, GND (+RTS/CTS) | point-to-point, asynchronous, agreed baud (9600 … 921600+), 8N1 = 10 bits/byte | `/dev/ttyUSB0`/`ttyACM0` (USB-serial), `/dev/ttyAMA0`/`ttyS0` (SoC UART); `termios`; group `dialout` | autopilots (MAVLink), **DJI OSDK**, GNSS NMEA, PPS |
| **I2C** | SDA, SCL (open-drain, pull-ups) | multi-drop master–slave, 7-bit address, 100k/400k/1M | `/dev/i2c-N`, `i2cdetect`, `ioctl(I2C_RDWR)` | IMUs, magnetometers, power monitors |
| **SPI** | MOSI, MISO, SCLK, CS per device | master–slave, full duplex, tens of MHz, mode 0–3 (CPOL/CPHA) | `/dev/spidevB.C`, `spi_ioc_transfer` | fast IMUs, ADCs, displays |
| **CAN / CAN FD** | CAN-H/L differential, 120 Ω both ends | multi-master, ID arbitration (lower ID wins, non-destructive), 1 Mbit/s classic; FD 64-byte frames, ~5–8 Mbit/s data phase | **SocketCAN**: `ip link set can0 up type can bitrate 500000`, `AF_CAN`, `candump`; DBC decodes payloads | motor controllers, BMS, vehicle buses, actuators |

- **Serial arithmetic:** bytes/s ≈ baud/10. DJI's Onboard SDK on the M300/M350 talks to
  Hovermap over the OSDK serial port at **230400 baud** ≈ 23 kB/s — enough for telemetry
  and velocity/position setpoints at tens of Hz, nowhere near imagery; the payload does its
  own perception and only *commands* the aircraft ("Cortex issues velocity/position
  commands; DJI keeps the inner attitude loop"). No clock line, so both ends must be within
  ~2–3% of the baud; framing errors after a cable wiggle are the classic symptom.
- **CAN's design point** is determinism and noise immunity on a vehicle: arbitration
  guarantees the highest-priority frame wins without collision loss, terminated differential
  pairs survive motor noise, every node checks every CRC; the cost is 8 bytes (64 in FD) per
  frame. Linux makes it a network interface, which is why `ros2_socketcan` and
  `ros2_control` hardware interfaces just open a socket.
- **I2C vs SPI:** I2C is two wires and addressable but slow and prone to bus lock-ups (a
  slave holding SDA low needs clock-pulse recovery); SPI is fast and simple but a chip select
  per device and no ACK. Neither should feed a SLAM IMU from *userspace* reads — the latency
  jitter is what preintegration can't tolerate; real rigs timestamp in the sensor or an MCU
  and send stamped packets.
- **Interviewer's target sentence:** "UART for point-to-point links to autopilots and GNSS,
  I2C/SPI for board-level sensors, CAN when it must survive a vehicle and multiple masters —
  on Linux they're all file descriptors: `termios` on a tty, ioctls on `/dev/i2c-N` and
  `/dev/spidev`, a socket for CAN. The engineering is timestamping and framing, not opening
  the device."
- **`piros2` line:** not touched — the only bus is USB (UVC over `xhci`); the user is in
  `dialout` via cloud-init without ever opening a tty. Two adjacent facts are real: `gpio`,
  `i2c`, `spi` groups **do not exist** on Ubuntu for Pi (Raspberry Pi OS vendor additions —
  naming one in cloud-init `groups:` makes `useradd` fail and the headless board comes up
  with no way in), so the servo milestone needs groups plus a udev rule; and V4L2 controls
  are set one `--set-ctrl` per call because a control held inactive by its auto mode fails a
  batched set with a misleading `Permission denied`.

## 6. Device drivers and kernel interfaces

- **The shapes a driver exposes:** character devices (`/dev/video0`, `/dev/ttyUSB0`,
  `/dev/i2c-1`, `/dev/gpiochip0` — `open/read/write/ioctl/mmap`), block devices, network
  interfaces (`can0`, `wlan0` — sockets, not files), **sysfs** (one value per file:
  `/sys/class/net/wlan0/statistics/tx_bytes`, `/sys/class/thermal/thermal_zone0/temp`),
  **procfs** (`/proc/<pid>/fd`, `/proc/uptime`, `/proc/device-tree/…` on ARM), **netlink**
  (uevents, routing), and **udev** turning uevents into stable symlinks
  (`/dev/v4l/by-id/usb-046d_C922…-video-index0`) and permissions (`GROUP="video"`). ARM
  boards add the **device tree** — firmware describes non-discoverable hardware (UARTs, I2C,
  GPIO) to the kernel; on a Pi `config.txt` overlays edit it, and `/proc/device-tree/chosen`
  is how you ask the firmware what actually happened.
- **V4L2 as the worked example:** `uvcvideo` (in-tree) drives any USB Video Class camera;
  userspace negotiates a format (`VIDIOC_S_FMT`), requests `mmap`'d buffers
  (`VIDIOC_REQBUFS`, no copy), queues/dequeues them, and reads controls via `VIDIOC_G/S_CTRL`
  — `v4l2-ctl` is a CLI over the same ioctls. Only one process may *stream* (exclusive
  capture) but control writes work alongside a live capture; the metadata node is a
  separate device yielding UVC timing metadata, not frames. Kernel version matters at the
  *control-name* level: `exposure_auto` → `auto_exposure`, `focus_auto` →
  `focus_automatic_continuous`; a driver written to old names silently does nothing.
- **Modules, DKMS, mismatch:** out-of-tree drivers (`v4l2loopback`, NVIDIA) compile against
  the running kernel via DKMS; a kernel API change breaks the build and leaves apt
  half-configured; userspace/module version skew breaks every consumer until reboot.
  Embedded teams pin kernels and treat "apt upgraded the driver" as change control.
- **GPIO** on modern kernels is `libgpiod` over `/dev/gpiochipN` (sysfs GPIO is
  deprecated); bit-banging a µs-timed protocol from Linux userspace is the classic mistake —
  hence MCUs for anything with tight timing.
- **Interviewer's target sentence:** "I reach hardware through the kernel's contract — a
  char device with ioctls, a sysfs attribute, a socket — and read the device tree and
  `/proc` to learn what the firmware configured; when something 'doesn't work' I ask which
  module claimed the device, whether control names changed with the kernel, and whether
  userspace and module versions match."
- **`piros2` line:** V4L2/UVC end to end: `uvcvideo` on `6.8.0-1047-raspi`, capture on
  `/dev/video0` (`/dev/video1` is the metadata node — hardware.md's "common early mistake";
  the Pi's ISP/HEVC blocks occupy `/dev/video19`–`37` and are not cameras), the by-id symlink
  in `group_vars` because node numbers shift across kernels, `usb_cam`'s ROS 1-era control
  names dead on this kernel so `v4l2-ctl` is the only channel, and **controls persist inside
  the camera** across processes and reboots (a benchmark's manual exposure blackened every
  later session; the camera powers on with `exposure_dynamic_framerate=1` despite the
  driver's reported default 0) — hence `just camera` / `just camera-reset`.
  `camera.launch.py` pre-flights a held device by scanning `/proc/*/fd` and names the holder
  PID (a leaked usb_cam had fed a whole session unnoticed). Dev-box module lessons:
  `v4l2loopback-dkms` failing against kernel 7.0.0 blocked all of apt; the NVIDIA mismatch.
  No driver has been *written*.

## 7. Boot, init and systemd on embedded Linux

- **The chain** on a Pi: SoC ROM → **bootloader in SPI EEPROM** (Pi 4/5: holds `BOOT_ORDER`,
  USB timeouts, its own version) → `config.txt`/`cmdline.txt` on the boot partition → kernel
  + device tree (+ overlays, initramfs) → root mounted by `PARTUUID=`/`LABEL=`/`UUID=` →
  **systemd** as PID 1 bringing up targets from unit files. Jetson uses UEFI (JetPack 5+)
  with A/B rootfs slots; x86 uses UEFI + GRUB. A Pi has no BIOS, and its bootloader is a
  firmware image with its own update path (§9).
- **systemd for a robot:** services (`Type=simple|oneshot|notify`, `Restart=on-failure`,
  `WatchdogSec=` + `sd_notify` for a supervised process, `After=` vs `Requires=`/`Wants=`),
  **timers** as cron (`OnBootSec`, `OnUnitActiveSec`, `AccuracySec`), **device units**
  (`sys-subsystem-net-devices-wlan0.device`) to hang work off hardware appearing, and
  `journalctl` as the single log. Pitfalls: `After=network-online.target` doesn't guarantee
  a *usable* link (Wi-Fi may still be associating); a `RemainAfterExit` oneshot won't re-run
  when its trigger fires again; a ROS launch as a service needs the ROS environment *in the
  unit* — systemd reads no `.profile` (`Environment=`/`EnvironmentFile=` or a `bash -lc`
  wrapper).
- **cloud-init / first boot** on Ubuntu images: `user-data` (users, groups, keys, password
  hash) and `network-config` (netplan) on the boot partition are consumed once; on a
  headless Wi-Fi-only board a typo there means a keyboard and monitor.
- **Power-loss safety** (what a payload wants and a learning Pi doesn't have): overlayfs
  root, logs on tmpfs or a wear-managed partition, `fsync` discipline for bags, no
  unclean-shutdown surprises — a battery pull is an unclean shutdown *every flight*.
- **Interviewer's target sentence:** "On embedded Linux the boot chain is firmware →
  bootloader → kernel + device tree → systemd, and every hop is configuration you own: boot
  order in the EEPROM, root by PARTUUID, cloud-init on first boot, units and timers for
  supervision — with the journal as the flight recorder you read after the fact."
- **`piros2` line:** lived, some of it painfully. `BOOT_ORDER=0xf41` (nibbles read right to
  left: SD, USB, restart), `USB_MSD_PWR_OFF_TIME=3000` to boot from USB at all, boot medium
  read back from `/proc/device-tree/chosen/bootloader/boot-mode`. Root is pinned by
  **PARTUUID** (`5ec0ffee-01/-02` via `sfdisk --disk-id`, in `cmdline.txt` *and*
  `/etc/fstab`) because Ubuntu's Pi image labels every copy `system-boot`/`writable` and a
  second copy collides on label, filesystem UUID *and* PARTUUID — root was a coin-flip.
  cloud-init wrote the user with `video`/`dialout`/`sudo` (never `gpio`/`i2c`/`spi`) and the
  Wi-Fi PSK. systemd is used the way a fielded box would: the Ansible `wifi` role installs
  `wifi-watchdog.timer` (`OnBootSec=90`, `OnUnitActiveSec=60`, `AccuracySec=10`) firing a
  `Type=oneshot` service that runs an **escalation ladder** — gateway ping; after 3
  consecutive minute-spaced failures reassociate, after 6 reload `brcmfmac`, after 9 reboot,
  guarded by a 600 s uptime floor and a 3600 s cooldown; the fail counter lives in `/run`
  (tmpfs, resets on boot), the reboot marker in `/var/lib` (survives one) — and
  `wifi-powersave.service` is `WantedBy` the `wlan0` *device unit* so a driver reload
  re-runs it. Drilled: the reassociate rung reproduced the AP's `status_code=16` rejection,
  the driver-reload rung recovered unaided at T+426 s. ROS nodes are deliberately *not*
  services here — sessions are foreground recipes torn down with the window — a
  learning-project choice a payload wouldn't make.

## 8. Thermal and resource constraints

- **Thermal:** SoCs throttle by design — a Pi 5 starts pulling clocks around 80–85 °C and
  needs the active cooler for sustained load; Jetsons have `nvpmodel` modes, `jetson_clocks`
  and `tegrastats`; x86 has PL1/PL2. Read `/sys/class/thermal/thermal_zone*/temp` and
  cpufreq's `scaling_cur_freq`; on a Pi `vcgencmd get_throttled` (bit flags: under-voltage,
  currently/previously throttled). A sealed payload has airflow in flight and none on the
  ground — the ground case is the thermal design point for a pre-flight SLAM init.
- **Memory** is the other hard wall: 4–8 GB shared with the GPU on a Jetson, no swap worth
  using on SD/eMMC, the OOM killer taking the largest process — usually the map — without
  warning. Rules: cap every accumulator (map, keyframe store, triangles), preallocate on
  RT paths, watch RSS over an hour, not a minute.
- **Storage:** SD cards are slow on random writes and wear out; bagging tens of MB/s onto SD
  is a real bottleneck; industrial eMMC/NVMe with `noatime` is the fielded answer.
- **The link as a resource:** on a payload it is intermittent by design (Hovermap stores
  maps onboard, uploads on return to Wi-Fi/radio); anything that *depends* on it is a fault
  waiting to happen.
- **Interviewer's target sentence:** "Edge constraints are coupled — heat caps clocks, clocks
  stretch the compute budget, memory caps the map, storage caps the recording — so I cap
  accumulators explicitly, measure at steady state, and make the system degrade loudly and
  locally rather than depend on the link."
- **`piros2` line:** thermals only checked negatively (the Wi-Fi incidents ruled out power
  and heat: `throttled=0x0`, no undervoltage, load ~0.03; the Pi 5's USB-C connector unseating
  on a camera replug rebooted the board once, reverting the camera's controls). Caps are
  deliberate: voxel map hard-capped with `max_range` 6 m, the mesher decimates to a triangle
  budget (120k in the fork), `max_rate` paces the GPU, marching cubes' 6 GB OOM falls back
  to CPU, the keyframe store is novelty-gated. The resource that *actually* failed was the
  **Wi-Fi link**: five dev-box readers each pulled a unicast copy of the compressed stream
  and collapsed it (14+ MiB/s, ~2 frames/s each) — fixed by one Wi-Fi reader
  (`camera_relay`) fanning out on loopback — and a subscriber remapped to `/image_raw`
  silently pulled 2.7 MB *raw* frames over the air. Storage: MCAP bags at ~36 MiB per 24 s,
  recorded on the Pi's SD then fetched.

## 9. Firmware update paths

- **What must be updatable on a fielded box:** the *bootloader/EEPROM* (Pi's
  `rpi-eeprom-update`; Jetson's UEFI/QSPI via the flashing tools), the *OS image or rootfs*,
  the *application* (ROS workspace, containers), and *peripheral firmware* (LiDAR, camera,
  autopilot — DJI updates the M300 through its own app; MCUs via DFU/serial bootloaders).
  Each has its own tooling and its own way to brick a device.
- **The patterns:** **A/B dual-slot images** with a bootloader that flips slots and rolls
  back unless the new image marks itself healthy within N boots (Mender, RAUC, SWUpdate;
  Jetson supports A/B rootfs); **package-based** (`apt`, Debian packages from CI — cheap,
  not atomic, can leave a half-configured system); **container images** for the application
  layer (pull a tag, restart the service; the OS stays stable); **delta/OTA** over a metered
  link with signature verification. Field rules: signed images, an always-bootable recovery
  slot, a watchdog that reboots a hung update, never bootloader and OS in one step, and a
  version you can *read back from the device*.
- **The classic mistake:** the update tool reports the version it *bundles*, not the one
  *running*; a downgrade that preserves config keys looks like success.
- **Interviewer's target sentence:** "For a fielded payload I want atomic A/B updates with
  automatic rollback and signed images, the application decoupled from the OS, and running
  versions readable from the device — because the failure mode isn't 'update fails loudly',
  it's 'update silently regresses'."
- **`piros2` line:** no A/B or OTA machinery — updates are `apt` through idempotent Ansible
  roles (`changed=0` on rerun) plus `rsync` + `colcon build`. But the silent-regression
  lesson is documented from real output: **Ubuntu downgrades the Pi's bootloader** — its
  `rpi-eeprom` package bundles only `pieeprom-2024-09-23.bin` and enables
  `rpi-eeprom-update.service`, so newer firmware applied from Raspberry Pi OS is reverted
  the moment `rpi-eeprom-config --apply` runs; config keys survive, the version doesn't, and
  `rpi-eeprom-update` reports Ubuntu's bundle as both CURRENT and LATEST — the running
  version is read from `/proc/device-tree/chosen/bootloader/build-timestamp`. Also lived: a
  kernel update's DKMS post-install failing left three `linux-*` packages unconfigured and
  every later apt run red — the non-atomic package path failing as advertised. And camera
  *state* persists inside the C922 across reboots; the `just camera-reset` baseline is the
  nearest thing to a known-good image for a peripheral.

## What to say if asked "have you shipped on embedded hardware?"

"I've run ROS 2 on a Raspberry Pi 5 as a sensor head — Ubuntu 24.04 aarch64, provisioned by
Ansible, camera driver and light CPU vision on the board, the neural depth network on a
desktop GPU across the LAN because the Pi has no accelerator — and I've dealt with what
actually breaks: the EEPROM bootloader silently downgraded, PARTUUID vs label root pinning,
cloud-init groups on a headless board, V4L2 control state persisting in the camera, a Wi-Fi
link that dies while the OS lives, fixed with a systemd-timer watchdog and an escalation
ladder. My professional embedded background is firmware-side C/C++, not Jetson or CAN — I
know JetPack, TensorRT, SocketCAN and cross-compilation at reading depth and would say so."
Then stop.
