# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this is

A **learning project** for ROS 2 on real hardware: a Raspberry Pi 5 with a Logitech
C922 webcam, driven from an Ubuntu dev box. The value is in understanding ROS 2, not
in shipping a product.

That changes how to work here. When implementing something, explain the ROS concept
it exercises — why a QoS profile matters, what a launch file actually does, why TF
frames are shaped the way they are. Prefer the idiomatic ROS 2 way over a shortcut
that happens to work. Don't generate a large finished subsystem when a small
working piece the user can reason about would teach more; if a task is genuinely
large, build it in steps that each run.

**ROS distro: Jazzy Jalisco.** Don't mix in Humble or Foxy instructions — the CLI,
launch API, and QoS defaults differ between them.

## The two machines

| | Dev box (here) | Raspberry Pi |
| --- | --- | --- |
| Reach it | local | `ssh pi` (→ `bthek1@192.168.2.17`, key auth, works with `BatchMode=yes`) |
| OS | Ubuntu 24.04.4 LTS, x86_64 | Ubuntu 24.04.4 LTS, aarch64 (reflashed 2026-07-23) |
| Kernel | — | `6.8.0-1047-raspi` |
| ROS | `ros-jazzy-desktop`, native apt — **installed 2026-07-24** | `ros-jazzy-ros-base`, native apt — **installed 2026-07-24** |
| Provisioning | Ansible control node (core 2.16.3, installed) | Ansible managed host |
| Runs | `rviz2`, `rqt`, editing, builds | camera node, anything touching hardware |
| Network | `192.168.2.109/24` on `enp6s18` | `192.168.2.17` on **`wlan0`** — no Ethernet cable |
| Display | GNOME + Xwayland — the only host that can run `rviz2` | headless |
| `sudo` | prompts for a password | passwordless |

**Both machines run ROS natively.** Docker was considered for the Pi and rejected —
do not reintroduce container instructions.

The Pi is reachable non-interactively, so **verify claims about the hardware by
running commands over SSH** rather than assuming. For example:

```bash
ssh pi 'v4l2-ctl --list-devices'
ssh pi 'ls -l /dev/video0'
```

Full measured specs: [docs/info/hardware.md](docs/info/hardware.md).

## Current state

**The `ansible/` tree is built and working** (2026-07-24): `site.yml` plus five
roles — `ros2_apt`, `ros2_install`, `ros2_env`, `camera`, `workspace` — and the
playbook is idempotent (a clean rerun reports `changed=0` on the Pi).

Machine state:

- **Pi: fully provisioned and verified.** `ros-base` + `demo_nodes_cpp`, the
  camera stack (`usb_cam`, `image-transport-plugins`, `v4l-utils`), env vars
  (`ROS_DOMAIN_ID=42`, CycloneDDS pinned to `wlan0`), repo synced to `~/piros2`.
- **Dev box: provisioned and verified** — `desktop`, env vars, CycloneDDS pinned
  to `enp6s18`. Sudo prompts here, so playbook runs need `--ask-become-pass`.
  The playbook is idempotent here too (`changed=0`); the kernel/DKMS apt
  blocker was resolved 2026-07-24 by removing the unused `v4l2loopback-dkms`.
- **Milestone 0 passed 2026-07-24**: `/chatter` published on the Pi arrived on
  the dev box across the LAN. [docs/info/roadmap.md](docs/info/roadmap.md) tracks status.

First package: `src/piros2_hello` (milestone 1, done 2026-07-24) — a
hand-written `ament_python` talker/listener pair, built on both machines and
verified across the LAN (`just hello`).

Milestone 2 (camera) is done (2026-07-24): `usb_cam` publishes from the Pi at
a measured 29.72 fps, viewed live in `rqt_image_view` over compressed
transport (`just cam` runs camera + viewer + cleanup). Image tuning lives in
`camera.yaml` (brightness, JPEG re-encode quality) plus the `gain:=` launch
argument.

Milestone 3 (launch files, done 2026-07-24): `src/piros2_camera` —
`launch/camera.launch.py` + `config/camera.yaml`, resolution/framerate as
launch arguments. `just cam` runs it; args pass through
(`just cam image_width:=640`). The usb_cam TF param is **`frame_id`** —
`camera_frame_id` is a ROS 1 name it silently ignores.

Milestone 4 (image processing, done 2026-07-27): `src/piros2_vision` — a
hand-written Canny edge detector (`cv_bridge`, hand-rolled compressed
variant), ~16–20 fps at ~30–45 ms/frame on the Pi; `vision.launch.py`
composes the camera launch via `IncludeLaunchDescription` (`just edges`), and
`test_edge_detector.py` unit-tests `on_frame` with captured publishers. Its
first version exposed the camera's timestamp fault, and its QoS exposed a
second trap: **BEST_EFFORT receives zero large frames** — 2.7 MB messages
fragment into ~1800 UDP datagrams, one always drops, and only RELIABLE
reassembles; the node subscribes RELIABLE/KEEP_LAST-1 on purpose.

Milestone 5 (TF, in progress 2026-07-27): `camera.launch.py` publishes the
static frame chain `base_link → camera_link → camera_optical_frame`
(placeholder mount pose 5 cm up; canonical −90/0/−90 optical rotation), and
image headers now carry `camera_optical_frame` — verified across the LAN with
`tf2_echo`. The RViz visual check passed 2026-07-28 (closed by perception
P2's live-cloud session); checkerboard calibration remains open as an
accuracy upgrade — perception P0's approx intrinsics released its gate. A
first calibration session (2026-07-27 evening) live-debugged
`just calibrate` into working shape — three recipe bugs fixed (dead
`-p camera:=` service remap, window close ignored by the calibrator,
`kill %N` traps orphaning nodes; see docs/info/troubleshooting.md) — but
ended without a save, so the measured yaml is still unwritten.

Milestone 6 (record/replay, done 2026-07-27): `just record` bags the
compressed stream + `camera_info` + `/tf_static` on the Pi (24 s ≈ 36 MiB
MCAP) and fetches it to `bags/`; `just replay` runs it through
`image_transport republish` into the edge detector entirely on the dev box —
no Pi needed. Jazzy's `republish` takes transports as *parameters*, not
positional args (see docs/info/troubleshooting.md).

**The roadmap concluded 2026-07-27.** The perception build followed it —
[docs/plans/completed/perception-plan.md](docs/plans/completed/perception-plan.md), **closed 2026-07-29 by
decision, before P3's map was built**: phases P0–P4 building
`src/piros2_perception` (neural monocular depth → point clouds → a room map).
P0's gate was released 2026-07-28 with **spec-derived approximate
intrinsics**: `c922_720p_approx.yaml` (78° diag FOV → fx = fy ≈ 907 px,
centred principal point, zero distortion) is live via `camera_info_url`,
verified on `/camera_info`. Good to a few percent; the checkerboard run is
now an accuracy upgrade, not a blocker — the measured yaml replaces the
approx file when someone waves the board.

Perception P1 (depth node, done 2026-07-28): `src/piros2_perception` —
`depth_estimator` subscribes `/image_raw/compressed`, runs Depth Anything V2
Small (fp32 ONNX, `just fetch-model`, checksum-pinned, git-ignored) and
publishes `/depth` (32FC1) + a colourised preview; verified against the
milestone-6 bag. **On the GPU since 2026-07-30**: `onnxruntime-gpu` with
the pip `[cuda,cudnn]` extras runs it on the dev box's GTX 1660 SUPER at
72–79 ms/frame in-node (~13 fps; the CPU figure was 280–305 ms, ~3 fps).
The node prefers CUDA, falls back to CPU, and logs the winning provider —
the GPU path degrades to CPU *silently* if the nvidia pip libs are missing
or `preload_dlls()` isn't called (docs/info/troubleshooting.md). It is the
repo's documented venv escape hatch: onnxruntime is PyPI-only, lives in
`~/.venvs/piros2-perception` (`--system-site-packages`), and the node must
run as `python -m` under that interpreter — colcon's hardcoded shebang
misses the venv, so `ros2 run` gets no onnxruntime. `just depth` owns the
invocation; the package README explains it.

Perception P2 (cloud projector, done 2026-07-28): `cloud_projector` syncs
`/depth` + `/image_raw/compressed` by header stamp (message_filters),
projects through `/camera_info`'s K, and hand-builds `PointCloud2` (numpy
structured array = the wire format); measured 33k–57k points in ~12 ms per
cloud, verified live. `perception.launch.py` runs both dev-box nodes (the
estimator via `ExecuteProcess` under the venv python — launch_ros `Node`
would exec the system-shebang entry point); it deliberately does NOT
include the camera launch, which would open `/dev/video0` locally —
`just cloud` starts the camera over SSH and opens RViz.
The RViz check passed 2026-07-28 (live cloud, correctly posed in
`base_link`, human-confirmed) — which also closed milestone 5's deferred
RViz checkbox. The one human step still open: the tape-measure scale check
(lit room, wall at a known distance, tune `depth_scale`).

Perception P3 (mapping; plumbing verified 2026-07-29, then the plan was
closed by decision — the sweep and the map itself were never run):
`mapping.launch.py` (depth estimator + RTAB-Map `rgbd_odometry` + `rtabmap`,
exact sync off the depth node's honest headers, `base_link` poses) and
`just map [bag]` (plays a bag once — looping teleports odometry — then
republish + launch + `rtabmap_viz`). rtabmap 0.22.1 installed via
`extra_ros_packages` / `just deploy-dev`; the static plumbing bag ran the
full chain with odometry quality 447–563 and a 1-node map (correct for a
motionless scene — RTAB-Map merges lookalike frames). The milestone-6 bag
can't feed mapping — its `/camera_info` predates the P0 intrinsics (K all
zeros); `just record` now takes a bag name, and `bags/static1` (19 s, valid
K) is the plumbing bag. RTAB-Map's `delay=` figure is the bag's age, and its
5-second no-data warnings are a watchdog beating against ~1–2 Hz synced
pairs — neither is a fault. If mapping resumes: exposure fix, hand-held
sweep (`just record 45 sweep1`), `just map bags/sweep1`, tuning loop.

The world dashboard (done 2026-08-03, the whole
[world plan](docs/plans/completed/world-plan.md) in one session):
`src/piros2_world` — `keypoint_detector` (ORB on the compressed stream,
~14 ms/frame at the 500-feature cap — briefly cut to 100 on 2026-08-04,
restored the same day once match colouring made the overlay legible —
publishes `/keypoints/compressed` +
`/keypoints/count` as a plain Int32; since 2026-08-04 it also
Hamming-matches descriptors against a 10-frame window — matched
keypoints drawn green, new ones yellow, count on `/keypoints/matched`,
shown as a percentage on the dashboard —
the first step toward tracking camera pose) and `dashboard` (three latest-wins
RELIABLE subscriptions — deliberately no message_filters sync, the
contrast with `cloud_projector` is the lesson — composed by a 10 Hz
wall-timer into a 2×2 mosaic + stats panel on
`/world/dashboard/compressed`). All rates and the STALE banners are
measured against the dashboard's own receipt clock, never `header.stamp`
(the 0.73 s camera fault). `world.launch.py` follows the
perception-launch rules (venv `ExecuteProcess` for depth, no camera
include); `just world` runs it live and is the new `just run` target.
The live run surfaced a finding, verified 2026-08-04: the camera panel
reads 42–60 msgs/s, not ~30 — all distinct frames; the C922 genuinely
exceeds the old "30 fps ceiling" now (see the 720p60 bullet below).
Throttling is left for the "reduce compute" todo.

**In progress:** the [world 3D plan](docs/plans/in-progress/world-3d-plan.md)
(authored 2026-08-04, not started) — rotation-only camera orientation from
the keypoint matches (Kabsch on bearing rays; the essential matrix is
degenerate under pure rotation) published as `odom → base_link`, then the
depth clouds accumulated in that frame into an RViz panorama. Phases
P0–P4; honest scope: orientation without position, drift accepted, reset
services instead of loop closure.

Testing: `just test` (colcon test + result aggregation) or the VSCode Testing
sidebar — both report identically. All packages are style-clean and the suite
is green (47 tests; `piros2_vision`, `piros2_perception` and `piros2_world`
carry real unit tests — none need hardware or model weights: a fake ONNX
session for the estimator, synthetic depth planes for the projector,
synthetic chessboards for the keypoint detector, and pure-function
`compose_grid`/`rates` tests for the dashboard, and matching tests on
seeded greyscale noise — a chessboard's lookalike corners defeat the
matcher's cross-check by design) as of 2026-08-04.

Don't write docs or code that imply a package exists when it does not. If a doc
describes something not yet built, mark it as planned — the existing docs follow
this convention and [docs/info/roadmap.md](docs/info/roadmap.md) tracks status.

## Constraints that are easy to get wrong

- **Both machines are now Ubuntu 24.04 noble**, Jazzy's Tier 1 platform, so
  `apt install ros-jazzy-ros-base` works on the Pi. `packages.ros.org` serves 3361
  `ros-jazzy-*` binaries for `noble/arm64` against **zero** for `bookworm/arm64`,
  which is why the reflash happened — reasoning and rejected alternatives in
  [docs/info/setup.md](docs/info/setup.md).
- **The Pi is on Wi-Fi, not Ethernet.** `eth0` has no carrier; it reaches the LAN
  via `wlan0` on `THEKKEL_MESH`. Anything that assumes a wired link — a doc, an
  Ansible fact, a bandwidth estimate — is wrong. It also means a bad network config
  leaves the machine needing a keyboard and monitor, so treat network changes on
  the Pi as higher-risk than they look.
- **`v4l2-ctl` comes from the `camera` role, not the OS.** Ubuntu Server has no
  `v4l-utils`; Raspberry Pi OS did. It is installed on the Pi now (camera role,
  2026-07-24), but a fresh reflash loses it until the playbook runs — don't
  report a camera command as failing before checking this.
- **Ubuntu silently downgrades the Pi's bootloader.** Its `rpi-eeprom` package
  bundles only `pieeprom-2024-09-23.bin` and enables `rpi-eeprom-update.service`,
  so any firmware update applied from Raspberry Pi OS is reverted the moment
  `rpi-eeprom-config --apply` runs. Config keys survive; the version does not.
  `rpi-eeprom-update` reports Ubuntu's bundle as both CURRENT and LATEST, so read
  `/proc/device-tree/chosen/bootloader/build-timestamp` instead.
- **The SD card is pinned by PARTUUID, not label** (`5ec0ffee-01`/`-02`, in both
  `cmdline.txt` and `/etc/fstab`). Ubuntu's Pi image labels every copy
  `system-boot`/`writable`, so a second copy of the image collides on label,
  filesystem UUID *and* PARTUUID. Don't "simplify" these back to `LABEL=`.
- **The dev box has interfaces DDS must not bind to**: three Docker bridges
  (`172.17`–`172.19`), `tailscale0`, and a WireGuard interface named `laptop` at
  `10.8.0.3`. DDS will happily pick one instead of `enp6s18` and advertise an
  address the Pi cannot route to. The VPN interfaces are the nastier half — they
  look routable and are not. Pin via `CYCLONEDDS_URI` —
  [docs/info/networking.md](docs/info/networking.md).
- **`ROS_DOMAIN_ID=42`** on both machines. Non-default on purpose; `0` is shared
  with every other project on the LAN. It, `ROS_LOCALHOST_ONLY=0` and
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` must be identical on both hosts; they
  get one definition in Ansible `group_vars`, so change them there rather than in
  a `.bashrc`.
- **A non-interactive `ssh pi '...'` does not read the ROS environment.** The
  exports live in `~/.profile` (put there by the `ros2_env` role — `.bashrc`'s
  interactivity guard would hide them from every non-interactive shell), so
  `ssh pi 'ros2 topic list'` silently runs on domain 0 with the default RMW. Use
  a login shell — `ssh pi "bash -lc '...'"` — when verifying over SSH, and don't
  report such a result as evidence of anything without checking this first.
- **The Pi's user is in `video`** — set via cloud-init at reflash time, and
  `/dev/video0` is readable without `sudo`. But **`gpio`, `i2c`, and `spi` no
  longer exist** as groups; they were a Raspberry Pi OS vendor addition. Milestone
  7's servo option needs them created plus a udev rule, not just a `usermod`.
- **`/dev/video1` is not a capture device.** It is the C922's UVC metadata node.
  Capture is `/dev/video0` only.
- **The camera is confirmed working** (verified 2026-07-23 by capturing a frame and
  streaming). But on stock settings it runs at **18–21 fps, not 30** — the C922's
  `exposure_dynamic_framerate` trades frame rate for exposure in indoor light.
  Fixing the exposure gives a measured 30.00 fps. Never quote a frame-rate figure
  without stating the exposure mode it was measured under.
- **The "30 fps ceiling" fell on 2026-08-04.** 720p60 measured ~29.7 fps in
  July, but re-measured under the `just camera-reset` baseline it delivers
  **42–60 distinct frames/s** at true 1280×720 MJPG (0 duplicate payloads in
  634 messages), the rate tracking the auto-exposure time. What changed was
  never isolated; the flip coincides with the baseline clearing the camera's
  persistent control state. Budget every consumer for up to 60 fps —
  [docs/info/hardware.md](docs/info/hardware.md#capture-modes).
- **`usb_cam` needs `framerate:=60` to deliver the camera's real frame rate.**
  It grabs frames on a ROS timer at the requested rate, and at 30 that timer
  beats against the camera's cadence — measured 24.0 fps steady while raw V4L2
  capture delivered 30. It also mangles the by-id symlink; pass it through
  `readlink -f` — [docs/info/camera.md](docs/info/camera.md#running-it).
- **usb_cam's exposure/focus ROS parameters are dead on this kernel** — it uses
  ROS 1-era control names (`exposure_auto`) the kernel has renamed; `v4l2-ctl`
  is the only working channel for those controls. And **V4L2 controls persist
  inside the camera** across processes and reboots — a manual exposure left by
  a benchmark makes every later session black, and the C922 powers on with
  `exposure_dynamic_framerate=1` (the 18–21 fps thief) despite the driver
  reporting its default as 0. Treat camera state as inspectable machine
  state: `just camera` prints every control current-vs-default, and
  `just camera-reset` restores the known-good baseline — run them before
  diagnosing black frames or low fps as a software bug. Gain is never
  auto-adjusted on Linux; dim rooms need it raised (`just cam gain:=128`).
- **The dev box's `python3` is PlatformIO's venv**, which shadows the system
  Python for anything with an `#!/usr/bin/env python3` shebang — rqt tools
  crash with `No module named 'yaml'`. colcon-built nodes are immune (hardcoded
  shebang). Prefix `PATH="/usr/bin:$PATH"` for GUI tools —
  [docs/info/troubleshooting.md](docs/info/troubleshooting.md#rqt-tools-crash-with-no-module-named-yaml).
- **The dev box session is Wayland; Qt5 windows are invisible to X tools.**
  A Qt5 app (OpenCV's highgui included) opens a native Wayland surface unless
  `QT_QPA_PLATFORM=xcb` forces it through Xwayland — `xwininfo` sees nothing
  otherwise, and no `xdotool`/`wmctrl` is installed anyway. `just calibrate`'s
  window watchdog depends on this pin. **rviz2 needs the same pin to run at
  all** (OGRE renders via GLX, X11-only), and as of 2026-07-28 additionally
  needs `__GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1` until the
  next reboot — apt upgraded the NVIDIA userspace to 595.84 while the loaded
  kernel module is 595.71, breaking all hardware GL
  ([docs/info/troubleshooting.md](docs/info/troubleshooting.md#rviz2-crashes-unable-to-create-the-rendering-window-glxcontext-100-tries)).
- **Justfile cleanup traps must `pkill -f` node patterns, not `kill %N`.**
  Background jobs in recipes are `bash -lc` wrappers; killing the wrapper
  orphans the actual ros2-run grandchildren, which sit silent until a live
  stream feeds their subscription again (bit twice on 2026-07-27) —
  [docs/info/troubleshooting.md](docs/info/troubleshooting.md#orphaned-nodes-keep-logging-into-a-terminal-after-a-recipe-ends).
- **`/image_raw` header stamps lag wall clock by a steady ~0.73 s** — a
  UVC/driver timestamping fault (`ros2 topic delay /image_raw` shows it; the
  frames are live). Never gate freshness or report latency against
  `header.stamp` on this camera: a stamp-age gate silently dropped 100% of
  frames. Measure processing cost against one process's own clock only —
  [docs/info/camera.md](docs/info/camera.md#timestamps).
- **Never stream raw images across the LAN.** 1280×720 RGB8 @ 30 fps is ~83 MB/s.
  Use `image_transport` compressed topics — [docs/info/camera.md](docs/info/camera.md).
- **No CSI camera is attached.** `libcamera`/`rpicam` guidance does not apply; the
  C922 is a standard UVC device on V4L2.
- **Restart the ROS daemon** (`ros2 daemon stop && ros2 daemon start`) after
  changing any `ROS_*` or DDS environment variable. It caches discovery state and
  will otherwise report the stale view — this masks fixes that actually worked.

## Conventions

- This repo doubles as the colcon workspace; packages go in `src/`.
- Day-to-day commands are recipes in the `justfile` (`just` lists them by
  group: provision, sync, status, test, build). Add a recipe rather than
  documenting a long one-off command; keep recipes and docs in agreement.
- Provisioning lives in `ansible/` — `inventory.yml`, `group_vars/`, `roles/`, and
  `site.yml`. Machine-specific values belong in `group_vars`, never hard-coded in a
  role. See [docs/info/ansible.md](docs/info/ansible.md) for the intended layout.
- `rosdep init` is not idempotent and needs a `creates:` guard; `rosdep update` and
  `colcon build` must run as the login user, never under `become`/`sudo`.
- Build with `colcon build --symlink-install` so Python and launch edits apply
  without a rebuild.
- Tests run through colcon (`just test`) or the VSCode Testing sidebar; the
  sidebar needs three accommodations that already exist — `.vscode/ros.env`
  (the ROS python path; the sidebar's pytest has no login shell),
  `pytest.ini` (importlib mode + launch_testing plugins off), and per-package
  linter tests **anchored on `__file__`, not the CWD**. `ros2 pkg create`
  generates the CWD-dependent form: when adding a package, copy the anchored
  tests from an existing one and add the package to `.vscode/tasks.json`'s
  picker.
- **Camera handling rules are consolidated in
  [docs/info/camera.md#handling-rules](docs/info/camera.md#handling-rules)** —
  ownership (the Pi opens the device, dev-box launches must not), exclusive
  capture vs shared live controls, persistent camera state, fail-loudly,
  transport, and timestamp rules. Read them before writing any code, recipe
  or doc that touches the camera, and keep `just camera` /
  `just camera-reset` in agreement with them.
- **Camera consumers fail loudly.** Anything that opens or depends on the
  camera must verify it and exit nonzero with a clear message rather than
  idling: `camera.launch.py` pre-flight-checks the device (usb_cam itself
  logs one ERROR on a missing device and then idles forever) and puts
  `on_exit=Shutdown()` on the camera node; the camera recipes verify the
  launch survived warm-up, and `record`/`calibrate` check a stream exists
  before starting. Apply the same rule to any new node, launch file or
  recipe that uses the camera —
  [docs/info/camera.md](docs/info/camera.md#when-the-camera-is-missing).
- Package naming: `piros2_<thing>` (e.g. `piros2_camera`).
- Python packages use `ament_python`; C++ uses `ament_cmake`.
- Parameters belong in `config/*.yaml` and launch files in `launch/*.launch.py` —
  not baked into long `--ros-args` command lines.
- `build/`, `install/`, `log/`, and bag files are git-ignored.
- Prose in docs uses British-ish spelling consistent with the existing files;
  match the surrounding style rather than reformatting.
- **Docs are split by kind**: reference docs live in `docs/info/`, plans in
  `docs/plans/`. A new plan starts in `docs/plans/in-progress/` and moves to
  `docs/plans/completed/` when its work is done — moving the file *is* the
  status change, so fix inbound links when it moves. Don't create plan files
  outside `docs/plans/`.
- **Plans are structured as stable phases** (P0, P1, …), each ending with
  something runnable and checkable. Once a phase is written, its number and
  scope stay fixed — record progress by annotating the phase (dates, ✓ marks,
  what actually happened), never by renumbering or reshuffling phases, so
  that "P2" means the same thing in every doc, commit message, and
  conversation that mentions it.

## Syncing to the Pi

The repo lives on the dev box at `~/Documents/piros2`; the Pi keeps its own copy
at `~/piros2` and builds there. Keep them in step with:

```bash
rsync -av --delete --exclude build --exclude install --exclude log \
      --exclude src/piros2_perception/models \
      ~/Documents/piros2/ pi:~/piros2/
```

(The models exclude keeps the 99 MB depth weights off the Pi — inference is
dev-box-only.)

The Ansible `workspace` role does the same as part of a run. Remote is
`git@github.com:bthek1/piros2.git`.

## Documentation map

`docs/info/` holds reference docs; `docs/plans/` holds plans, sorted by status
into `in-progress/` and `completed/` (see Conventions).

| File | Contents |
| --- | --- |
| [README.md](README.md) | Overview and entry point |
| [docs/info/hardware.md](docs/info/hardware.md) | Measured specs of both machines and the camera's capture modes |
| [docs/info/setup.md](docs/info/setup.md) | Reflashing the Pi, provisioning both machines, rejected alternatives |
| [docs/info/ansible.md](docs/info/ansible.md) | The playbook: inventory, roles, gotchas |
| [docs/info/networking.md](docs/info/networking.md) | DDS discovery, domain IDs, interface pinning |
| [docs/info/camera.md](docs/info/camera.md) | Driver choice, transport, V4L2 controls, calibration |
| [docs/info/troubleshooting.md](docs/info/troubleshooting.md) | Symptom → cause |
| [docs/info/roadmap.md](docs/info/roadmap.md) | Milestones and their status |
| [docs/info/perception.md](docs/info/perception.md) | Perception design: camera → depth → point-cloud room map, what mono can honestly do |
| [docs/plans/completed/perception-plan.md](docs/plans/completed/perception-plan.md) | Perception build order — phases P0–P4, the `src/piros2_perception` package, per-phase proofs |
| [docs/plans/completed/world-plan.md](docs/plans/completed/world-plan.md) | Build order for `src/piros2_world` — camera/depth/keypoint feeds + live stats in one dashboard window; done 2026-08-03, kept as the build log |
| [docs/plans/in-progress/world-3d-plan.md](docs/plans/in-progress/world-3d-plan.md) | Camera orientation from keypoint matches + accumulated cloud map, both in RViz — phases P0–P4, not started |
| [docs/plans/completed/ansible-plan.md](docs/plans/completed/ansible-plan.md) | Build order for the `ansible/` tree — done 2026-07-24, kept as the build log |

When hardware facts change (camera replugged, Pi reflashed, IP moved), update
[docs/info/hardware.md](docs/info/hardware.md) from real command output and note the date.
