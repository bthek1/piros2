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

Full measured specs: [docs/hardware.md](docs/hardware.md).

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
  the dev box across the LAN. [docs/roadmap.md](docs/roadmap.md) tracks status.

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
`tf2_echo`. Remaining: the RViz visual check (`just pipeline` + `rviz2`) and
checkerboard calibration. A first calibration session (2026-07-27 evening)
live-debugged `just calibrate` into working shape — three recipe bugs fixed
(dead `-p camera:=` service remap, window close ignored by the calibrator,
`kill %N` traps orphaning nodes; see docs/troubleshooting.md) — but ended
without a save, so `camera_info` is still empty and P0 stays open.

Milestone 6 (record/replay, done 2026-07-27): `just record` bags the
compressed stream + `camera_info` + `/tf_static` on the Pi (24 s ≈ 36 MiB
MCAP) and fetches it to `bags/`; `just replay` runs it through
`image_transport republish` into the edge detector entirely on the dev box —
no Pi needed. Jazzy's `republish` takes transports as *parameters*, not
positional args (see docs/troubleshooting.md).

**The roadmap concluded 2026-07-27.** The project now runs on
[docs/perception-plan.md](docs/perception-plan.md): phases P0–P4 building
`src/piros2_perception` (neural monocular depth → point clouds → a room map).
P0 is the human-gated calibration — still open, and it gates P2 (metres come
from the K matrix).

Perception P1 (depth node, done 2026-07-28): `src/piros2_perception` —
`depth_estimator` subscribes `/image_raw/compressed`, runs Depth Anything V2
Small (fp32 ONNX, `just fetch-model`, checksum-pinned, git-ignored) and
publishes `/depth` (32FC1) + a colourised preview; measured 280–305 ms/frame
on the dev-box CPU (~3 fps), verified against the milestone-6 bag. It is the
repo's documented venv escape hatch: onnxruntime is PyPI-only, lives in
`~/.venvs/piros2-perception` (`--system-site-packages`), and the node must
run as `python -m` under that interpreter — colcon's hardcoded shebang
misses the venv, so `ros2 run` gets no onnxruntime. `just depth` owns the
invocation; the package README explains it.

Testing: `just test` (colcon test + result aggregation) or the VSCode Testing
sidebar — both report identically. All packages are style-clean and the suite
is green (20 tests; `piros2_vision` and `piros2_perception` carry real unit
tests — the latter injects a fake ONNX session so no weights are needed) as
of 2026-07-28.

Don't write docs or code that imply a package exists when it does not. If a doc
describes something not yet built, mark it as planned — the existing docs follow
this convention and [docs/roadmap.md](docs/roadmap.md) tracks status.

## Constraints that are easy to get wrong

- **Both machines are now Ubuntu 24.04 noble**, Jazzy's Tier 1 platform, so
  `apt install ros-jazzy-ros-base` works on the Pi. `packages.ros.org` serves 3361
  `ros-jazzy-*` binaries for `noble/arm64` against **zero** for `bookworm/arm64`,
  which is why the reflash happened — reasoning and rejected alternatives in
  [docs/setup.md](docs/setup.md).
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
  [docs/networking.md](docs/networking.md).
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
- **720p60 does not work**, despite being advertised and negotiating successfully.
  It measures ~29.7 fps. 30 fps is the ceiling at every resolution.
- **`usb_cam` needs `framerate:=60` to deliver the camera's real 30 fps.** It
  grabs frames on a ROS timer at the requested rate, and at 30 that timer beats
  against the camera's 33 ms cadence — measured 24.0 fps steady while raw V4L2
  capture delivered 30. It also mangles the by-id symlink; pass it through
  `readlink -f` — [docs/camera.md](docs/camera.md#running-it).
- **usb_cam's exposure/focus ROS parameters are dead on this kernel** — it uses
  ROS 1-era control names (`exposure_auto`) the kernel has renamed; `v4l2-ctl`
  is the only working channel for those controls. And **V4L2 controls persist
  inside the camera** across processes and reboots — a manual exposure left by
  a benchmark makes every later session black. Gain is never auto-adjusted on
  Linux; dim rooms need it raised (`just cam gain:=128`).
- **The dev box's `python3` is PlatformIO's venv**, which shadows the system
  Python for anything with an `#!/usr/bin/env python3` shebang — rqt tools
  crash with `No module named 'yaml'`. colcon-built nodes are immune (hardcoded
  shebang). Prefix `PATH="/usr/bin:$PATH"` for GUI tools —
  [docs/troubleshooting.md](docs/troubleshooting.md#rqt-tools-crash-with-no-module-named-yaml).
- **The dev box session is Wayland; Qt5 windows are invisible to X tools.**
  A Qt5 app (OpenCV's highgui included) opens a native Wayland surface unless
  `QT_QPA_PLATFORM=xcb` forces it through Xwayland — `xwininfo` sees nothing
  otherwise, and no `xdotool`/`wmctrl` is installed anyway. `just calibrate`'s
  window watchdog depends on this pin.
- **Justfile cleanup traps must `pkill -f` node patterns, not `kill %N`.**
  Background jobs in recipes are `bash -lc` wrappers; killing the wrapper
  orphans the actual ros2-run grandchildren, which sit silent until a live
  stream feeds their subscription again (bit twice on 2026-07-27) —
  [docs/troubleshooting.md](docs/troubleshooting.md#orphaned-nodes-keep-logging-into-a-terminal-after-a-recipe-ends).
- **`/image_raw` header stamps lag wall clock by a steady ~0.73 s** — a
  UVC/driver timestamping fault (`ros2 topic delay /image_raw` shows it; the
  frames are live). Never gate freshness or report latency against
  `header.stamp` on this camera: a stamp-age gate silently dropped 100% of
  frames. Measure processing cost against one process's own clock only —
  [docs/camera.md](docs/camera.md#timestamps).
- **Never stream raw images across the LAN.** 1280×720 RGB8 @ 30 fps is ~83 MB/s.
  Use `image_transport` compressed topics — [docs/camera.md](docs/camera.md).
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
  role. See [docs/ansible.md](docs/ansible.md) for the intended layout.
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
- Package naming: `piros2_<thing>` (e.g. `piros2_camera`).
- Python packages use `ament_python`; C++ uses `ament_cmake`.
- Parameters belong in `config/*.yaml` and launch files in `launch/*.launch.py` —
  not baked into long `--ros-args` command lines.
- `build/`, `install/`, `log/`, and bag files are git-ignored.
- Prose in docs uses British-ish spelling consistent with the existing files;
  match the surrounding style rather than reformatting.

## Syncing to the Pi

The repo lives on the dev box at `~/Documents/piros2`; the Pi keeps its own copy
at `~/piros2` and builds there. Keep them in step with:

```bash
rsync -av --delete --exclude build --exclude install --exclude log \
      ~/Documents/piros2/ pi:~/piros2/
```

The Ansible `workspace` role does the same as part of a run. Remote is
`git@github.com:bthek1/piros2.git`.

## Documentation map

| File | Contents |
| --- | --- |
| [README.md](README.md) | Overview and entry point |
| [docs/hardware.md](docs/hardware.md) | Measured specs of both machines and the camera's capture modes |
| [docs/setup.md](docs/setup.md) | Reflashing the Pi, provisioning both machines, rejected alternatives |
| [docs/ansible.md](docs/ansible.md) | The playbook: inventory, roles, gotchas |
| [docs/ansible-plan.md](docs/ansible-plan.md) | Build order for the `ansible/` tree — working doc, delete once green |
| [docs/networking.md](docs/networking.md) | DDS discovery, domain IDs, interface pinning |
| [docs/camera.md](docs/camera.md) | Driver choice, transport, V4L2 controls, calibration |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Symptom → cause |
| [docs/roadmap.md](docs/roadmap.md) | Milestones and their status |
| [docs/perception.md](docs/perception.md) | Perception design: camera → depth → point-cloud room map, what mono can honestly do |
| [docs/perception-plan.md](docs/perception-plan.md) | Perception build order — phases P0–P4, the `src/piros2_perception` package, per-phase proofs |

When hardware facts change (camera replugged, Pi reflashed, IP moved), update
[docs/hardware.md](docs/hardware.md) from real command output and note the date.
