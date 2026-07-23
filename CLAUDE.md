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
| OS **now** | Ubuntu 24.04 LTS, x86_64 | Raspberry Pi OS / Debian 12 bookworm, aarch64 |
| OS **planned** | unchanged | **Ubuntu Server 24.04 LTS arm64** (reflash) |
| ROS | `ros-jazzy-desktop`, native apt (planned) | `ros-jazzy-ros-base`, native apt (planned) |
| Provisioning | Ansible control node (core 2.16.3, installed) | Ansible managed host |
| Runs | `rviz2`, `rqt`, editing, builds | camera node, anything touching hardware |
| Network | `192.168.2.106/24` on `eth2` | `192.168.2.17` |

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

The repository is **documentation only**. No ROS packages exist, and neither
machine has ROS installed yet. `src/` and `ansible/` have not been created, and
**the Pi has not been reflashed** — it is still on Raspberry Pi OS.

Don't write docs or code that imply a package exists when it does not. If a doc
describes something not yet built, mark it as planned — the existing docs follow
this convention and [docs/roadmap.md](docs/roadmap.md) tracks status. This applies
to the Ubuntu reflash in particular: [docs/hardware.md](docs/hardware.md) records
the Debian state as measured, with the Ubuntu column marked as pending.

## Constraints that are easy to get wrong

- **Native ROS 2 on the Pi requires the Ubuntu reflash first.** Debian bookworm is
  Tier 3 for Jazzy: `packages.ros.org` serves a `bookworm` suite containing **zero**
  `ros-jazzy-*` binaries (only bootstrap tooling), while `noble/arm64` serves 3361.
  So `apt install ros-jazzy-ros-base` works on Ubuntu and cannot work on the Pi's
  current OS. Until the card is reflashed, **any `apt` ROS instruction aimed at the
  Pi is wrong** — reasoning and rejected alternatives in
  [docs/setup-pi.md](docs/setup-pi.md).
- **The dev box has ~11 Docker bridge interfaces** (`172.17`–`172.26`). DDS will
  happily bind to one of them instead of `eth2` and advertise an unroutable
  address. Pin the interface via `CYCLONEDDS_URI` —
  [docs/networking.md](docs/networking.md).
  Dropping Docker from the Pi does **not** fix this — the bridges are on the dev box.
- **`ROS_DOMAIN_ID=42`** on both machines. Non-default on purpose; `0` is shared
  with every other project on the LAN. It, `ROS_LOCALHOST_ONLY=0` and
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` must be identical on both hosts; they
  get one definition in Ansible `group_vars`, so change them there rather than in
  a `.bashrc`.
- **A non-interactive `ssh pi '...'` does not read the ROS environment.** The
  exports live in the interactive part of `.bashrc`, so `ssh pi 'ros2 topic list'`
  silently runs on domain 0 with the default RMW. Set the vars inline or use a
  login shell when verifying over SSH — and don't report such a result as evidence
  of anything without checking this first.
- **After the reflash the Pi's user is not in `video`.** Raspberry Pi OS put it
  there; Ubuntu will not. `/dev/video0` will be permission-denied until fixed, and
  a group change needs a fresh login to take effect.
- **`/dev/video1` is not a capture device.** It is the C922's UVC metadata node.
  Capture is `/dev/video0` only.
- **The camera is confirmed working** (verified 2026-07-23 by capturing a frame and
  streaming). But on stock settings it runs at **18–21 fps, not 30** — the C922's
  `exposure_dynamic_framerate` trades frame rate for exposure in indoor light.
  Fixing the exposure gives a measured 30.00 fps. Never quote a frame-rate figure
  without stating the exposure mode it was measured under.
- **720p60 does not work**, despite being advertised and negotiating successfully.
  It measures ~29.7 fps. 30 fps is the ceiling at every resolution.
- **Never stream raw images across the LAN.** 1280×720 RGB8 @ 30 fps is ~83 MB/s.
  Use `image_transport` compressed topics — [docs/camera.md](docs/camera.md).
- **No CSI camera is attached.** `libcamera`/`rpicam` guidance does not apply; the
  C922 is a standard UVC device on V4L2.
- **Restart the ROS daemon** (`ros2 daemon stop && ros2 daemon start`) after
  changing any `ROS_*` or DDS environment variable. It caches discovery state and
  will otherwise report the stale view — this masks fixes that actually worked.

## Conventions

- This repo doubles as the colcon workspace; packages go in `src/`.
- Provisioning lives in `ansible/` — `inventory.yml`, `group_vars/`, `roles/`, and
  `site.yml`. Machine-specific values belong in `group_vars`, never hard-coded in a
  role. See [docs/ansible.md](docs/ansible.md) for the intended layout.
- `rosdep init` is not idempotent and needs a `creates:` guard; `rosdep update` and
  `colcon build` must run as the login user, never under `become`/`sudo`.
- Build with `colcon build --symlink-install` so Python and launch edits apply
  without a rebuild.
- Package naming: `piros2_<thing>` (e.g. `piros2_camera`).
- Python packages use `ament_python`; C++ uses `ament_cmake`.
- Parameters belong in `config/*.yaml` and launch files in `launch/*.launch.py` —
  not baked into long `--ros-args` command lines.
- `build/`, `install/`, `log/`, and bag files are git-ignored.
- Prose in docs uses British-ish spelling consistent with the existing files;
  match the surrounding style rather than reformatting.

## Syncing to the Pi

The repo lives on the dev box; the Pi keeps its own copy at `~/piros2` and builds
there. Keep them in step with:

```bash
rsync -av --delete --exclude build --exclude install --exclude log \
      ~/piros2/ pi:~/piros2/
```

The Ansible `workspace` role does the same as part of a run. Remote is
`git@github.com:bthek1/piros2.git`.

## Documentation map

| File | Contents |
| --- | --- |
| [README.md](README.md) | Overview and entry point |
| [docs/hardware.md](docs/hardware.md) | Measured specs of both machines and the camera's capture modes |
| [docs/setup-dev.md](docs/setup-dev.md) | ROS 2 Jazzy on Ubuntu 24.04 |
| [docs/setup-pi.md](docs/setup-pi.md) | Reflashing the Pi to Ubuntu 24.04 for native ROS; rejected alternatives |
| [docs/ansible.md](docs/ansible.md) | Provisioning both machines from one playbook |
| [docs/networking.md](docs/networking.md) | DDS discovery, domain IDs, interface pinning |
| [docs/camera.md](docs/camera.md) | Driver choice, transport, V4L2 controls, calibration |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Symptom → cause |
| [docs/roadmap.md](docs/roadmap.md) | Milestones and their status |

When hardware facts change (camera replugged, Pi reflashed, IP moved), update
[docs/hardware.md](docs/hardware.md) from real command output and note the date.
