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
| OS | Ubuntu 24.04 LTS, x86_64 | Raspberry Pi OS / Debian 12 bookworm, aarch64 |
| ROS | native apt install (planned) | inside Docker (planned) |
| Runs | `rviz2`, `rqt`, editing, builds | camera node, anything touching hardware |
| Network | `192.168.2.106/24` on `eth2` | `192.168.2.17` |

The Pi is reachable non-interactively, so **verify claims about the hardware by
running commands over SSH** rather than assuming. For example:

```bash
ssh pi 'v4l2-ctl --list-devices'
ssh pi 'ls -l /dev/video0'
```

Full measured specs: [docs/hardware.md](docs/hardware.md).

## Current state

The repository is **documentation only**. No ROS packages exist, and neither
machine has ROS installed yet. `src/` has not been created.

Don't write docs or code that imply a package exists when it does not. If a doc
describes something not yet built, mark it as planned — the existing docs follow
this convention and [docs/roadmap.md](docs/roadmap.md) tracks status.

## Constraints that are easy to get wrong

- **No native ROS 2 on the Pi.** Debian bookworm is Tier 3 for Jazzy; `packages.ros.org`
  serves a `bookworm` suite but it contains **zero** `ros-jazzy-*` binaries (only
  bootstrap tooling). `apt install ros-jazzy-ros-base` on the Pi will not work.
  Docker is the chosen path — reasoning and alternatives in
  [docs/setup-pi.md](docs/setup-pi.md).
- **The container must use `network_mode: host`.** DDS discovery does not survive a
  Docker bridge. This is the top cause of "the Pi publishes but the dev box sees
  nothing".
- **The dev box has ~11 Docker bridge interfaces** (`172.17`–`172.26`). DDS will
  happily bind to one of them instead of `eth2` and advertise an unroutable
  address. Pin the interface via `CYCLONEDDS_URI` —
  [docs/networking.md](docs/networking.md).
- **`ROS_DOMAIN_ID=42`** on both machines. Non-default on purpose; `0` is shared
  with every other project on the LAN.
- **`/dev/video1` is not a capture device.** It is the C922's UVC metadata node.
  Capture is `/dev/video0` only.
- **Never stream raw images across the LAN.** 1280×720 RGB8 @ 30 fps is ~83 MB/s.
  Use `image_transport` compressed topics — [docs/camera.md](docs/camera.md).
- **No CSI camera is attached.** `libcamera`/`rpicam` guidance does not apply; the
  C922 is a standard UVC device on V4L2.
- **Restart the ROS daemon** (`ros2 daemon stop && ros2 daemon start`) after
  changing any `ROS_*` or DDS environment variable. It caches discovery state and
  will otherwise report the stale view — this masks fixes that actually worked.

## Conventions

- This repo doubles as the colcon workspace; packages go in `src/`.
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

The repo lives on the dev box; the Pi mounts its own copy into the container.
Keep them in step with:

```bash
rsync -av --delete --exclude build --exclude install --exclude log \
      ~/piros2/ pi:~/piros2/
```

Remote is `git@github.com:bthek1/piros2.git`.

## Documentation map

| File | Contents |
| --- | --- |
| [README.md](README.md) | Overview and entry point |
| [docs/hardware.md](docs/hardware.md) | Measured specs of both machines and the camera's capture modes |
| [docs/setup-dev.md](docs/setup-dev.md) | ROS 2 Jazzy on Ubuntu 24.04 |
| [docs/setup-pi.md](docs/setup-pi.md) | Docker + ROS on the Pi; why native apt is out |
| [docs/networking.md](docs/networking.md) | DDS discovery, domain IDs, interface pinning |
| [docs/camera.md](docs/camera.md) | Driver choice, transport, V4L2 controls, calibration |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Symptom → cause |
| [docs/roadmap.md](docs/roadmap.md) | Milestones and their status |

When hardware facts change (camera replugged, Pi reflashed, IP moved), update
[docs/hardware.md](docs/hardware.md) from real command output and note the date.
