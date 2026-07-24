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
  One wart: an **unrelated pre-existing kernel/DKMS failure** makes every apt
  task (and so the playbook) report red on this host until fixed —
  [docs/troubleshooting.md](docs/troubleshooting.md#apt-fails-on-linux--kernel-packages-dev-box).
- **Milestone 0 passed 2026-07-24**: `/chatter` published on the Pi arrived on
  the dev box across the LAN. [docs/roadmap.md](docs/roadmap.md) tracks status;
  next is milestone 1, the first hand-written package.

No ROS packages exist yet; `src/` contains only a `.gitkeep` and both build
tasks in the `workspace` role are guarded on packages existing.

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

When hardware facts change (camera replugged, Pi reflashed, IP moved), update
[docs/hardware.md](docs/hardware.md) from real command output and note the date.
