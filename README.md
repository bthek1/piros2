# piros2

A learning project: **ROS 2 on a Raspberry Pi 5 with a Logitech C922 webcam.**

The goal is to get hands-on with ROS 2 fundamentals — nodes, topics, launch files,
parameters, TF, and image pipelines — using real hardware rather than simulation.

## The setup

| Role | Host | What it is |
| --- | --- | --- |
| Robot / sensor node | `pi` → `192.168.2.17` | Raspberry Pi 5 (8 GB), Logitech C922 on `/dev/video0`. Currently Raspberry Pi OS; **to be reflashed to Ubuntu Server 24.04 arm64** |
| Dev / visualisation | `test` → `192.168.2.106` | Ubuntu 24.04 LTS, x86_64 — runs `rviz2`, `rqt`, and the editor |

Both machines run ROS 2 natively from `apt`, provisioned by a shared set of
Ansible roles.

Both machines sit on the same `192.168.2.0/24` LAN, so ROS 2 nodes discover each
other over DDS without a central master.

**ROS distro: Jazzy Jalisco** (the current LTS, supported until May 2029).

## Status

The repository is currently **documentation only** — no ROS packages have been
written yet, and neither machine has ROS installed. The docs below describe the
verified hardware and the setup path to follow.

See [docs/roadmap.md](docs/roadmap.md) for what is planned and in what order.

## Quick start

1. Install ROS 2 Jazzy on the dev box — [docs/setup-dev.md](docs/setup-dev.md)
2. Reflash the Pi to Ubuntu 24.04 and install ROS natively — [docs/setup-pi.md](docs/setup-pi.md)
3. Capture both as Ansible roles — [docs/ansible.md](docs/ansible.md)
4. Make the two talk to each other — [docs/networking.md](docs/networking.md)
5. Publish camera frames — [docs/camera.md](docs/camera.md)

## Documentation

| Doc | Contents |
| --- | --- |
| [hardware.md](docs/hardware.md) | Verified specs of the Pi, the dev box, and the C922's capture modes |
| [setup-dev.md](docs/setup-dev.md) | ROS 2 Jazzy on the Ubuntu 24.04 dev box (native apt install) |
| [setup-pi.md](docs/setup-pi.md) | Reflashing the Pi to Ubuntu 24.04 for a native ROS 2 install, and why |
| [ansible.md](docs/ansible.md) | Provisioning both machines from one playbook |
| [networking.md](docs/networking.md) | DDS discovery across the LAN, domain IDs, and the Docker-bridge gotcha |
| [camera.md](docs/camera.md) | Driver choice, capture modes, image transport, calibration |
| [troubleshooting.md](docs/troubleshooting.md) | Symptoms → causes for the failures you are most likely to hit |
| [roadmap.md](docs/roadmap.md) | The learning path, milestone by milestone |

## Why this shape

ROS 2 Jazzy publishes binaries for **Ubuntu 24.04 noble** on both `amd64` and
`arm64` — 3373 and 3361 `ros-jazzy-*` packages respectively. For Debian 12
bookworm, which Raspberry Pi OS is built on, it publishes **zero**.

So the Pi is reflashed to Ubuntu Server 24.04 rather than worked around. Both
machines then run the same native `apt` install, share one set of Ansible roles,
and match every tutorial and error message you will search for. The container and
build-from-source alternatives were considered and rejected — the reasoning is in
[setup-pi.md](docs/setup-pi.md).

The dev box gets `ros-jazzy-desktop` so GUI tools like `rviz2` run without
X-forwarding gymnastics; the Pi gets the much smaller `ros-jazzy-ros-base`, since
it is a sensor head with no reason to carry the Qt stack.
