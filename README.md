# piros2

A learning project: **ROS 2 on a Raspberry Pi 5 with a Logitech C922 webcam.**

The goal is to get hands-on with ROS 2 fundamentals — nodes, topics, launch files,
parameters, TF, and image pipelines — using real hardware rather than simulation.

## The setup

| Role | Host | What it is |
| --- | --- | --- |
| Robot / sensor node | `pi` → `192.168.2.17` | Raspberry Pi 5 (8 GB), Raspberry Pi OS 64-bit (Debian 12 bookworm), Logitech C922 on `/dev/video0` |
| Dev / visualisation | `test` → `192.168.2.106` | Ubuntu 24.04 LTS, x86_64 — runs `rviz2`, `rqt`, and the editor |

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
2. Install Docker + the ROS 2 container on the Pi — [docs/setup-pi.md](docs/setup-pi.md)
3. Make the two talk to each other — [docs/networking.md](docs/networking.md)
4. Publish camera frames — [docs/camera.md](docs/camera.md)

## Documentation

| Doc | Contents |
| --- | --- |
| [hardware.md](docs/hardware.md) | Verified specs of the Pi, the dev box, and the C922's capture modes |
| [setup-dev.md](docs/setup-dev.md) | ROS 2 Jazzy on the Ubuntu 24.04 dev box (native apt install) |
| [setup-pi.md](docs/setup-pi.md) | ROS 2 Jazzy on the Pi via Docker, and why native apt is not an option |
| [networking.md](docs/networking.md) | DDS discovery across the LAN, domain IDs, and the Docker-bridge gotcha |
| [camera.md](docs/camera.md) | Driver choice, capture modes, image transport, calibration |
| [troubleshooting.md](docs/troubleshooting.md) | Symptoms → causes for the failures you are most likely to hit |
| [roadmap.md](docs/roadmap.md) | The learning path, milestone by milestone |

## Why this shape

The Pi runs Raspberry Pi OS rather than Ubuntu, which rules out a native `apt`
install of ROS 2 — there are no `ros-jazzy-*` binaries for Debian bookworm.
Running ROS in a container on the Pi keeps the host OS untouched (its camera
stack, GPIO tooling, and vendor kernel all stay stock) while giving a stock ROS 2
environment. The dev box, being Ubuntu 24.04, gets ROS natively so that GUI tools
like `rviz2` run without X-forwarding gymnastics.

See [setup-pi.md](docs/setup-pi.md) for the full reasoning and the alternatives
that were considered.
