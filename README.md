# piros2

A learning project: **ROS 2 on a Raspberry Pi 5 with a Logitech C922 webcam.**

The goal is to get hands-on with ROS 2 fundamentals — nodes, topics, launch files,
parameters, TF, and image pipelines — using real hardware rather than simulation.

## The setup

| Role | Host | What it is |
| --- | --- | --- |
| Robot / sensor node | `pi` → `192.168.2.17` (Wi-Fi) | Raspberry Pi 5 (8 GB), Logitech C922 on `/dev/video0`. **Ubuntu Server 24.04 arm64** since 2026-07-23 |
| Dev / visualisation | `ml5` → `192.168.2.109` | Ubuntu 24.04 LTS, x86_64, GNOME desktop — runs `rviz2`, `rqt`, and the editor |

Both machines run ROS 2 natively from `apt`, provisioned by a shared set of Ansible
roles, and sit on the same `192.168.2.0/24` LAN — so nodes discover each other over
DDS without a central master.

**ROS distro: Jazzy Jalisco** (the current LTS, supported until May 2029).

## Status

**Milestone 0 is done** (2026-07-24): both machines are provisioned by the
Ansible playbook in `ansible/` — ROS 2 Jazzy `ros-base` + camera stack on the
Pi, `desktop` on the dev box — and a `talker` on the Pi reached a listener on
the dev box across the LAN on domain 42 with CycloneDDS pinned per host.

Groundwork before that: the Pi was **reflashed to Ubuntu Server 24.04 on
2026-07-23**, which is what makes the native `apt` install possible. No ROS
packages have been written yet — `src/` is empty until milestone 1.

See [docs/roadmap.md](docs/roadmap.md) for what is planned and in what order.

## Quick start

1. Provision both machines — [docs/setup.md](docs/setup.md) *(the Pi's reflash, step 1–3, is done)*
2. Write the playbook that does it — [docs/ansible.md](docs/ansible.md)
3. Make the two talk to each other — [docs/networking.md](docs/networking.md)
4. Publish camera frames — [docs/camera.md](docs/camera.md)

## Documentation

| Doc | Contents |
| --- | --- |
| [hardware.md](docs/hardware.md) | Verified specs of the Pi, the dev box, and the C922's capture modes |
| [setup.md](docs/setup.md) | Reflashing the Pi, provisioning both machines, and why Ubuntu on both |
| [ansible.md](docs/ansible.md) | The playbook: inventory, roles, and the gotchas |
| [networking.md](docs/networking.md) | DDS discovery across the LAN, domain IDs, and the Docker-bridge gotcha |
| [camera.md](docs/camera.md) | Driver choice, capture modes, image transport, calibration |
| [troubleshooting.md](docs/troubleshooting.md) | Symptoms → causes for the failures you are most likely to hit |
| [roadmap.md](docs/roadmap.md) | The learning path, milestone by milestone |

## Why this shape

ROS 2 Jazzy publishes binaries for **Ubuntu 24.04 noble** on both `amd64` and
`arm64` — 3373 and 3361 `ros-jazzy-*` packages respectively. For Debian 12
bookworm, which Raspberry Pi OS is built on, it publishes **zero**.

So the Pi was reflashed to Ubuntu Server 24.04 rather than worked around. Both
machines now run the same native `apt` install, share one set of Ansible roles,
and match every tutorial and error message you will search for. The container and
build-from-source alternatives were considered and rejected — the reasoning is in
[setup.md](docs/setup.md).

The dev box gets `ros-jazzy-desktop` so GUI tools like `rviz2` run without
X-forwarding gymnastics; the Pi gets the much smaller `ros-jazzy-ros-base`, since
it is a sensor head with no reason to carry the Qt stack.
