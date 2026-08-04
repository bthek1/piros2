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

**The roadmap — milestones 0 through 6 — concluded on 2026-07-27**: both
machines provisioned by the Ansible playbook in `ansible/` (ROS 2 Jazzy
`ros-base` + camera stack on the Pi, `desktop` on the dev box, domain 42,
CycloneDDS pinned per host); `src/piros2_hello`, a hand-written
talker/listener pair verified across the LAN (`just hello`); the C922
streaming from the Pi at a measured ~30 fps over compressed transport with
its configuration owned by `src/piros2_camera`'s launch file and YAML
(`just cam`); `src/piros2_vision`, a Canny edge detector with a composed
launch file and unit tests (`just edges`); the static TF chain
`base_link → camera_link → camera_optical_frame`; and bag record/replay so
the pipeline iterates without hardware (`just record` / `just replay`).
[docs/info/roadmap.md](docs/info/roadmap.md) is the milestone-by-milestone log.

After it, the [perception plan](docs/plans/completed/perception-plan.md)
built `src/piros2_perception` — camera → neural depth → point clouds —
and was **closed 2026-07-29**, before its final mapping phase produced a
room map:

- **P0 (2026-07-28)** — camera intrinsics live on `/camera_info`,
  spec-derived (fx = fy ≈ 907 px); a checkerboard run remains as an
  accuracy upgrade.
- **P1 (2026-07-28)** — `depth_estimator`: Depth Anything V2 Small (ONNX)
  turns the compressed stream into `/depth` (`just depth`); ~3 fps on the
  dev-box CPU then, ~13 fps on its GPU since 2026-07-30.
- **P2 (2026-07-28)** — `cloud_projector`: depth + intrinsics into a
  coloured `PointCloud2`, verified live in RViz, correctly posed in
  `base_link` (`just cloud`).
- **P3 (plumbing verified 2026-07-29; plan closed here)** — RTAB-Map RGB-D
  mapping. `just map` ran a static bag through depth → odometry → RTAB-Map
  (odometry quality ~450–560). The hand-held sweep, the tuning loop, and
  therefore the map itself were not run — the plan records how to resume.

After that, the [world plan](docs/plans/completed/world-plan.md) built
`src/piros2_world` (**done 2026-08-03**): an ORB keypoint detector plus a
dashboard node that composes the camera feed, the neural depth preview,
and the annotated keypoints into one 2×2 mosaic with a live stats panel —
per-stream rates measured against the node's own clock, cumulative counts,
and staleness banners — published as a single compressed topic. `just world`
(the new `just run`) starts the camera on the Pi and opens the one-window
view in `rqt_image_view`. Since **2026-08-04** the detector also matches
descriptors against a 10-frame window — re-observed keypoints drawn green,
new ones yellow, matched count on the dashboard — the first step toward the
[world 3D plan](docs/plans/in-progress/world-3d-plan.md) (**in progress**):
rotation-only camera orientation from those matches, plus the depth clouds
accumulated into an RViz panorama.

Groundwork before all of it: the Pi was **reflashed to Ubuntu Server 24.04 on
2026-07-23**, which is what makes the native `apt` install possible.

## Quick start

1. Provision both machines — [docs/info/setup.md](docs/info/setup.md) *(the Pi's reflash, step 1–3, is done)*
2. Write the playbook that does it — [docs/info/ansible.md](docs/info/ansible.md)
3. Make the two talk to each other — [docs/info/networking.md](docs/info/networking.md)
4. Publish camera frames — [docs/info/camera.md](docs/info/camera.md)

## Documentation

Reference docs live in [docs/info/](docs/info/); build plans live in
[docs/plans/](docs/plans/), filed under `in-progress/` or `completed/` by
status.

| Doc | Contents |
| --- | --- |
| [hardware.md](docs/info/hardware.md) | Verified specs of the Pi, the dev box, and the C922's capture modes |
| [setup.md](docs/info/setup.md) | Reflashing the Pi, provisioning both machines, and why Ubuntu on both |
| [ansible.md](docs/info/ansible.md) | The playbook: inventory, roles, and the gotchas |
| [networking.md](docs/info/networking.md) | DDS discovery across the LAN, domain IDs, and the Docker-bridge gotcha |
| [camera.md](docs/info/camera.md) | Driver choice, capture modes, image transport, calibration |
| [troubleshooting.md](docs/info/troubleshooting.md) | Symptoms → causes for the failures you are most likely to hit |
| [roadmap.md](docs/info/roadmap.md) | The learning path, milestone by milestone — concluded 2026-07-27 |
| [perception.md](docs/info/perception.md) | Perception design: camera → depth → point-cloud room map |
| [perception-plan.md](docs/plans/completed/perception-plan.md) | Perception build order, phases P0–P4 — closed 2026-07-29, kept as the build log |
| [world-plan.md](docs/plans/completed/world-plan.md) | Build order for `src/piros2_world`, the one-window dashboard — done 2026-08-03, kept as the build log |
| [world-3d-plan.md](docs/plans/in-progress/world-3d-plan.md) | Camera orientation from keypoint matches + an accumulated cloud map in RViz — in progress, phases P0–P4 |
| [ansible-plan.md](docs/plans/completed/ansible-plan.md) | Build order for the `ansible/` tree — completed, kept as the build log |

## Why this shape

ROS 2 Jazzy publishes binaries for **Ubuntu 24.04 noble** on both `amd64` and
`arm64` — 3373 and 3361 `ros-jazzy-*` packages respectively. For Debian 12
bookworm, which Raspberry Pi OS is built on, it publishes **zero**.

So the Pi was reflashed to Ubuntu Server 24.04 rather than worked around. Both
machines now run the same native `apt` install, share one set of Ansible roles,
and match every tutorial and error message you will search for. The container and
build-from-source alternatives were considered and rejected — the reasoning is in
[setup.md](docs/info/setup.md).

The dev box gets `ros-jazzy-desktop` so GUI tools like `rviz2` run without
X-forwarding gymnastics; the Pi gets the much smaller `ros-jazzy-ros-base`, since
it is a sensor head with no reason to carry the Qt stack.
