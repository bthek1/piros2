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
(then the `just run` target) starts the camera on the Pi and opens the one-window
view in `rqt_image_view`. (The mosaic retired in stages: its window on
2026-08-05, the topic itself on 2026-08-12 — the stats panel lives on as
the dashboard's output, in RViz.) Since **2026-08-04** the detector also matches
descriptors against a 10-frame window — re-observed keypoints drawn green,
new ones yellow, matched rate on the dashboard — feeding the
[world 3D plan](docs/plans/completed/world-3d-plan.md) (**done
2026-08-05**): rotation-only camera orientation from those matches —
strict frame-pair matches unprojected to bearing rays, the rotation
solved by Kabsch, composed and broadcast as `odom → base_link` — plus the
depth clouds accumulated into a persistent voxel panorama on
`/world/map_points`. Since the combined-plan merge (**2026-08-05**,
[world-combined-plan.md](docs/plans/completed/world-combined-plan.md))
one command — `just world`, the `just run` target until 2026-08-15 — opens one RViz
window: axes, live cloud and the accumulating map panorama in one 3D
scene, with raw camera, keypoints, depth and stats image panels docked
alongside;
reset/clear services stand in for loop closure, and the honest scope is
a panorama from one viewpoint, not a walkable map.

The [world fusion plan](docs/plans/completed/world-fusion-plan.md)
(**done 2026-08-10**) turned that map into real fusion — per-voxel
weighted averages live, plus an offline pipeline (`tools/recon/`,
Open3D under the perception venv) from bag to TSDF mesh to a
plane-labelled `room.json`, with the depth scale pinned by tape measure
(`depth_scale: 2.69`, +0.1% on verification). The live mesh work
(**2026-08-11**; its plan file was retired 2026-08-15, absorbed by the
world mesh plan below) then brought the surface live: `tsdf_mesher`
integrates depth in-session and re-meshes onto `/world/mesh_live`
every ~10 s (the LiveMesh display in the same RViz window), a
high-pass scale aligner halves the depth model's per-frame wobble, and
`just dev odom:=rgbd` optionally swaps the rotation-only compass for
RTAB-Map's live 6-DoF odometry. The
[world mesh plan](docs/plans/completed/world-mesh-plan.md)
(**2026-08-12, closed by decision 2026-08-15** — the live sweep
checks moved to the todo list) forks the world stack into its own
package, `piros2_world_mesh`, run as `just world_mesh` — aliased
**`just dev`** and, since 2026-08-15, **`just run`**, making it the
day-to-day target — with 6-DoF odometry and quality-biased
TSDF settings by default, and `just mesh-save` writing the live
surface to `meshes/live_<stamp>.ply` mid-session. The fork's first
real divergence landed the same day: it drops `cloud_mapper` — the
TSDF is its fusion accumulator — while `just world` keeps the voxel
panorama.

Reliability groundwork
([wifi-watchdog-plan.md](docs/plans/completed/wifi-watchdog-plan.md),
**done 2026-08-12** after the Pi's Wi-Fi died twice while the OS ran on):
the Ansible `wifi` role gives the Pi an escalation-ladder watchdog
(reassociate → driver reload → guarded reboot — drilled to unaided
recovery in ~7 min against the mesh's real `status_code=16` rejection),
and a dead link now reaps the camera session instead of orphaning it
against `/dev/video0`. `just wifi` shows link health.

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
| [just_world_mesh_diagrams.html](docs/info/just_world_mesh_diagrams.html) | The `just world_mesh` session drawn four ways — dataflow, deployment, TF tree, lifecycle — plus topic/service/cost reference tables (open in a browser; mermaid renders via CDN) |
| [roadmap.md](docs/info/roadmap.md) | The learning path, milestone by milestone — concluded 2026-07-27 |
| [perception.md](docs/info/perception.md) | Perception design: camera → depth → point-cloud room map |
| [perception-plan.md](docs/plans/completed/perception-plan.md) | Perception build order, phases P0–P4 — closed 2026-07-29, kept as the build log |
| [world-plan.md](docs/plans/completed/world-plan.md) | Build order for `src/piros2_world`, the one-window dashboard — done 2026-08-03, kept as the build log |
| [world-3d-plan.md](docs/plans/completed/world-3d-plan.md) | Camera orientation from keypoint matches + an accumulated cloud map in RViz — done 2026-08-05, kept as the build log |
| [world-combined-plan.md](docs/plans/completed/world-combined-plan.md) | One command, three windows: dashboard mosaic + orientation RViz + map RViz — done 2026-08-05, kept as the build log |
| [world-fusion-plan.md](docs/plans/completed/world-fusion-plan.md) | Learning plan for TSDF fusion, pose graphs and meshing; upgraded the cloud map to weighted fusion, built the offline recon pipeline, pinned the depth scale — done 2026-08-10, kept as the build log |
| [world-mesh-plan.md](docs/plans/completed/world-mesh-plan.md) | `piros2_world_mesh` (`just world_mesh`, aliased `just dev` and `just run`): the world stack forked into its own package and diverged mesh-first — 6-DoF odometry by default, quality-biased TSDF, a saved PLY at the end — built 2026-08-12, closed by decision 2026-08-15, kept as the build log |
| [wifi-watchdog-plan.md](docs/plans/completed/wifi-watchdog-plan.md) | The Pi heals its own Wi-Fi link: escalation-ladder watchdog, outage-reaped camera sessions, `just wifi` — done 2026-08-12, kept as the build log |
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
