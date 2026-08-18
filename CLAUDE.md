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

**The `ansible/` tree is built and working** (2026-07-24): `site.yml` plus six
roles — `ros2_apt`, `ros2_install`, `ros2_env`, `camera`, `workspace`, and
`wifi` (added 2026-08-12) — and the playbook is idempotent (a clean rerun
reports `changed=0` on the Pi).

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
contrast with `cloud_projector` is the lesson — rendered by a 10 Hz
wall-timer into the stats panel on `/world/stats/compressed`; its
original 2×2 mosaic on `/world/dashboard/compressed` was removed
2026-08-12 after sitting subscriber-less since the RViz per-feed panels
took over, and with it went the only reason to decode incoming frames —
the node now counts arrivals without decoding anything). All rates and
the STALE lines are
measured against the dashboard's own receipt clock, never `header.stamp`
(the 0.73 s camera fault). `world.launch.py` follows the
perception-launch rules (venv `ExecuteProcess` for depth, no camera
include); `just world` runs it live and became the `just run` target
(until 2026-08-15, when `run` retargeted to `world_mesh`).
The live run surfaced a finding, verified 2026-08-04: the camera panel
reads 42–60 msgs/s, not ~30 — all distinct frames; the C922 genuinely
exceeds the old "30 fps ceiling" now (see the 720p60 bullet below).
Throttling is left for the "reduce compute" todo.

The world 3D build (done 2026-08-05, the whole
[world 3D plan](docs/plans/completed/world-3d-plan.md) P0–P4 across two
days; per-phase build log in the plan): rotation-only camera orientation
from the keypoint matches, and the depth clouds accumulated into an RViz
panorama. The detector estimates per-frame rotation from strict
consecutive-pair matches (Kabsch on bearing rays — the essential matrix
is degenerate under pure rotation), composes it, and publishes
`/camera/orientation` plus `odom → base_link` TF in base_link axes via
the canonical optical conjugation; byte-identical usb_cam duplicates are
CRC-skipped whole. `cloud_mapper` — the repo's first TF consumer,
latest-only lookups per the stamp fault — accumulates `/points` into a
voxel map (5 cm, hard-capped, `max_range` 6 m; weighted-average fusion
since the world fusion plan's P2, see below) and
republishes `/world/map_points` at 1 Hz in `odom`. The repo's first
services: `/keypoint_detector/reset` and `/cloud_mapper/clear` — the
drift strategy in lieu of loop closure; honest scope is orientation
without position (a panorama, not a walkable map — RTAB-Map keeps that
job). `just orient` (axes only) runs the lightweight session; the full
stack runs under `just world` since the combined-plan merge (below). All
phases were bag-verified against `bags/static1` (orientation within
~0.001° of identity on a static scene, map ≈ live cloud) and
live-verified by hand pan/sweep.

The combined session (done 2026-08-05, the
[world combined plan](docs/plans/completed/world-combined-plan.md) in
one sitting): `world.launch.py` is now the whole dev-box stack — all
five nodes, six since 2026-08-11 added the live mesh plan's
`tsdf_mesher` (next paragraphs). A seventh, `mesh_marker` (a latched
TRANSIENT_LOCAL Marker pointing RViz at the newest offline-fused
`meshes/*.ply` via `file://` mesh_resource, shown as FusedMesh), lived
one day: added 2026-08-11, removed 2026-08-12 once the live mesh made
the offline overlay redundant — its measured findings (RViz's assimp
loads PLY but rejects Open3D's GLB; RViz caches `mesh_resource` by URI)
are recorded here — the live-mesh plan file that first logged them was
retired 2026-08-15. `just world` (the `just run`
target until 2026-08-15) opens one
RViz window (`world.rviz`, renamed from `orient.rviz` once it became the
whole session): the Depth3D live cloud + CloudMap panorama + LiveMesh
surface + TF axes in one 3D scene, each toggleable in Displays, plus raw
camera / keypoints / depth / stats image panels docked alongside. Closing it tears everything
down. The stats panel is fed by
`/world/stats/compressed` — since 2026-08-12 the dashboard's only
output (the mosaic removal, below). The redundant
`world3d.launch.py`, `world.rviz` and `just world3d` recipe were deleted
after the merged session was proven live. The layout iterated through
the day (recorded in the plan): three windows → a rejected
whole-mosaic-in-RViz variant → the rqt mosaic window retired in favour
of per-feed panels (stats via `/world/stats/compressed`) → the map
display merged into the orientation scene and the map window (and
`map.rviz`) removed. `/world/dashboard/compressed` outlived its window
as a courtesy topic until 2026-08-12, when a check found it publishing
2×2 mosaics to nobody at 10 Hz and it was removed outright — the
dashboard now renders only the stats panel. `just orient` stays as the
lightweight no-GPU session — it opens the same `world.rviz`, so its
Depth3D/CloudMap/DepthPreview/Stats slots sit empty there by design
(only the detector runs; empty panels are honesty, not a fault).

The world fusion plan (done 2026-08-10, all in one day —
[docs/plans/completed/world-fusion-plan.md](docs/plans/completed/world-fusion-plan.md),
the learning plan for `info.md`'s capture/fusion/output topics, kept as
the build log): `piros2_world/se3.py` (shared SE(3) pure
functions — both quaternion conversions, `make_transform`/`invert`/
`transform_points`, `BASE_FROM_OPTICAL`); **`cloud_mapper`'s VoxelMap
fuses now** — array-backed running weighted average per voxel (weight =
clouds observed, capped at `max_weight` so evidence stays displaceable;
`min_weight: 2` holds one-look noise voxels out of the published map;
27–30 ms per 45k-point cloud); and a new offline pipeline in
`tools/recon/` under the perception venv (open3d 0.19, whose pip wheel
is CUDA-enabled out of the box): `just fetch-tum` / `fuse-tum` (TSDF a
TUM sequence — fr1/desk fuses 596 frames in 5.9 s at 10 ms/frame on the
GPU; marching cubes OOMs on the 6 GB card below ~8 mm voxels and falls
back to CPU), `just export-capture <bag> <name>` (bag → TUM-layout
keyframes: CRC dup-skip, ONNX depth as 16-bit mm PNGs, rotation-only
poses in a separate rewritable `groundtruth.txt`), `just fuse-capture`
(same TSDF over an export; `--trajectory` swaps in RTAB-Map's poses —
export them with `rtabmap-export --poses_camera --poses_format 1`), and
`just room-layer <mesh>` (RANSAC planes, floor via the
scene-above-fraction — largest-horizontal picks the *desk* in fr1/desk —
Manhattan snap, `room.json` + GLB). `datasets/`, `captures/`, `meshes/`
are git-ignored. The lit-sweep experiment fused the same 44 s capture
under both pose files and got two *different* failure signatures:
rotation-only poses smear radially (RTAB-Map measured 0.9 m of real
arm-arc translation in the "rotation-only" pan), while RTAB-Map's poses
reveal layered shingling instead — the neural depth's per-frame ±4%
scale wobble (measured on a static scene), the named next lever
(per-frame depth-to-TSDF alignment, out of scope). The tape-measure
check pinned **`depth_scale: 2.69`** (a 2.50 m wall read 9.30 m at
scale 10; re-export verification +0.1%) in `perception.yaml`. Also
hardened along the way: `mapping.launch.py` got 30-deep sync queues
(exact-sync pairing at the default 5 was a coin toss — 0–6 vs a
deterministic 24 odometry updates on the same replay), and
`rtabmap-export` does not work on these databases — `rtabmap-report
--poses_raw` (via `just map-headless`) is the pose export that works.

The live mesh build (started 2026-08-11; its plan file was retired
2026-08-15 when the world mesh fork absorbed the remaining work — this
paragraph is now the record):
the live pipeline grew a surface. `tsdf_mesher` (venv `ExecuteProcess`,
open3d lazy-imported) integrates synced `/depth` + compressed RGB into a
`VoxelBlockGrid` at 2 cm voxels (~52–78 ms/frame CUDA) and re-meshes on
a 10 s timer onto `/world/mesh_live` as a latched TRIANGLE_LIST Marker
(real geometry, dev-box-local only — RViz caches `mesh_resource` by URI
so files can't refresh; `integrate()` pairs only (float, float) or
(uint16, uint8) dtypes; since 2026-08-12 `fill_hole_radius: 0.06`
triangulates small interior gaps closed at extraction — the radius
bound keeps the scan's open outer boundary open, and the 60k triangle
cap is the other, louder source of visible holes — its warning says
what was dropped). `world.rviz` shows it as **LiveMesh**
(FusedMesh, the offline overlay via `mesh_marker`, defaulted off once
LiveMesh landed and was removed outright 2026-08-12). P2 landed the
depth-alignment todo the hard way: conform-to-map is *unstable* —
VoxelBlockGrid's ray-cast reads ~1.25 voxels far (voxel-proportional,
measured) and the feedback loop walks walls away — so `depth_align.py`'s
`ScaleAligner` is a high-pass (correct only the deviation from a
rolling ratio median; drift impossible by construction). Measured
benefit: per-frame placement spread 4.0% → 2.9%; the residual is
spatially structured model error a global scale can't touch (affine
alignment is the named next lever in todo.md). `fuse_capture --align`
uses the same code. P3's `odom:=rgbd` launch arg swaps the detector's
rotation-only TF (new `publish_tf` param — REP-105, one parent per
frame) for live `rgbd_odometry` + a raw republisher; bag-verified (101
odometry updates, rgbd owns `/tf`). P1's live glance closed 2026-08-12
— the live sessions watched the surface and their "triangles have
gaps" observation fed straight back into `fill_hole_radius` and the
cap finding. Open: P3's live hand-sweep gate (and one live click of
`~/reset`, bag-proven only) — both tracked in todo.md since the world
mesh plan closed.

The world mesh project (built 2026-08-12, plan closed by decision
2026-08-15 with the live gates unrun — measurement work, not build
work, remained; the gates moved to todo.md —
[docs/plans/completed/world-mesh-plan.md](docs/plans/completed/world-mesh-plan.md)):
**`src/piros2_world_mesh` — a full fork of `piros2_world`** (the four
nodes at fork time, `se3.py`/`depth_align.py`, the whole test suite,
imports renamed; perception stays shared), re-posed for the surface
and free to drift — same node names, topics and services, so it is an
*alternative* session, never run alongside `just world`. The first
divergence beyond defaults landed 2026-08-15: **`cloud_mapper` was
removed from the fork** (node, test, config, CloudMap display, trap
pattern) — the TSDF is this session's fusion accumulator and the
voxel panorama duplicated it; `piros2_world` keeps its mapper. First built
that day as a session wrapper inside `piros2_world` (include +
`extra_params` overlay); rebuilt as its own package by decision the
same day, the wrapper bits reverted. `just world_mesh` — aliased
**`just dev`** and, since 2026-08-15, **`just run`** too (the fork is
now the day-to-day target *and the sole development target — new work
lands in the fork, see Conventions*; `just world` stays as the frozen
known-good fallback) — runs `world_mesh.launch.py`: `odom:=rgbd`
by default (6-DoF, seven processes since the mapper removal; the
recipe trap carries the two
extra pkill patterns), the fork's own full `world_mesh.yaml` with
quality-biased `tsdf_mesher` values (1.5 cm / 120k triangles / 15 s —
provisional until the sweep measures them), and `world_mesh.rviz`
with no CloudMap display. `tsdf_mesher` (both forks' copies) grew
`~/save` (`just mesh-save`): the surface outlives the session as
`meshes/live_<stamp>.ply`, hand-written ASCII PLY so the path stays
open3d-free and unit-testable. P0–P3 built and tested same-day; the
live gates (hand sweep — shared with live-mesh P3 — and an in-session
save) are open. Found while building: `just world` passes args to the
camera launch only, so `just world odom:=rgbd` never actually reached
`world.launch.py`; `world_mesh` routes args to both launches.
The 2026-08-16 live-debug session (Depth3D flickering between a TF
error and rendering) rebuilt the fork's transport: rgbd's `rgb/image`
had been remapped to `/image_raw` — usb_cam's *raw* topic, so it was
silently pulling 2.7 MB frames over the Wi-Fi — and five dev-box
readers each pulled their own unicast copy of the compressed stream,
collapsing the link (~2 frames/s each at 14+ MiB/s of traffic). Now
the stream crosses the Wi-Fi once (`camera_relay` fans it out locally
on `/camera_relay/compressed`), the depth estimator republishes the
exact frame it inferred on as `/depth/rgb` (`publish_rgb`, stamps
identical to `/depth`, so exact sync pairs every depth frame — the
60 fps republisher node is gone from this session), and the estimator
paces the whole pipeline (`max_rate: 5` — what rgbd sustains, so the
odom TF stays current with the clouds instead of trailing a queue
backlog; the GPU also does half the work), and `cloud_projector`
publishes `/points` already posed in `odom` (`output_frame`, a
latest-TF lookup — the mapper/mesher rule) because even paced, RViz's
wait-for-TF-at-stamp raced the always-late odometry transform and
flapped the Depth3D status under RViz's own load; a cloud whose frame
*is* the fixed frame cannot lose that race. Same
session: `camera.launch.py` pre-flights a *held* device (names the
holder PID; a leaked usb_cam had fed a whole session unnoticed) —
docs/info/troubleshooting.md#a-live-session-crawls-at-2-fps-while-the-pis-wi-fi-is-saturated.
`just world`'s `odom:=rgbd` mode still carries the raw-topic collision
— live it will saturate the link; its default `kp` mode is unaffected.
The fork's mesher runs the mesh-completion pass since 2026-08-18
(`mesh_fill.py`: debris components pruned, interior holes filled from
their rings with each component's frontier left open, quadric
decimation instead of the pinhole-punching subsample cap, and
`save_watertight` writing a Poisson-closed `_closed.ply` beside the
honest save — measured live: 373 debris pruned, 142 holes filled,
zero interior loops ≤ 0.25 m surviving).

The Wi-Fi watchdog (done 2026-08-12, the whole
[watchdog plan](docs/plans/completed/wifi-watchdog-plan.md) planned,
built and drilled in one day, after the Pi's link died twice in two
days with the OS running on): `just wifi` (link health below
`just status`), the Ansible `wifi` role (power-save off,
boot-persistent; the escalation-ladder watchdog on a 60 s timer —
reassociate → `brcmfmac` reload → reboot guarded by a 10-min uptime
floor and 1-hour cooldown; sshd ClientAlive), and the reap contract on
the justfile's camera launchers (`ssh -tt` + ServerAlive +
`</dev/null`). Every rung and both guards carry live journal evidence;
the drill reproduced incident 1's `status_code=16` AP rejection and
recovered unaided at T+426 s via the driver-reload rung — the
escalation exists because reassociation alone measurably cannot clear
this failure. Wi-Fi outages now reap the camera session (device
released in ~60 s) instead of orphaning it.

Verification without a person (built and proven 2026-08-18, the day the
question "can Claude check the output like Playwright checks a web page?"
was asked — [docs/info/verification.md](docs/info/verification.md)):
the `verify` recipe group. `just snap` writes what a running session is
publishing to files (JPEG bytes off every image topic, counts, mesh
triangles, `odom → base_link`, and every X window titled rviz/rqt/`.ply`
dumped via `xwd` → ffmpeg — the Xwayland root can't be dumped, windows
can); `just run-bag [bag]` runs the whole `world_mesh` session from a
bag with no Pi; `just gate-bags` cuts a real sweep into two *gate bags*
(`tools/verify/make_gate_bag.py`: flick = A → noise → B → noise → A′,
occlude = A → noise → A′, A′ repeating part of A so the pipeline's own
poses during A are the reference); `just gate flick|occlude` runs one
headless and exits 0/1 (`gate_check.py`: A′-vs-A pose error over the
tail + the plan's promised log lines, `poses.png` as the picture); and
`just mesh-views` renders a saved PLY from fixed viewpoints offscreen.
Measured the same day: **flick PASS** (65.3° correction, tail 0.48°),
**occlude PASS** (Δ 18.4° snap, tail 0.95°/3 cm), and the black-fill
occlude **FAIL → fix → PASS**: a covered lens yields no descriptors,
`could_estimate` stayed False and the loss counter never moved — the
detector now counts a nothing-to-match frame as lost once tracking has
ever succeeded (`was_tracking`, two tests; toggling it off reproduces
the 19.7° failure). The relocalization plan's two "needs a human" gates
closed that way; the hand-sweep gate ran as `run-bag bags/sweep3` +
`snap` + `mesh-save` + `mesh-views` (723k-triangle PLY, one far sheet
top-down). Notes for Claude: rviz2 sometimes needs two SIGTERMs; never
put a node's source path on the same command line as a session recipe
(its EXIT trap's `pkill -f` matches your shell — exit 144).

The SLAM build (started and mostly built 2026-08-18, late — the plan is
still in-progress:
[docs/plans/in-progress/slam-plan.md](docs/plans/in-progress/slam-plan.md)):
the fork gained the backend it lacked. `keypoint_detector` (rgbd mode)
keeps a **keyframe pose graph** — every stored keyframe is a node with
an odometry edge, and every few depth-paired frames the live view is
matched against the *older* keyframes, verified by RANSAC PnP on the
keyframe's stored 3D landmarks (`pnp_pose`, 3D-3D cross-check,
implied-drift bound) and, if it survives, added as a **loop edge**;
`pose_graph.py` (pure numpy, `se3.py`'s new `hat/so3_exp/so3_log/
se3_exp/se3_log/adjoint`) then runs Gauss-Newton on SE(3) with Huber
on loop edges and the correction goes out as **`map → odom`**
(REP-105; `slam:=own`), the optimised keyframe poses on
`/world/trajectory` (+ `/world/trajectory_odom`, its odom twin) and
the edges on `/world/keyframe_graph` (RViz `Trajectory`,
`KeyframeGraph`). `tsdf_mesher` integrates in `map` when a backend
owns it and, under `slam:=own`, remembers its frames (aligned depth,
JPEG, odom pose, applied correction) and **rebuilds the volume from
memory** when the trajectory moves (~100 frames in 2–3.5 s); its refresh
now finishes off-thread (decimate → complete → publish) because at
1.5 cm voxels this scene's 0.7–1.6 M triangles cost 12–21 s inline and
starved integration to ~50 frames a run. `~/save_map` writes the graph
beside the keyframes; `map_path:=` restores and extends it. The
launch: `slam:=off|own|rtabmap` (rtabmap = RTAB-Map's SLAM node as the
yardstick, never alongside ours), `depth_source:=estimator|external`,
`mesh_watertight:=`, `mesh_save_frames:=`. **Gates, all headless
(docs/info/verification.md):** `just gate-loop [own|rtabmap|off]` on
`bags/gate_loop` (a *palindrome* of `sweep3` — out then back —
`make_gate_bag.py loop`; scored by `traj_check.py loop` on files
`traj_record.py` wrote: own 6.1 cm / 1.9° → 2.3 cm / 0.85°, RTAB-Map
0.7–1.4 cm / 0.6–1.6°, off FAIL), `just gate-tum [own|rtabmap|off]`
(`tum_player.py` plays fr1/desk as camera + real depth + static TF;
`traj_check.py ate` vs ground truth: own 0.163 → 0.089 m, RTAB-Map
0.212 → 0.096 m), `just gate-mesh` (`mesh_split.py`: OUT vs BACK halves
re-integrated at odom vs corrected poses — PASS directionally but not
yet credible, see the plan), `just gate-map` (persistence — written,
unrun). Three findings worth remembering: landmark geometry built from
the *latest* depth + *latest* TF was off by degrees at hand-pan speed
(depth lands ~80 ms, odom TF ~200 ms after the frame) — the rgbd
geometry now runs on exact (frame, own depth, TF-at-stamp) triples via
a short queue; RTAB-Map's DB odometry column is re-based on every
odometry auto-reset, so a correction is `optimised ∘ tf_odom(t_k)⁻¹`
from the recorded TF, never from that column; and `/mapPath`'s poses
carry no per-pose stamps. **The SLAM claim has not flipped**: scope
lines, the GitHub topic and `docs/to_learn/emescent.md` wait for
`gate-map` and a credible P3 metric.

Testing: `just test` (colcon test + result aggregation) or the VSCode Testing
sidebar — both report identically. All packages are style-clean and the suite
is green (199 tests; `piros2_vision`, `piros2_perception`, `piros2_camera`,
`piros2_world` and its fork `piros2_world_mesh` (which carries the
same suite minus the mapper tests, imports renamed, plus the
camera_relay byte-identity tests and, since 2026-08-18, the SLAM
suite — `test_pose_graph.py`: Lie round trips, a drifted circle one
closure shuts, Huber vs a planted wrong loop, the `g2o` oracle; PnP on
a synthetic scene; store exclude/force/translation-novelty/node_id;
graph save/load) hold real unit tests — none need hardware or model weights: a fake ONNX
session for the estimator (plus its `publish_rgb` stamp-twin and
`max_rate` pacing tests), synthetic depth planes for the projector
(plus its odom-frame output through a stubbed TF buffer),
a fake `/proc` tree for the camera launch's busy-device pre-flight,
synthetic chessboards for the keypoint detector, pure-function
`rates`/`stats_lines` tests for the dashboard, matching and
rotation-geometry tests on seeded noise and synthetic ray bundles — a
chessboard's lookalike corners defeat the matcher's cross-check by
design — SE(3) geometry tests driving all four quaternion branches
(`test_se3.py`), stubbed-TF weighted-fusion voxel-map tests for
`piros2_world`'s mapper: noise averages toward the plane, min_weight
holdback, capped
inertia — and pure-function marker/alignment/PLY-serialisation tests
for the live mesh and its `~/save`, and the mesh-completion
suite: punched-plane/pinched-hole/debris/tint/idempotence fixtures
for `mesh_fill.py`, plus a parameter-contract test pinning the
completion knobs, and the relocalization suite — keyframe store,
recovery geometry, rigid 3D fit, map persistence, keyframe marker, and
the two blackout-counts-as-lost tests the black-fill gate bag forced)
— 227 tests as of 2026-08-18 night (the fork alone 122). `tools/` scripts (recon, verify) sit outside colcon and
are exercised by running them, not by the suite.

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
- **The Pi's Wi-Fi link dies; the Pi does not.** Twice (2026-08-11/12) the link
  dropped and never recovered — once with the mesh AP rejecting re-association
  100 times (`status_code=16`), once silently for 15 h — while the OS ran on
  undisturbed (journal alive, load ~0, no undervoltage). So never diagnose an
  unreachable Pi as "crashed" without evidence: `ping` first, and after recovery
  read `journalctl -b -1` — the truth is in the previous boot. Since 2026-08-12
  the link self-heals: the Ansible `wifi` role runs an escalation-ladder
  watchdog (reassociate → `brcmfmac` reload → guarded reboot, 60 s timer;
  flight recorder `journalctl -t wifi-watchdog`, thresholds in
  `group_vars/robot.yml`) — drilled to unaided recovery in ~7 min, with the
  drill reproducing the AP's `status_code=16` rejection and proving only the
  driver-reload rung clears it. Dead sessions reap themselves: camera
  launchers use `ssh -tt` + keepalives and the Pi's sshd runs ClientAlive, so
  a link death releases `/dev/video0` in ~60 s (a lit LED on an unreachable
  Pi now means the reap failed — `just stragglers` after recovery; a held
  device is usb_cam's `char*` abort). Every scripted `ssh pi` must carry
  `-o BatchMode=yes -o ConnectTimeout=5` — a bare ssh hangs ~2 min against a
  dead link and wedges any trap it sits in; camera launchers additionally
  need the `-tt`/keepalive/`</dev/null` shape (copy an existing recipe).
  `just wifi` is the link-health view. Build log:
  [docs/plans/completed/wifi-watchdog-plan.md](docs/plans/completed/wifi-watchdog-plan.md);
  incident record in [docs/info/networking.md](docs/info/networking.md#wi-fi-link-reliability).
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
  GLFW apps (Open3D's viewer) need a *different* pin: unsetting
  `WAYLAND_DISPLAY` alone doesn't stop GLFW picking Wayland — add
  `XDG_SESSION_TYPE=x11` (`just view-mesh` carries both).
- **Justfile cleanup traps must `pkill -f` node patterns, not `kill %N`.**
  Background jobs in recipes are `bash -lc` wrappers; killing the wrapper
  orphans the actual ros2-run grandchildren, which sit silent until a live
  stream feeds their subscription again (bit twice on 2026-07-27). The full
  teardown contract is in Conventions; `just stragglers` sweeps both
  machines for survivors —
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
- **The active world stack is `piros2_world_mesh` — all new work lands
  there** (and in its shared dependency `piros2_perception`), never in
  `piros2_world`. The unforked `piros2_world` is the frozen known-good
  fallback: leave it alone unless it stops launching, and don't backport
  the fork's changes for parity. Two accepted consequences of the
  freeze: its `odom:=rgbd` mode still carries the raw-topic collision
  (it will saturate the Wi-Fi if run live — its default `kp` mode is
  unaffected), and it predates the 2026-08-16 transport rework
  entirely (no relay, no pacing, viewer-transformed clouds).
- Day-to-day commands are recipes in the `justfile` (`just` lists them by
  group: provision, sync, status, test, build, recon). Add a recipe rather
  than documenting a long one-off command; keep recipes and docs in
  agreement.
- **Sessions tear themselves down — no stragglers.** Closing the session's
  window and Ctrl-C must both end *everything* the recipe started, on both
  machines. The mechanism every session recipe uses: the viewer runs in
  the foreground (window close ends the recipe) and a `trap … EXIT`
  `pkill -f`s every node pattern the recipe started — bash fires the EXIT
  trap on Ctrl-C too, so one trap covers both exits. When a session gains
  a node (or a new session recipe is written), its pkill pattern goes into
  the trap in the same change, and `just stragglers` — a both-machine
  sweep that prints `clean` per host — is the check —
  [docs/info/troubleshooting.md](docs/info/troubleshooting.md#orphaned-nodes-keep-logging-into-a-terminal-after-a-recipe-ends).
- **Ad-hoc background runs get the same teardown — this means you,
  Claude.** Anything started by hand outside a recipe while debugging or
  verifying — a camera launch over SSH, a node, a republisher, a bag
  play — has no EXIT trap, so it leaks unless you own its shutdown: bound
  it up front (`timeout -s INT 30 …`, and on the Pi
  `ssh pi 'timeout -s INT 30 bash -lc "…"'`) or `pkill -f` its pattern
  when done, then run `just stragglers` and get `clean` on both hosts
  *before* reporting results. A leaked usb_cam is worse than noise — it
  holds the camera's exclusive capture, so every later session dies with
  `Device or resource busy`
  ([docs/info/camera.md#handling-rules](docs/info/camera.md#handling-rules),
  rule 10).
- **Gates are closed by scripts, not eyes, wherever a script can —
  [docs/info/verification.md](docs/info/verification.md).** When a plan
  phase "ends with" a check, write the check so it names its evidence: a
  number on a topic or a log line with a threshold, compared against the
  pipeline's own earlier output (`gate_check.py` is the model); the bag
  it replays (`just record` once by a person, `make_gate_bag.py` if the
  motion is a re-ordering of views we have); or the picture that
  answers it (`just snap` for a session, `just mesh-views` for a saved
  surface — say which view). Reserve "needs a human" for the physical
  world — a motion that was never recorded, the tape-measure scale
  check, exposure in a real room, taste — and say what one recording
  would turn it into. Before reporting a live-behaviour claim, run the
  gate or take the snap and cite the file; the RViz window is a viewer,
  not the evidence.
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
`git@github.com:bthek1/piros2.git`. The GitHub repo carries a one-line
description and the topics `ros2`, `robotics`, `computer-vision`,
`point-cloud`, `raspberry-pi`, `onnx` (set 2026-08-11) — "SLAM" is
deliberately excluded, matching the repo's honest rotation-only scope;
don't add it.

## Documentation map

`docs/info/` holds reference docs; `docs/plans/` holds plans, sorted by status
into `in-progress/` and `completed/` (see Conventions).

| File | Contents |
| --- | --- |
| [README.md](README.md) | Overview and entry point |
| [docs/info/project-overview.md](docs/info/project-overview.md) | Single-page account of the project and progress so far — packages, timeline, current state |
| [docs/info/just_world_mesh_diagrams.html](docs/info/just_world_mesh_diagrams.html) | The `just world_mesh` session drawn four ways — node/topic dataflow, two-machine deployment, TF ownership, recipe lifecycle — plus reference tables of every topic, service, and per-node measured cost (renamed from just-world-diagrams.html 2026-08-15 when the fork became the day-to-day session; the old name is gone — fix links, don't resurrect it; redrawn 2026-08-16 for the transport rework, topic list regenerated from the live graph). Mermaid rendered in-browser (loads mermaid.js from a CDN); open it, don't read it as source. Keep it in step with the session when nodes, topics, or measured figures change |
| [docs/info/hardware.md](docs/info/hardware.md) | Measured specs of both machines and the camera's capture modes |
| [docs/info/setup.md](docs/info/setup.md) | Reflashing the Pi, provisioning both machines, rejected alternatives |
| [docs/info/ansible.md](docs/info/ansible.md) | The playbook: inventory, roles, gotchas |
| [docs/info/networking.md](docs/info/networking.md) | DDS discovery, domain IDs, interface pinning |
| [docs/info/camera.md](docs/info/camera.md) | Driver choice, transport, V4L2 controls, calibration |
| [docs/info/troubleshooting.md](docs/info/troubleshooting.md) | Symptom → cause |
| [docs/info/verification.md](docs/info/verification.md) | Checking output without a person: `just snap` (topics + X windows to files), `just run-bag` (the session from a bag), gate bags + `just gate` (the relocalization gates as pass/fail runs, measured 2026-08-18), the SLAM gates (`gate-loop`, `gate-tum`, `gate-mesh`, `gate-map` — palindrome loop bag, TUM player, trajectory recorder/checker), `just mesh-views` (offscreen renders), `just mesh-planes`; how to write a gate a script can close; what still needs a human |
| [docs/info/roadmap.md](docs/info/roadmap.md) | Milestones and their status |
| [docs/info/perception.md](docs/info/perception.md) | Perception design: camera → depth → point-cloud room map, what mono can honestly do |
| [docs/plans/completed/perception-plan.md](docs/plans/completed/perception-plan.md) | Perception build order — phases P0–P4, the `src/piros2_perception` package, per-phase proofs |
| [docs/plans/completed/world-plan.md](docs/plans/completed/world-plan.md) | Build order for `src/piros2_world` — camera/depth/keypoint feeds + live stats in one dashboard window; done 2026-08-03, kept as the build log |
| [docs/plans/completed/world-3d-plan.md](docs/plans/completed/world-3d-plan.md) | Camera orientation from keypoint matches + accumulated cloud map, both in RViz — done 2026-08-05, kept as the build log |
| [docs/plans/completed/world-combined-plan.md](docs/plans/completed/world-combined-plan.md) | One command, three windows: dashboard mosaic + orientation RViz + map RViz — done 2026-08-05, kept as the build log |
| [docs/plans/completed/ansible-plan.md](docs/plans/completed/ansible-plan.md) | Build order for the `ansible/` tree — done 2026-07-24, kept as the build log |
| [docs/plans/completed/world-fusion-plan.md](docs/plans/completed/world-fusion-plan.md) | Learning plan for `info.md`'s topics — TSDF fusion, pose graphs, meshing, plane fitting — phases P0–P6: weighted cloud-map fusion, the `tools/recon/` offline pipeline, `depth_scale` pinned at 2.69; done 2026-08-10, kept as the build log |
| [docs/plans/completed/world-mesh-plan.md](docs/plans/completed/world-mesh-plan.md) | Fork `piros2_world` into a new package `piros2_world_mesh` (`just world_mesh`, aliased `just dev` and `just run`) and diverge it mesh-first — `odom:=rgbd` default, quality-biased TSDF params, mesh-centric RViz, a `~/save` PLY export; built 2026-08-12, closed by decision 2026-08-15 (live sweep gates unrun, moved to todo.md), kept as the build log |
| [docs/plans/completed/world-mesh-diagrams-plan.md](docs/plans/completed/world-mesh-diagrams-plan.md) | Redraw `just_world_mesh_diagrams.html` for the 2026-08-16 transport rework (relay, `/depth/rgb` twin, pacing, odom-frame clouds) — done 2026-08-16 same day, kept as the build log; its regenerate-from-the-live-graph rule caught cloud_projector still pulling a second Wi-Fi copy |
| [docs/plans/completed/mesh-completion-plan.md](docs/plans/completed/mesh-completion-plan.md) | Upgrade the fork's `tsdf_mesher` so interior holes get filled from surrounding detail — P0–P4 all built and live-verified 2026-08-18 (decimation replaces the sieve cap, per-component loop classification + fan fill, both P3 levers measured out, Poisson-closed export); the hand-sweep gate ran by replay the same evening (`just run-bag bags/sweep3` → `mesh-save` → `mesh-views`, docs/info/verification.md) |
| [docs/plans/completed/relocalization-plan.md](docs/plans/completed/relocalization-plan.md) | Remember the room's keypoints, recover the pose: a novelty-gated keyframe store in the fork's `keypoint_detector` (descriptors + bearing rays + 3D landmarks in odom), absolute Kabsch/Umeyama recovery when tracking breaks — kp mode snaps its own orientation, rgbd mode snaps via RTAB-Map's `/reset_odom_to_pose` (semantics verified live) — saved room maps (`~/save_map` / `just map-save` → `maps/room_<stamp>.npz`, loaded back with `just run map_path:=…`; the cold-start relocalize-before-any-odom gate passed live) and a latched `/world/keyframes` marker (`Keyframes` display, off by default). All five phases built, 37 tests, 2026-08-18; the two "hand-motion" gates closed the same evening as replayable gate bags (`just gate flick` / `just gate occlude`, both PASS — and the black-fill variant found and fixed the blackout-isn't-loss bug); store hygiene deferred by decision until live evidence asks for it |
| [docs/plans/in-progress/slam-plan.md](docs/plans/in-progress/slam-plan.md) | Make `world_mesh` SLAM: the gap table (front-end + TSDF map + loss-only relocalization, but no loop detection while healthy, no backend, no map correction, no `map → odom`) and phases P0–P4. P0 done (palindrome loop bag, `slam:=rtabmap` yardstick, `traj_record`/`traj_check`/`tum_player`, `gate-loop`/`gate-tum`), P1 done (keyframe graph, always-on PnP-verified loop detection on exact-sync geometry), P2 done (hand-written SE(3) pose-graph optimiser checked against `g2o`, `map → odom`, `/world/trajectory`; own 2.3 cm / 0.85° vs RTAB-Map 0.7–1.4 cm / 0.6–1.6° on the loop bag, fr1/desk ATE 0.163 → 0.089 m), P3 built (TSDF rebuild from frame memory, threaded refresh; `gate-mesh` metric provisional), P4 built (graph persistence; `gate-map` unrun). Claims not flipped yet — 2026-08-18 |
| [docs/plans/completed/wifi-watchdog-plan.md](docs/plans/completed/wifi-watchdog-plan.md) | The Pi heals its own Wi-Fi link — `just wifi` visibility, power-save off, an escalation-ladder watchdog (reassociate → driver reload → guarded reboot) via the Ansible `wifi` role, and outages reap camera sessions instead of orphaning them; planned, built and drilled 2026-08-12 after two link-death incidents (kept as the build log) |

When hardware facts change (camera replugged, Pi reflashed, IP moved), update
[docs/info/hardware.md](docs/info/hardware.md) from real command output and note the date.
