# Project overview and progress

*Written 2026-08-11, updated 2026-08-15. A single-page account of what piros2 is, what has been
built, and where it stands. The detailed logs live in
[roadmap.md](roadmap.md) and the plans under
[docs/plans/completed/](../plans/completed/); this page is the map.*

## What this project is

A **learning project for ROS 2 Jazzy on real hardware**: a Raspberry Pi 5
with a Logitech C922 webcam publishing over Wi-Fi to an Ubuntu dev box that
runs the heavy compute and visualisation. The value is understanding ROS 2 —
QoS, TF, launch composition, transports — not shipping a product. Every
stage ended with something runnable and verified before moving on.

| Role | Host | Runs |
| --- | --- | --- |
| Sensor node | Pi 5 (`192.168.2.17`, Wi-Fi, headless) | `usb_cam`, anything touching hardware |
| Dev / viz | dev box (`192.168.2.109`, GNOME, GTX 1660 SUPER) | depth inference, RViz, rqt, builds |

Both machines run Jazzy natively from apt on Ubuntu 24.04, provisioned by
the Ansible tree in `ansible/` (six roles, idempotent on both hosts),
talking over CycloneDDS on domain 42 with interfaces pinned per host.
Day-to-day commands are `just` recipes; `just world_mesh` is the current
`just run` target (retargeted 2026-08-15) and `piros2_world_mesh` is
where all new work lands — `just world` remains the frozen known-good
fallback, deliberately untouched since the fork took over.

## The packages

All in `src/`, built with colcon (`--symlink-install`); the repo doubles as
the workspace.

| Package | What it is |
| --- | --- |
| `piros2_hello` | Hand-written talker/listener pair — the first node, verified across the LAN |
| `piros2_camera` | The camera launch + YAML: usb_cam on the Pi, static TF chain `base_link → camera_link → camera_optical_frame`, approximate intrinsics on `/camera_info` |
| `piros2_vision` | Canny edge detector (`cv_bridge`), the first processing node; its QoS and timestamp findings shaped everything after it |
| `piros2_perception` | `depth_estimator` (Depth Anything V2 Small, ONNX on the dev box GPU, 72–79 ms/frame) and `cloud_projector` (`/depth` + image → `PointCloud2` through K); plus `mapping.launch.py` for the RTAB-Map route |
| `piros2_world` | `keypoint_detector` (ORB + descriptor matching → rotation-only camera orientation via Kabsch on bearing rays, published as TF), `cloud_mapper` (voxel map with weighted-average fusion → `/world/map_points`), `dashboard` (live stats panel on `/world/stats/compressed`; its 2×2 mosaic retired 2026-08-12), and `se3.py` (shared SE(3) pure functions) |
| `piros2_world_mesh` | **The active world stack** (mesh-first fork of `piros2_world`; plan closed 2026-08-15): `odom:=rgbd` and quality-biased TSDF values by default, `~/save` writing the live surface to `meshes/*.ply`, and since the 2026-08-16 transport rework `camera_relay` (the session's single Wi-Fi reader, fanned out locally) with a paced depth pipeline and odom-frame clouds — run as `just world_mesh` / `just dev` / `just run` |

Outside `src/`: `tools/recon/` is an offline reconstruction pipeline under
the perception venv (open3d) — bag → TUM-layout capture export → TSDF
fusion → mesh → RANSAC room layer (`room.json` + GLB).

## How it got here

### The roadmap — milestones 0–6, 2026-07-23 → 2026-07-27

Concluded in five days ([roadmap.md](roadmap.md)):

- **M0 Environment** — Pi reflashed to Ubuntu Server 24.04 arm64 (the only
  platform with Jazzy arm64 binaries), Ansible roles written, `/chatter`
  crossed the LAN.
- **M1 First node** — `piros2_hello`, built on both machines, `just hello`.
- **M2 Camera** — usb_cam at a measured ~30 fps over compressed transport,
  after fixing the C922's `exposure_dynamic_framerate` frame-rate thief.
- **M3 Launch files** — camera config moved to launch + YAML, arguments
  passing end to end; caught the dead `camera_frame_id` parameter.
- **M4 Image processing** — the edge detector. Two load-bearing findings:
  the camera's **~0.73 s header-stamp fault** (never gate freshness on
  `header.stamp`), and **BEST_EFFORT delivers zero large frames** (a 2.7 MB
  frame fragments into ~1800 datagrams; only RELIABLE reassembles).
- **M5 TF** — the static frame chain, verified with `tf2_echo`; RViz check
  closed 2026-07-28. Checkerboard calibration remains the one open box —
  spec-derived approximate intrinsics (fx ≈ 907 px) released its gate, so
  it is an accuracy upgrade, not a blocker.
- **M6 Record/replay** — MCAP bags recorded on the Pi, fetched, and replayed
  through the pipeline entirely on the dev box; hardware no longer needed to
  iterate.

### Perception — 2026-07-27 → 2026-07-29

[perception-plan.md](../plans/completed/perception-plan.md) built
camera → neural monocular depth → point clouds. P1's depth node runs the
ONNX model (GPU since 2026-07-30, ~13 fps; the venv escape hatch for
PyPI-only onnxruntime is documented in the package README). P2's projector
hand-builds `PointCloud2` and was verified live in RViz. P3 (RTAB-Map
mapping) reached plumbing-verified — the full chain ran against a static
bag — and the plan was then **closed by decision before the room map was
built**; `just map` remains runnable if mapping resumes.

### The world stack — 2026-08-03 → 2026-08-05

Three plans in three days built and then unified `piros2_world`:

- **[world-plan.md](../plans/completed/world-plan.md)** (done 2026-08-03) —
  the dashboard: ORB keypoints plus every feed and its live stats in one
  mosaic, deliberately using latest-wins subscriptions as the contrast to
  `cloud_projector`'s exact sync (the mosaic itself was removed
  2026-08-12; the latest-wins stats panel is the surviving output). The live run re-measured the camera at
  **42–60 distinct frames/s** — the old "30 fps ceiling" is gone.
- **[world-3d-plan.md](../plans/completed/world-3d-plan.md)** (done
  2026-08-05) — rotation-only camera orientation from keypoint matches
  (the essential matrix is degenerate under pure rotation, hence Kabsch on
  bearing rays) publishing `odom → base_link` TF, and the depth clouds
  accumulated into a voxel panorama. Honest scope: orientation without
  position — a panorama, not a walkable map.
- **[world-combined-plan.md](../plans/completed/world-combined-plan.md)**
  (done 2026-08-05) — everything merged into `world.launch.py`: `just world`
  opens one RViz window with the live cloud, the map panorama and TF axes in
  one 3D scene, image panels docked alongside. `just orient` stays as the
  lightweight no-GPU session.

### Fusion — 2026-08-10

[world-fusion-plan.md](../plans/completed/world-fusion-plan.md), all in one
day — the learning plan for `info.md`'s capture/fusion/output topics:

- `cloud_mapper` now **fuses**: a running weighted average per voxel with
  capped weight and a `min_weight` holdback, replacing latest-wins.
- `tools/recon/` — TSDF fusion via Open3D's `VoxelBlockGrid` (fr1/desk:
  596 frames in 5.9 s on the GPU), bag → TUM-layout capture export,
  fuse-capture with swappable pose files, and a RANSAC room layer with
  Manhattan snapping.
- The tape-measure scale check pinned **`depth_scale: 2.69`** (a 2.50 m
  wall had read 9.30 m).
- The lit-sweep experiment fused one capture under two pose files and got
  two different failure signatures: rotation-only poses smear radially
  (RTAB-Map measured 0.9 m of real arm-arc translation in the "pure"
  pan), while RTAB-Map's poses reveal shingling from the neural depth's
  per-frame ±4% scale wobble — which names the next lever.

### The live mesh — 2026-08-11 → absorbed 2026-08-15

The live pipeline grows a surface (its plan file was retired
2026-08-15, the remaining work absorbed by the
[world mesh plan](../plans/completed/world-mesh-plan.md)):

- `tsdf_mesher` (the sixth `just world` node, venv-run like the depth
  estimator) integrates synced depth + RGB into a `VoxelBlockGrid`
  in-session and re-meshes every ~10 s onto `/world/mesh_live` — the
  **LiveMesh** display in the same RViz window, shipped as real triangle
  geometry because RViz caches `mesh_resource` files by URI.
- A high-pass `ScaleAligner` lands the depth-alignment todo: per-frame
  placement spread 4.0% → 2.9% (conform-to-map feedback proved
  *unstable* — the ray-cast reads ~1.25 voxels far and walks walls away).
- `odom:=rgbd` optionally swaps the rotation-only compass for RTAB-Map's
  live 6-DoF odometry.
- A `mesh_marker` node (RViz overlay of the newest offline mesh) lived
  one day — added 2026-08-11, removed 2026-08-12 once LiveMesh made it
  redundant.

### The world mesh fork — 2026-08-12 → closed 2026-08-15

[world-mesh-plan.md](../plans/completed/world-mesh-plan.md) — the
world stack forked for the surface; the plan closed by decision with
the live gates unrun (measurement, not build work — they moved to the
todo list):

- `src/piros2_world_mesh`: a full copy of the four world nodes plus
  their test suite (imports renamed; perception stays shared), free to
  diverge mesh-first without ever touching `piros2_world`. Since
  2026-08-15 the fork drops `cloud_mapper` — the TSDF is its fusion
  accumulator, and the voxel panorama duplicated it.
- `just world_mesh` — aliased `just dev`, and since 2026-08-15
  `just run` too — defaults `odom:=rgbd` (6-DoF poses, seven
  processes), quality-biased TSDF values (1.5 cm voxels / 120k
  triangles, provisional until measured), no CloudMap in its RViz
  layout. `just world` remains the classic session.
- `tsdf_mesher` grew `~/save` (`just mesh-save`): the live surface
  outlives the session as `meshes/live_<stamp>.ply` — hand-written
  ASCII PLY, unit-tested without open3d.
- Still open after the close, tracked in the repo's todo list: the
  live hand-sweep gate (inherited from the retired live-mesh plan)
  and an in-session save check.
- The 2026-08-16 transport rework (a live-debug day chasing a
  flapping RViz display down to its roots, recorded in
  [troubleshooting.md](troubleshooting.md#a-live-session-crawls-at-2-fps-while-the-pis-wi-fi-is-saturated))
  rebuilt the fork's data path: `camera_relay` makes the camera
  stream cross the Wi-Fi exactly once (five direct readers had
  collapsed the link into a retransmit storm), the depth estimator
  paces the pipeline at 5 Hz and publishes a stamp-identical raw twin
  (`/depth/rgb`) so the odometry pairs every frame — the old
  republisher's remap had been silently pulling *raw* frames over the
  Wi-Fi — and `cloud_projector` poses `/points` in `odom` itself via
  latest TF, so RViz never waits on the always-late odometry
  transform. The camera launch also pre-flights a *held* device now,
  naming the leaked holder. All of it landed in the fork and the
  shared `piros2_perception`; `piros2_world` was left frozen.

### The Wi-Fi watchdog — 2026-08-12

[wifi-watchdog-plan.md](../plans/completed/wifi-watchdog-plan.md), planned,
built and drilled in one day after the Pi's link died twice while the OS
ran on (the AP rejecting re-association with `status_code=16`; incident
record in [networking.md](networking.md#wi-fi-link-reliability)):

- The Ansible `wifi` role (the sixth role): radio power-save off, an
  escalation-ladder watchdog on a 60 s timer — reassociate → `brcmfmac`
  reload → guarded reboot — and sshd ClientAlive.
- The drill reproduced the `status_code=16` rejection under control and
  recovered unaided at T+426 s via the driver-reload rung; every rung and
  both reboot guards carry live journal evidence
  (`journalctl -t wifi-watchdog`).
- Outages now reap camera sessions (`ssh -tt` + keepalives on every
  launcher) instead of orphaning them against `/dev/video0`; `just wifi`
  is the link-health view.

## Testing

`just test` (or the VSCode Testing sidebar — identical results): **162
tests green** as of 2026-08-16, all style-clean, none needing hardware or
model weights — fake ONNX sessions, synthetic depth planes and
chessboards, seeded-noise rotation geometry, SE(3) quaternion-branch
coverage, stubbed-TF weighted-fusion tests for the mapper,
pure-function marker/alignment/PLY-serialisation tests for the live mesh
and its `~/save`, and the transport rework's additions: relay
byte-identity, the estimator's stamp-twin and pacing, the projector's
odom-frame output through a stubbed TF buffer, and a fake-`/proc`
busy-device pre-flight; `piros2_world_mesh` carries the same suite as
`piros2_world`, imports renamed, minus the mapper tests.

## Where it stands now

- **Working end to end:** `just dev` (= `just run` since 2026-08-15 —
  the mesh-first `piros2_world_mesh` session, 6-DoF odometry, one
  Wi-Fi copy of the camera stream since 2026-08-16) runs the
  dev-box stack against the live camera, with `just world` as the
  frozen classic baseline; the offline pipeline goes bag → mesh →
  room layer without hardware; the Pi repairs its own Wi-Fi link.
- **Committed:** the entire fusion-plan day — `se3.py`, the fusing
  `cloud_mapper`, `tools/recon/`, the pinned `depth_scale`, doc updates —
  landed as `bbe8c73` on 2026-08-11. The GitHub repo carries a one-line
  description and topics (`ros2`, `robotics`, `computer-vision`,
  `point-cloud`, `raspberry-pi`, `onnx`) as of the same day; "SLAM" is
  deliberately not among them — the honest scope is rotation-only
  orientation.
- **Open items** (plus [todo.md](../../todo.md)'s standing ambition, a
  C/C++ rewrite):
  - The world-mesh live gates: the lit-room hand sweep (settles the
    provisional TSDF values) and an in-session `just mesh-save` check.
  - Affine depth-to-TSDF alignment — the next lever now that the live
    high-pass scale aligner landed (placement spread 4.0% → 2.9%; the
    residual is spatially structured model error a global scale cannot
    touch).
  - Checkerboard calibration — optional accuracy upgrade over the
    approximate intrinsics.
  - The "reduce compute" throttle for consumers now that the camera
    delivers up to 60 fps.

## Recurring lessons worth knowing before touching anything

The traps that shaped the code (full list in
[troubleshooting.md](troubleshooting.md)): the ~0.73 s stamp fault, the
BEST_EFFORT/large-frame inversion, V4L2 controls persisting inside the
camera (`just camera-reset`), the PlatformIO venv shadowing `python3`,
Wayland vs Qt5/rviz2 pins, `pkill -f` not `kill %N` in justfile traps, and
non-interactive SSH not reading the ROS environment.
