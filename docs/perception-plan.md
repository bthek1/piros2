# Perception build plan — camera to 3D room map

> **Working document.** [perception.md](perception.md) is the design — what is
> being built and what a single webcam honestly can do. This is the build
> order: stable phases, each ending with something you can run and check
> before the next starts. The [roadmap](roadmap.md) concluded 2026-07-27; its
> two open human checkboxes (the RViz eyeball check and calibration) carry
> over here as P0.

## Preconditions, verified

What the perception stack inherits from milestones 0–6:

| Have | Where |
| --- | --- |
| Camera at ~30 fps, compressed stream over Wi-Fi | `just pipeline`, `src/piros2_camera` |
| TF chain `base_link → camera_link → camera_optical_frame`, headers stamped with the optical frame | `camera.launch.py` |
| Record/replay loop so hardware can stay off | `just record` / `just replay`, `bags/` |
| Calibration made turnkey | `just calibrate`, `docs/checkerboard-8x6-25mm.svg` |
| Working QoS knowledge for megabyte messages | RELIABLE for large frames — [troubleshooting.md](troubleshooting.md) |
| Test conventions | anchored linter tests, `pytest.ini`, `.vscode/` picker |

Known constraints that shape the phases: the camera's `header.stamp` lags
~0.73 s (never gate on it); BEST_EFFORT drops all large messages; the neural
model and cloud assembly run on the dev box; the Pi stays a sensor head.

## The new package

```
src/piros2_perception/            # ament_python, created in P1
├── package.xml                   # rclpy, sensor_msgs, cv_bridge, tf2_ros
├── setup.py                      # nodes below + launch/ + config/
├── piros2_perception/
│   ├── depth_estimator.py        # P1: compressed frames → /depth
│   └── cloud_projector.py        # P2: /depth + /camera_info → /points
├── launch/perception.launch.py   # P2: composes vision pipeline + both nodes
├── config/perception.yaml        # model path, subsample, depth scale
├── models/                       # ONNX weights, git-ignored, fetched by recipe
└── test/                         # anchored linters + real unit tests per node
```

Conventions carried over: `piros2_<thing>` naming, anchored tests copied from
an existing package (never the generated CWD-dependent form), package added to
`.vscode/tasks.json`'s picker, recipes added to the justfile as they appear.

## P0 — Calibration (human, gates everything)

> **2026-07-27 — attempted; tooling fixed, calibration still open.** A first
> session live-debugged `just calibrate` into genuinely turnkey shape: the
> calibrator's service hookup was a dead parameter (now a proper remap onto
> `/usb_cam/set_camera_info`), closing its window left the node spinning
> forever (the recipe now watches the window's X id, with `QT_QPA_PLATFORM=xcb`
> because the Qt window is otherwise a Wayland surface no X tool can see), and
> the `calibrate`/`replay` cleanup traps orphaned their nodes (`kill %N` →
> `pkill` patterns). All verified — window close now stops the whole stack.
> The session ended without a save, so `camera_info` is still empty and P0
> keeps gating. Details in
> [troubleshooting.md](troubleshooting.md#closing-the-calibrator-window-doesnt-stop-it).

Print `docs/checkerboard-8x6-25mm.svg` at **100 % scale** (ruler-check one
square = 25 mm), then:

```bash
just pipeline        # terminal 1
just calibrate       # terminal 2 — wave the board until all bars green
```

**Calibrate → Save**, extract `ost.yaml` from `/tmp/calibrationdata.tar.gz`
as `src/piros2_camera/config/c922_720p.yaml`, set in `camera.yaml`:

```yaml
camera_info_url: package://piros2_camera/config/c922_720p.yaml
```

**Proves:** `ros2 topic echo /camera_info --once` shows a non-zero K matrix.
While at the screen, do the deferred RViz check: `rviz2`, Fixed Frame
`base_link`, TF + Image displays — three frames and the live picture.

**Concept:** intrinsics — fx/fy/cx/cy are what turn a pixel ray into a
direction in space; without them a depth image cannot become geometry.

## P1 — Depth estimator node

> **2026-07-28 — done, verified on the milestone-6 bag.**
> `src/piros2_perception` exists: `depth_estimator` subscribes
> `/image_raw/compressed`, decodes with `cv2.imdecode`, runs DA-V2 Small
> (fp32 ONNX, ~99 MB, `just fetch-model`, checksum-pinned) and publishes
> `/depth` (32FC1, input frame's header) + `/depth/preview/compressed`.
> The venv landed as designed (`~/.venvs/piros2-perception`, onnxruntime
> 1.28.0); the node runs as `python -m` under the venv interpreter because
> colcon's hardcoded shebang would miss it — `just depth` owns the exact
> invocation, the package README documents why. Measured on the dev-box CPU:
> **280–305 ms/frame steady (~3 fps), first inference ~1.3 s warm-up**.
> Proof: the milestone-6 bag replayed through the node produced a preview
> that is unmistakably the recorded desk — near desk bright, monitors mid,
> window wall dark. Five unit tests (fake-session injection, no weights)
> green in `just test`. `just depth` against the live camera has not been
> run yet — replay was the verification, per the milestone-6 loop.

Scaffold `piros2_perception` (`ros2 pkg create`), then `depth_estimator.py`:

- Subscribes `/image_raw/compressed` directly and decodes with
  `cv2.imdecode` — no republisher needed inside a node; JPEG bytes → numpy is
  two lines, and it keeps the Wi-Fi budget at ~2 MB/s.
- Runs **Depth Anything V2 Small** via ONNX Runtime on the dev-box CPU.
  Model fetched once by a `just fetch-model` recipe into `models/`
  (git-ignored, checksum pinned in the recipe).
- Publishes `/depth` (32FC1, same header and `camera_optical_frame` as the
  input frame) and `/depth/preview/compressed` (colourised JPEG).
- Logs measured inference ms/frame against its own clock — stamps stay
  untrusted.

**Python-dependency reality:** ONNX Runtime is PyPI-only. This is the
documented escape hatch from [setup.md](setup.md#on-sourcing-ros):
`python3 -m venv --system-site-packages ~/.venvs/piros2-perception`, so
`rclpy` still resolves from the system while `onnxruntime` comes from pip.
The node runs under this venv's interpreter; the exact invocation lives in
the justfile recipe, not in anyone's memory. Recorded in the package
README the moment it exists.

**Proves:** `just depth` (pipeline + estimator + preview viewer) shows a
depth-shaded image that plausibly is the room — near things bright, far
things dark. Measured fps recorded in perception.md (expect low single
digits; mapping does not need more).

**Concepts:** the ROS-to-ML boundary, venv coexistence with rclpy, publishing
derived sensor data with honest headers.

## P2 — Point cloud projector

`cloud_projector.py`:

- Subscribes `/depth` + `/camera_info` (the P0 payoff) and the RGB frame for
  colour; message_filters approximate-time sync on the two image topics.
- Hand-rolled projection — the point is to meet the K matrix once:
  `x = (u - cx) * z / fx`, vectorised in numpy, subsampled (`config`
  parameter, default every 4th pixel ≈ 57k points) before building a
  `PointCloud2` in `camera_optical_frame`.
- Unit test in the P1/P4 style: synthetic depth plane + known K in, assert
  the cloud's geometry (a flat wall at 2 m comes back flat and at 2 m).

`launch/perception.launch.py` composes the Pi's vision launch (include, as
`vision.launch.py` did) with both perception nodes on the dev box — note this
launch runs *here*, not on the Pi.

**Proves:** RViz (Fixed Frame `base_link`) shows a coloured 3D frustum of the
live view, correctly posed relative to `base_link`; `just cloud` runs the
whole chain. A wall known to be ~3 m away lands near 3 m — rough scale check
against a tape measure, result recorded.

**Concepts:** `PointCloud2` memory layout, time synchronisation of paired
topics, why everything is published in the optical frame and RViz transforms
it.

## P3 — From clouds to a map

The hard phase; iterate on bags, not live hardware.

1. Fix exposure and gain first (`camera.md#v4l2-controls`) — auto-exposure
   fights feature tracking, and this is where that bites.
2. `just record` a slow hand-held sweep of the room (30–60 s, deliberate
   motion, revisit the start for loop closure).
3. RTAB-Map (`ros-jazzy-rtabmap-ros`, dev box, via `extra_ros_packages`) in
   RGB-D mode fed the replayed RGB + P1 depth. Known-community pattern, not a
   supported configuration — expect parameter tuning, and expect scale to be
   approximate.
4. Iterate: replay bag → tune → replay, using the milestone 6 loop.

**Proves:** a recognisable room-scale coloured cloud/mesh in RViz built from
one sweep, exported to a file (`rtabmap` database or PCD). Honest failure
criterion: if mono-depth + RTAB-Map cannot converge on this room, fall back
to accumulating P2 clouds from a handful of *held* poses entered as static
transforms — smaller result, same concepts, and the plan says so rather than
pretending.

**Concepts:** visual odometry, loop closure, why mapping needs poses, scale
ambiguity in monocular pipelines.

## P4 — Repeatable and honest

- Recipes: `just depth`, `just cloud`, `just map` (record-or-replay driven),
  each with cleanup traps like the existing ones.
- Unit tests green in `just test` and the VSCode sidebar on both machines'
  conventions.
- perception.md updated with **measured** fps, latency and scale error at
  each stage — numbers with conditions, per house rules.
- This file gets per-phase outcome annotations as phases land, ansible-plan
  style, and is archived when P3's map exists.

## Open decisions

| Question | Options | Lean |
| --- | --- | --- |
| Depth model | Depth Anything V2 Small ONNX vs MiDaS small | DA-V2 Small — better quality at similar cost |
| Where inference runs | dev-box CPU vs its NVIDIA GPU (driver present) | CPU first; GPU only if fps actually hurts |
| Cloud transport | publish `/points` over LAN vs view-only local | local only — a 57k-point cloud at even 5 fps is Wi-Fi abuse |
| P3 engine | RTAB-Map vs hand-rolled pose accumulation | RTAB-Map, with the fallback written into P3 |
| Venv home | `~/.venvs/piros2-perception` vs in-repo | home dir — the repo stays apt-only |
