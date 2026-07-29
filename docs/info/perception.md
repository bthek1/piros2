# Perception: from webcam to a 3D map of the room

> **Status: in build — P3 next.** P1 (depth node) and P2 (cloud projector)
> done 2026-07-28, both verified with measured numbers. P0's gate was
> released the same day with spec-derived approximate intrinsics
> (`/camera_info` now carries a real K); the checkerboard run is an
> accuracy upgrade whenever it happens. This document is the
> design; the phased build order — which phase creates what, and what each
> must prove, with per-phase outcome annotations — is
> [perception-plan.md](../plans/in-progress/perception-plan.md). Milestone 7 was an open choice —
> this is the chosen direction, and the roadmap concluded here.

The goal: point the C922 around the room and end up with a coloured 3D point
cloud of it, visible in RViz, anchored to the TF tree the earlier milestones
built.

## What a single webcam can and cannot do

Be clear-eyed about this before writing code:

- **The C922 measures no depth.** Every 3D point will be *inferred* by a
  neural monocular-depth model. Modern ones (Depth Anything V2) are good, but
  their output is relative depth with approximate scale — the map will be
  believable, not survey-grade.
- **Metres come from calibration.** Projecting a depth image into 3D uses the
  camera intrinsics (fx, fy, cx, cy). Until milestone 5's checkerboard run is
  done, `camera_info` is empty and no honest point cloud is possible. **P0 is
  the gate.**
- **Compute lives on the dev box.** The Pi publishes compressed frames
  (~1–3 MB/s); the neural model and cloud assembly run here. This is the same
  split every milestone has used: Pi as sensor head, dev box as brain.
- **A "map" needs poses.** One frame gives one cloud from one viewpoint. A
  room map means fusing clouds from many viewpoints, which requires knowing
  where the camera was for each — the hard part of the whole project, and
  where visual SLAM (RTAB-Map) enters at P3.

## Stages

Each stage runs and is verifiable before the next starts, per house rules.

### P0 — Calibration (the gate)

Already turnkey: print `docs/info/checkerboard-8x6-25mm.svg` at 100 %, run
`just pipeline` + `just calibrate`, commit the yaml as
`src/piros2_camera/config/c922_720p.yaml` and point `camera_info_url` at it.
Done when `/camera_info` carries a real K matrix.

**Gate released 2026-07-28** without the board: `c922_720p_approx.yaml`
derives K from the C922's spec (78° diagonal FOV → fx = fy ≈ 907 px, centred
principal point, zero distortion) and `/camera_info` now publishes it. A few
percent of geometric error, no distortion correction — acceptable against
monocular depth's already-approximate scale. The checkerboard run upgrades
accuracy whenever it happens.

### P1 — Monocular depth node

`piros2_perception/depth_estimator`: subscribes `/image_raw/compressed`
(decompressing locally, as `just replay` does), runs Depth Anything V2 Small
via ONNX Runtime on the dev box CPU, publishes:

- `/depth` (`sensor_msgs/Image`, 32FC1, metres-ish) — aligned 1:1 with the
  input frame, same header/frame_id
- `/depth/preview/compressed` — colourised JPEG for eyeballing in
  `rqt_image_view`

Python-package reality: ONNX Runtime is PyPI-only, so this is the documented
venv escape hatch (`python3 -m venv --system-site-packages`) from setup.md —
recorded in the package README when it happens. Expect a few fps; that is
fine, mapping does not need 30.

Done when: a depth preview of the room looks plausibly like the room.

**Done 2026-07-28.** Measured (dev-box CPU, fp32 DA-V2 Small at 518×518):
280–305 ms per frame steady, ~1.3 s first-inference warm-up — call it 3 fps.
Verified against the milestone-6 bag: the preview reproduced the recorded
desk scene with correct near/far ordering. Scale remains honest-relative;
`depth_scale` is a tuning knob until P2's tape-measure check.

### P2 — Point cloud

`piros2_perception/cloud_projector`: `/depth` + `/camera_info` →
`sensor_msgs/PointCloud2` in `camera_optical_frame`, hand-rolled projection
(the point *is* to meet fx/fy/cx/cy once) with colour from the RGB frame.

Done when: RViz shows a coloured 3D frustum of the current view, correctly
oriented relative to `base_link` — the TF payoff made visible.

**Done 2026-07-28.** Measured: 33k–57k points built in ~12 ms per cloud
(numpy-vectorised projection + structured-array serialisation), cloud rate
bounded by the depth node's ~3 fps. Verified live end to end; unit tests pin
the projection against a synthetic wall (flat, 2 m, pinhole-exact). The RViz
eyeball passed the same day — live cloud, correctly posed in `base_link`,
human-confirmed (after the display-stack fixes in
[troubleshooting.md](troubleshooting.md#rviz2-crashes-unable-to-create-the-rendering-window-glxcontext-100-tries)).
Only the tape-measure scale check remains — a lit room, a wall at a known
distance, and `depth_scale` tuned to match.

### P3 — From clouds to a map

The hard stage. Plan A: **RTAB-Map in RGB-D mode**, fed the camera stream
plus P1's synthetic depth — it estimates camera poses visually and fuses
clouds into a room-scale map. Plan B (simpler, more instructive, likely
first): sweep the camera between a handful of *held* positions, let
RTAB-Map's odometry do continuous tracking, and iterate on recorded bags
(milestone 6 tooling) rather than live hardware.

Done when: a recognisable, roughly-scaled point-cloud room in RViz, built
from a slow hand-held sweep.

**In progress 2026-07-29; plumbing verified.** `mapping.launch.py`
composes the depth estimator with RTAB-Map's `rgbd_odometry` and
`rtabmap` nodes (exact RGB/depth sync — the depth node's honest headers
pay off here — poses in `base_link`), and `just map [bag]` drives it from
a bag played once, watched in `rtabmap_viz`. The milestone-6 bag carries
an all-zero K (recorded before P0's intrinsics), so a fresh
valid-intrinsics bag (`bags/static1`) became the plumbing-check input —
and the check passed: on the static desk bag, odometry tracked with
quality 447–563 features (std dev 3–8 cm) at ~1–2 Hz synced pairs, and
RTAB-Map built a 1-node map, which is correct for a motionless scene.
First evidence Plan A's neural-depth pattern holds. Sweep numbers — map
quality, loop closure, scale error — follow the first real sweep.

### P4 — Make it repeatable

Bag-driven: `just record` a sweep once, iterate on the map pipeline offline.
Recipes for each stage; docs updated with measured fps/latency at every step,
per house rules.

## Honest risks

- Mono SLAM scale drift: the map may be internally consistent but 10 % off in
  size. Acceptable here; noted so nobody measures furniture with it.
- RTAB-Map with synthetic depth is a known-community pattern, not a supported
  configuration — expect tuning.
- The C922's rolling shutter and auto-exposure fight feature tracking; fixed
  exposure (camera.md#v4l2-controls) will matter again at P3.
