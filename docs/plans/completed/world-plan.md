# World dashboard build plan — every feed in one window

> **Completed 2026-08-03.** Build order for `src/piros2_world`: a package that
> composes the camera feed, the neural depth estimate, and a new keypoint
> detector into a single dashboard image with live statistics — frames/sec
> per stream, keypoints in the current frame, cumulative counts — viewed in
> one `rqt_image_view` window. Stable phases, each ending with something you
> can run and check before the next starts. All four phases landed in one
> session on 2026-08-03; kept as the build log.

## Why a composed image, not a window full of widgets

Two honest ways to get "one window":

1. **rqt perspective** — several Image View plugins docked in one shell.
   Zero code, but no good home for the stats (rqt's topic-echo plugins dump
   raw text; nothing renders "13.2 depth/sec" legibly), and the layout lives
   in a `.perspective` file rather than in code you can reason about.
2. **A dashboard node** that subscribes to every feed, composes a 2×2 mosaic
   with a stats panel drawn via OpenCV, and publishes it as one compressed
   image topic. The viewer stays a stock `rqt_image_view`.

This plan builds (2). It is the more idiomatic split — nodes process,
viewers display — and it exercises the ROS concepts this repo is for:
multi-subscription callback design, measuring rates against the right clock,
and publishing a derived image stream. It also means the dashboard works
over a bag with no hardware attached, and anyone on the LAN can open the
same dashboard topic.

## Preconditions, verified

| Have | Where |
| --- | --- |
| Camera at ~30 fps, compressed over Wi-Fi, started over SSH by recipes | `just cam`, `src/piros2_camera` |
| Depth estimator on the dev-box GPU, ~13 fps, `/depth/preview/compressed` | `src/piros2_perception`, `just depth` |
| Record/replay loop so hardware can stay off during development | `just replay`, `bags/static1` |
| RELIABLE QoS pattern for megabyte frames (`BIG_FRAME_QOS`) | `piros2_vision/edge_detector.py` |
| Hand-rolled compressed publish from Python (image_transport is C++-only) | same file |
| Test conventions: anchored linters, `pytest.ini`, `.vscode` picker | any existing package |

Constraints that shape the phases:

- **`header.stamp` lags wall clock ~0.73 s** on this camera. Every rate on
  the stats panel is measured by counting *arrivals against the dashboard's
  own clock* — never by differencing header stamps ([camera.md](../../info/camera.md#timestamps)).
- **BEST_EFFORT receives zero large frames** — every subscription in this
  package is RELIABLE/KEEP_LAST, copied from the vision node.
- **Only `/compressed` topics cross the LAN.** The dashboard republishes
  JPEG, not raw — three raw streams would be ~250 MB/s.
- The depth stream runs at ~13 fps while the camera runs at ~30. The stats
  panel will show that gap plainly — that mismatch being *visible* is half
  the point of the dashboard (and feeds the standing "reduce compute" todo).
- Keypoint detection and the dashboard use system OpenCV via `cv_bridge`,
  so they run under `ros2 run` normally. Only the depth estimator needs the
  venv escape hatch, and the launch file inherits `perception.launch.py`'s
  `ExecuteProcess` pattern for it.

## The new package

```
src/piros2_world/                  # ament_python — PLANNED
├── package.xml                    # rclpy, sensor_msgs, std_msgs, cv_bridge
├── setup.py
├── piros2_world/
│   ├── keypoint_detector.py       # P1: ORB on the camera feed → annotated image + count
│   └── dashboard.py               # P2: mosaic + stats panel → /world/dashboard/compressed
├── launch/world.launch.py         # P3: depth (venv ExecuteProcess) + keypoints + dashboard
├── config/world.yaml              # ORB feature cap, panel size, stats window length
└── test/                          # anchored linters + unit tests per phase
```

Conventions carried over: `piros2_<thing>` naming, anchored linter tests
copied from an existing package (never `ros2 pkg create`'s CWD-dependent
form), package added to `.vscode/tasks.json`'s picker, recipes added to the
justfile as they appear, parameters in `config/world.yaml` rather than
`--ros-args` strings.

## P0 — Scaffold ✓ (2026-08-03)

Create the package skeleton with no nodes yet: `package.xml`, `setup.py`,
empty module, anchored linter tests, `.vscode/tasks.json` picker entry.

**Runnable check:** `colcon build --symlink-install` then `just test` —
suite green with the new package counted, style-clean.

## P1 — Keypoint detector ✓ (2026-08-03)

> Landed as planned. Measured on the dev box: 500 keypoints (the cap, on a
> real scene) at ~14 ms/frame; against `bags/static1` the count topic ran
> at 14.85 Hz vs the bag's own 14.9 fps — the detector keeps up exactly.

`keypoint_detector.py`: subscribe `/image_raw/compressed` (RELIABLE),
run OpenCV **ORB** per frame — chosen because it is fast enough for the
full 30 fps stream on CPU, patent-free, and in the OpenCV already installed —
and publish:

- `/keypoints/compressed` (`CompressedImage`) — the frame with
  `cv2.drawKeypoints` overlaid, JPEG-encoded by hand on the conventional
  `<topic>/compressed` name so stock viewers find it.
- `/keypoints/count` (`std_msgs/Int32`) — keypoints found in that frame.
  A plain Int32 rather than a custom message: custom interfaces need a
  separate `ament_cmake` rosidl package, which this dashboard doesn't
  justify. If a richer stats contract is ever wanted, that becomes its own
  phase.

Parameters in `config/world.yaml`: `max_features` (ORB cap, default 500),
JPEG quality. Per-frame cost logged against the node's own clock, once,
the way the edge detector does.

**Unit tests** need no camera: feed `on_frame` a synthetic chessboard-ish
image through a captured publisher (the `piros2_vision` test pattern) and
assert keypoints were found, the annotated image round-trips, and the count
matches.

**Runnable check:** `just replay` variant against `bags/static1` with the
node running; annotated corners visible in `rqt_image_view
/keypoints/compressed`; `ros2 topic hz /keypoints/count` ≈ the bag's frame
rate. Add a `just keypoints` recipe (camera over SSH + node + viewer,
`pkill -f` cleanup — never `kill %N`).

## P2 — Dashboard node ✓ (2026-08-03)

> Landed as planned, plus a STALE banner unit test. Verified against
> `bags/static1` with the depth estimator and P1 node running: the mosaic
> published at a measured 10.1 Hz, all three panels live, stats reading
> camera 14.8/s, depth 14.5/s, keypoints 14.8/s (the bag's own rate is
> 14.9 fps — on a bag the GPU depth node keeps up; the ~13 fps gap only
> appears against the live 30 fps camera).

`dashboard.py`: three RELIABLE subscriptions, latest-wins with no
synchroniser — a dashboard wants "the newest of each", and the streams run
at deliberately different rates, so `message_filters` sync would be wrong
here (contrast with `cloud_projector`, where per-pair correctness demanded
it — that contrast is the lesson of this phase):

- `/image_raw/compressed` — camera panel
- `/depth/preview/compressed` — depth panel
- `/keypoints/compressed` + `/keypoints/count` — keypoint panel and stats

Each callback stores the decoded frame and appends an arrival time (node's
own clock) to a per-topic deque. A wall-timer at ~10 Hz composes:

```
┌────────────┬────────────┐
│ camera     │ depth      │
├────────────┼────────────┤
│ keypoints  │ stats      │
└────────────┴────────────┘
```

Panels resized to a common cell (e.g. 640×360, parameterised), stats panel
rendered with `cv2.putText`: camera fps, depth estimates/sec, keypoint
frames/sec (each = deque length over its time span), keypoints in the
current frame, cumulative totals since start, and a staleness marker when a
panel's newest frame is older than a threshold — measured against *receipt*
time, never `header.stamp` (the 0.73 s fault makes stamp-age gates drop
everything). Output: `/world/dashboard/compressed`, JPEG.

Composition and stats maths live in pure functions (`compose_grid(frames)`,
`rates(deques, now)`) so the unit tests exercise them with synthetic arrays
and hand-built deques — no ROS graph, no model weights, matching how the
perception tests avoid onnxruntime.

**Runnable check:** bag replay + depth estimator + P1 node + dashboard;
one `rqt_image_view /world/dashboard/compressed` window shows all four
panels, rates plausible (camera ≈ bag rate, depth ≈ 13/sec on GPU),
staleness marker appears when the bag ends. Unit tests green.

## P3 — Launch file, recipe, live run ✓ (2026-08-03)

> Landed as planned; `just run` now points at `just world`. The live run
> measured the dashboard at 10.1 Hz with depth at ~14/s (GPU) — and the
> camera panel at **59.7 msgs/s, not ~30**. The duplicate-frame hypothesis
> written here on the day was tested and refuted on 2026-08-04: every
> payload is distinct, and the C922 really does deliver 42–60 fps at 720p60
> under the camera-reset baseline, tracking auto-exposure — the old "30 fps
> ceiling" fell ([hardware.md](../../info/hardware.md#capture-modes)). The
> stats panel making that visible is the dashboard doing its job; whether to
> throttle belongs to the standing "reduce compute" todo, not this plan.

`launch/world.launch.py`, following `perception.launch.py`'s rules: the
depth estimator via `ExecuteProcess` under the venv interpreter (a
launch_ros `Node` would exec the system-shebang entry point and lose
onnxruntime), keypoint detector and dashboard as ordinary `Node`s, and it
deliberately does **not** include the camera launch — that would open
`/dev/video0` on the dev box.

`just world`: start the camera on the Pi over SSH (login shell — a bare
`ssh pi '...'` lands on domain 0), launch the file, open
`rqt_image_view /world/dashboard/compressed` with the `PATH="/usr/bin:$PATH"`
pin (PlatformIO venv shadows rqt's shebang); closing the viewer tears
everything down via `pkill -f` on node name patterns.

**Runnable check:** `just world` from cold on live hardware — one window,
four panels, live stats; camera panel ~30 fps and depth ~13/sec on the
panel itself; `just test` still green. Then the bookkeeping that closes the
plan: update the docs map and current-state notes in `CLAUDE.md` and
`README.md`, and move this file to `docs/plans/completed/` — the move *is*
the status change; fix inbound links.

## Out of scope, recorded so nobody wonders

- **Keypoint *matching* / tracking across frames** (the road to visual
  odometry) — a different project; this package only detects and counts.
- **A custom stats message type** — needs a rosidl `ament_cmake` package;
  Int32 + the dashboard's own arrival clocks cover today's stats.
- **Point-cloud or RViz integration** — the cloud stays in `just cloud`'s
  RViz; mixing 3D into a 2D mosaic buys nothing here.
- **Web dashboards** (Foxglove, rosbridge) — worth a look someday, but this
  plan stays inside the stock Jazzy toolset.
