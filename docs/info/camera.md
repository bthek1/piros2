# Publishing the C922 to ROS 2

The camera is a Logitech C922 Pro Stream on `/dev/video0`. Its capture modes are
listed in [hardware.md](hardware.md#capture-modes); the short version is **MJPG
1280×720** as the default. The long-standing "30 fps ceiling" was overturned on
re-measurement (2026-08-04): 720p60 delivers 42–60 *distinct* frames/s under
the camera-reset baseline, the rate tracking the auto-exposure time —
[hardware.md](hardware.md#capture-modes) has the numbers and caveats.

**Confirmed working (2026-07-23):** a 1280×720 MJPG frame captured over SSH decodes
to a correct, sharp, well-exposed image, and sustained capture holds exactly
30.00 fps *with a fixed exposure*. On stock auto-exposure settings it runs at
18–21 fps instead — read [the frame-rate note](hardware.md#frame-rate-and-auto-exposure)
before benchmarking anything.

## Handling rules

The invariants for any code, doc or session that touches the camera — each
learned the hard way, detail in the linked sections:

1. **The camera belongs to the Pi.** Dev-box launches must never open
   `/dev/video0` — recipes start the camera over SSH, and a dev-box launch
   file must not `IncludeLaunchDescription` the camera launch (the include
   executes locally; `perception.launch.py` documents the split).
2. **Capture is exclusive; controls are shared.** One process streams at a
   time — a second opener fails with `VIDIOC_REQBUFS … Device or resource
   busy` (verified 2026-08-01). Reading and setting V4L2 controls works
   fine *alongside* an active stream, so state can be inspected and tuned
   live.
3. **Fail loudly when the camera is missing** — never idle on a dead
   device or stream ([below](#when-the-camera-is-missing)).
4. **Camera state persists inside the camera.** Controls survive
   processes, node restarts and reboots; check `just camera` and restore
   the baseline with `just camera-reset` *before* debugging black frames
   or low fps as a software problem ([below](#camera-state)).
5. **Address it by the serial-keyed symlink**, resolved through
   `readlink -f`/`os.path.realpath` because usb_cam mangles symlinks;
   `/dev/video0` can renumber across replugs, and `/dev/video1` is the
   metadata node, never capture ([Running it](#running-it)).
6. **`v4l2-ctl` is the only working channel for exposure and focus** —
   usb_cam's equivalent ROS parameters use ROS 1-era control names this
   kernel no longer has ([V4L2 controls](#v4l2-controls)).
7. **Never trust `header.stamp` for freshness or latency** — the driver
   stamps ~0.73 s behind wall clock ([Timestamps](#timestamps)).
8. **Never stream raw images across the LAN** — compressed transport only;
   raw 720p30 is ~83 MB/s ([Image transport](#image-transport)). Two
   corollaries, both hit live on 2026-08-16
   (docs/info/troubleshooting.md#a-live-session-crawls-at-2-fps-while-the-pis-wi-fi-is-saturated):
   never remap a dev-box subscriber to a topic name usb_cam also
   publishes (`/image_raw` is its *raw* topic — DDS matches by name and
   streams raw over the Wi-Fi), and give a session **one** Wi-Fi reader
   of the compressed stream — every extra RELIABLE reader pulls its own
   unicast copy, and five collapsed the link; fan out locally
   (`camera_relay` in `piros2_world_mesh`).
9. **Never quote a frame rate without stating the exposure mode**, and run
   usb_cam at `framerate:=60` to get the camera's real 30
   ([Running it](#running-it)).
10. **A hand-started camera is yours to stop.** Ad-hoc runs outside the
    recipes — debugging one-offs, Claude's verification runs — have no
    EXIT trap, so bound them up front
    (`ssh pi 'timeout -s INT 30 bash -lc "…"'`) or kill them by pattern
    when done
    (`ssh pi 'pkill -f "ros2 [l]aunch piros2_camera"; pkill -f usb_cam_[n]ode_exe'`),
    and finish with `just stragglers` — clean on both hosts — before
    reporting a result. A leaked usb_cam is worse than noise: it holds the
    exclusive capture (rule 2), so every later session dies with
    `Device or resource busy`.

## Driver choice

| Package | Notes |
| --- | --- |
| **`usb_cam`** | **Recommended.** Handles MJPG decode, exposes the full set of V4L2 controls (exposure, white balance, autofocus) as ROS parameters, and publishes `camera_info`. Best fit for a UVC webcam. |
| `v4l2_camera` | Simpler and lighter. Fine for a first "is anything working" test, but weaker control exposure and less flexible about pixel formats. |
| `libcamera` / `rpicam` | **Not applicable.** This is the Raspberry Pi CSI-ribbon path. There is no CSI camera attached, and the C922 is a plain UVC device. |

Both `usb_cam` and `v4l2_camera` are installed on the Pi by the Ansible
`ros2_install` role — [ansible.md](ansible.md). They exist as `ros-jazzy-usb-cam`
and `ros-jazzy-v4l2-camera` in the `noble/arm64` suite, verified present alongside
`ros-jazzy-image-transport-plugins` and `ros-jazzy-camera-calibration`.

## What gets published

A camera driver node publishes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/image_raw` | `sensor_msgs/msg/Image` | Decoded frames |
| `/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG frames (needs `image_transport_plugins`) |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsics, distortion, frame size |

`camera_info` is empty-ish until you calibrate. Anything doing real geometry —
AprilTags, depth, visual odometry — needs it populated, so calibrate before
attempting those.

## Running it

Since milestone 3 the camera runs from a launch file (`just cam` from the dev
box, or on the Pi):

```bash
ros2 launch piros2_camera camera.launch.py
ros2 launch piros2_camera camera.launch.py image_width:=640 image_height:=480
```

Stable parameters live in `src/piros2_camera/config/camera.yaml`; resolution,
frame rate and device are launch arguments. The equivalent bare `ros2 run`,
for understanding what the launch file abstracts:

```bash
# on the Pi (an interactive or login shell already has ROS sourced)
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=$(readlink -f /dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5461327F-video-index0) \
  -p pixel_format:=mjpeg2rgb \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p framerate:=60.0 \
  -p frame_id:=camera_link
```

Three things there are load-bearing, all found the hard way (2026-07-24):

- **`frame_id`, not `camera_frame_id`.** The ROS 1-era name is silently
  ignored — the node runs happily while stamping every header with
  `default_cam`, which surfaces much later as TF lookups failing in
  milestone 5. Check with `ros2 param get /usb_cam frame_id`.

- **`readlink -f` around the by-id symlink.** `usb_cam` naively splices the
  symlink's relative target into `/dev/../../video0` and then rejects it as
  invalid. Resolve the stable name to the real device before passing it.
- **`framerate:=60.0`, so the poll never becomes the bottleneck.** `usb_cam`
  polls on a ROS timer at the requested rate, and at 30 the two 33 ms clocks
  (poll timer vs frame cadence) beat against each other: measured **24.0 fps**
  steady, with the camera provably delivering 30 to a raw V4L2 capture at the
  same moment. Polling at 60 (16 ms) catches every real frame: measured
  **29.72 fps** at 720p MJPG, manual exposure 150 (2026-07-24). *Since the
  2026-08-04 re-measurement the camera itself delivers 42–60 fps in this mode
  (see the header note), so the parameter is no longer "above the real rate" —
  it is the real rate's ceiling, and every consumer sees whatever the
  auto-exposure allows.*

Check it from the dev box:

```bash
ros2 topic hz /image_raw           # 30–60 depending on exposure time (2026-08-04)
ros2 topic bw /image_raw/compressed
ros2 run rqt_image_view rqt_image_view
```

The launch file (`src/piros2_camera/launch/camera.launch.py`) is the home for
all of this — milestone 3 in the [roadmap](roadmap.md), done 2026-07-24. The
YAML/launch split: facts about the camera in `config/camera.yaml`, run-to-run
knobs as launch arguments. Parameter files are keyed by node name — a mismatch
between the YAML's top-level key and the node's launch `name=` applies nothing,
silently.

### When the camera is missing

Everything that touches the camera **fails loudly and exits** rather than
idling (rule adopted 2026-07-31). It has to be done by hand because `usb_cam`
itself does not: given a missing device it logs one ERROR and then sits
forever, publishing nothing — the launch's static transform publishers keep
spinning and every recipe downstream opens a viewer on a dead stream
(measured 2026-07-31).

Three layers enforce it:

- **`camera.launch.py` pre-flight-checks the device.** An `OpaqueFunction`
  runs after argument resolution, on the launching machine (the one with the
  camera), and raises if the resolved `video_device` is missing, not a
  character device, or (since 2026-08-16) already held open by another
  process — the error names the holding PID and command line, because a
  held device otherwise surfaces as usb_cam's unexplained `char*` abort
  (docs/info/troubleshooting.md#usb_cam-dies-at-startup-terminate-called-after-throwing-an-instance-of-char).
  The launch aborts with exit 1 before any node starts.
  `vision.launch.py` inherits the check through its
  `IncludeLaunchDescription`.
- **The usb_cam node carries `on_exit=Shutdown()`.** If the node dies
  mid-run (camera yanked, driver fault), launch reports it as *required*
  and takes the whole process tree down instead of leaving the TF
  publishers idling.
- **The recipes surface it.** `just cam` / `cloud` / `edges` / `depth`
  treat their warm-up seconds as a health check and exit 1 if the camera
  job died before the viewer opens; `just record` and `just calibrate`
  refuse to start when no `usb_cam` is running on the Pi.

Any new node, launch file or recipe that consumes the camera must follow the
same rule — see the conventions in `CLAUDE.md`.

## MJPG and the decode cost

`pixel_format:=mjpeg2rgb` asks the camera for compressed JPEG and decodes it to RGB
on the Pi's CPU. That is the right trade for USB bandwidth — MJPG is the only way
to get 720p60 or 1080p30 off this camera — but the decode is not free, and at 1080p30
it is a measurable slice of one core.

If the decode becomes the bottleneck, the options in order of preference are:

1. Don't decode on the Pi at all — publish `/image_raw/compressed` only and let the
   dev box decode. Cheapest by far when the Pi is just a sensor head.
2. Drop to 640×480, where decode cost is roughly a fifth of 1080p.
3. Lower the frame rate — 15 fps is plenty for most vision experiments.

Measure before optimising: `ssh pi top` while the node runs will tell you
immediately whether decode is actually the problem.

## Image transport

**This is the setting that decides whether streaming to the dev box works at all.**

Raw 1280×720 RGB8 at 30 fps is about **83 MB/s**. That is not a link you want to
put on the LAN, and in practice the publisher stalls and frame rate collapses.

With `image_transport_plugins` installed, every `Image` publisher automatically
gains `compressed` and `theora` variants of its topic. **Both ends need the
package**: the publisher to encode, the subscriber to decode — `desktop` does
not include it, so the dev box gets it from the `ros2_install` role (added
2026-07-24 after RViz-side subscribes failed with
`Unable to load plugin ... compressed_sub`). Subscribe to the compressed
one from the dev box:

```bash
ros2 run rqt_image_view rqt_image_view /image_raw/compressed
```

### Only the transports that are used

Five transport plugins are installed (`ros2 run image_transport list_transports`),
and by default a publisher advertises **all** of them: a live session showed
`/image_raw/theora`, `/image_raw/zstd` and `/image_raw/compressedDepth` beside the
two that are actually read, each with zero subscribers. `camera.yaml` now pins the
whitelist:

```yaml
image_raw.enable_pub_plugins:
  - image_transport/raw
  - image_transport/compressed
```

Measured on the Pi (2026-08-19, same run shape with a local
`/image_raw/compressed` subscriber attached): the topic list drops from 5 to 2,
while usb_cam's CPU goes 61.9% → 62.6% and the rate 29.86 → 29.90 fps. **The dead
plugins were costing nothing** — image_transport only encodes for a transport that
has a subscriber. What they cost was a reader's attention: an advertised topic
reads as an offer, and three of these five were never on the menu.

`raw` stays for local debugging **on the Pi** (`ros2 topic hz /image_raw` there).
Nothing on the dev box may subscribe it — that is the 2.7 MB-per-frame Wi-Fi
collapse of 2026-08-16
([troubleshooting](troubleshooting.md#a-live-session-crawls-at-2-fps-while-the-pis-wi-fi-is-saturated)).

In RViz, set the Image display's **Transport Hint** to `compressed` rather than
`raw`. Forgetting this is the usual reason RViz shows a frozen or stuttering image
while `ros2 topic hz` on the Pi reports a healthy 30 Hz.

The compressed stream is a **re-encode** — the pipeline is camera JPEG → RGB on
the Pi → JPEG again at this quality — which is why a local webcam app always
looks better than the remote view: it decodes once and never re-encodes.
`camera.yaml` pins the quality at 90 (the plugin default of 80 is visibly
soft; 90 roughly doubles frame size). If the Wi-Fi viewer stutters, this is
the knob to lower, live:

```bash
ros2 param set /usb_cam .image_raw.compressed.jpeg_quality 80
```

(Note the node name `/usb_cam` — it is set in the launch file and must match
`camera.yaml`'s top-level key.) None of this affects processing nodes on the
Pi: they subscribe to `/image_raw` directly and never pay the re-encode tax.

## Timestamps

**Do not trust `header.stamp` on this camera.** Measured 2026-07-24:

```bash
ros2 topic delay /image_raw    # → steady ~0.73 s, even on a freshly started camera
```

The frames themselves are visibly live — this is a UVC/driver timestamping
fault, not a queue anywhere in the pipeline. The lag is steady rather than
growing, which is the tell: real backlog accumulates.

Consequences, learned the expensive way in `piros2_vision`:

- **Never gate frame freshness on stamp age.** The first edge-detector
  version dropped 100% of frames "as stale" — silently, because a
  freshness gate failing looks identical to no frames arriving.
- **Never report stamp-to-now as pipeline latency.** It is dominated by the
  camera's clock fault; only spans between one process's *own* clock reads
  are trustworthy for measuring processing cost.
- The stamp remains fine for frame ordering and for associating a frame with
  other data stamped by the *same* fault — just not for comparison against
  any other clock.

## V4L2 controls

Webcam auto-exposure and autofocus will actively fight computer-vision algorithms:
brightness shifts frame to frame, and focus hunts whenever the scene changes. Lock
them down for anything involving detection or calibration.

There is a second, less obvious reason to do this: on stock settings **the C922
drops to 18–21 fps in ordinary indoor light**, because `exposure_dynamic_framerate`
lets it lengthen exposure past the frame interval. Fixing the exposure restores a
measured 30.00 fps. Frame rate here is a function of room lighting until you pin
it down.

> **`v4l2-ctl` is installed by the Ansible `camera` role** (done 2026-07-24).
> Ubuntu Server does not ship `v4l-utils` — Raspberry Pi OS did — so after a
> future reflash the commands below fail until
> `ansible-playbook site.yml --limit robot` has run.

```bash
# on the Pi
v4l2-ctl -d /dev/video0 --list-ctrls

v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0
v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=0        # 0 = infinity on the C922
v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1         # 1 = manual, 3 = aperture priority
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=150
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0
v4l2-ctl -d /dev/video0 --set-ctrl=gain=128                # 0-255; NEVER auto-adjusted on Linux
```

**Gain is the dim-room lever.** Aperture-priority auto-exposure on the C922
adjusts exposure *time* only; gain stays wherever it was set (factory default
0). In a dim room that reads as "camera works but the image is dark". The
launch file exposes it as `gain:=` (`just cam gain:=128`); higher is brighter
and noisier. `brightness` is different again — a post-capture offset that
usb_cam force-sets to 50 unless `camera.yaml` overrides it (it does, to the
camera's native 128).

**One `--set-ctrl` per command, in this order.** A manual value is *inactive*
while its auto mode holds it, and an inactive control in a batched
`--set-ctrl=a,b` call fails the whole batch with a misleading
`Permission denied` (observed 2026-07-24). Disable the auto mode first, then
set the manual value in a separate call.

Control names vary between kernel versions — always confirm with `--list-ctrls`
rather than assuming. `usb_cam` nominally exposes equivalents as ROS parameters,
but **they do not work on this kernel**: the node still uses the ROS 1-era
control names, and its startup log shows `unknown control 'exposure_auto'` /
`'focus_auto'` (renamed to `auto_exposure` / `focus_automatic_continuous` in
current kernels). `v4l2-ctl` is the only working channel for exposure and
focus here.

### Camera state

The controls above are **state that lives in the camera itself**, not in any
process: they persist across node restarts, sessions and reboots, until the
camera loses USB power. A manual exposure set for a benchmark stays set for
every later session — which is how the viewer showed black frames on
2026-07-24 — and the drift is invisible unless you look for it.

So treat camera state like any other machine state in this repo — inspect
it, don't assume it:

```bash
just camera        # devices, symlink, group — and every control, current vs default
just camera-reset  # restore the known-good baseline
```

`--list-ctrls` prints each control's `value` next to its `default`; any
mismatch is leftover state from an earlier session. Measured example
(2026-08-01): a freshly checked camera showed `exposure_dynamic_framerate`
at `value=1` against `default=0` — the camera *powers on* with it enabled
despite what the driver reports as default, and it is exactly the control
that drops indoor frame rate to 18–21 fps.

`just camera-reset` restores the baseline: all autos on (`auto_exposure=3`,
continuous autofocus, auto white balance), neutral image controls, `gain=0`,
zoom/pan/tilt home, and `exposure_dynamic_framerate=0` so frame rate stays a
function of the requested mode rather than of room lighting. Both recipes
are safe while the camera streams — control access works alongside an
active capture (verified 2026-08-01; only *capture* is exclusive). It
deliberately leaves `power_line_frequency` alone: set it to the local mains
frequency (`1` = 50 Hz, `2` = 60 Hz, the camera default) only if indoor
frames show rolling brightness bands.

The reset restores *defaults*, which is the recovery move — locking
controls down manually for CV work (the block above) remains a deliberate,
per-session act layered on top of a known baseline. Rule of thumb: run
`just camera` before believing any camera symptom is a software bug, and
run `just camera-reset` after any session that set manual values.

## Calibration

Needed before AprilTags, visual odometry, or anything that turns pixels into metres.

> **Interim state (2026-07-28):** `camera_info_url` points at
> `config/c922_720p_approx.yaml` — intrinsics *derived from the spec sheet*
> (78° diagonal FOV → fx = fy ≈ 907 px, centred principal point, zero
> distortion), not measured. `/camera_info` therefore carries a plausible K
> right now, good to a few percent. The checkerboard procedure below
> replaces it with measured values; when the saved `c922_720p.yaml` lands,
> re-point `camera_info_url` and delete the approx file.

The tooling is provisioned (`ros-jazzy-camera-calibration`, dev box only, via
the `extra_ros_packages` group var) and the board is in the repo:
**print [checkerboard-8x6-25mm.svg](checkerboard-8x6-25mm.svg) at 100% scale**
and verify a square is really 25 mm with a ruler — a scaled print silently
scales every distance the calibration will ever report. Mount it on something
flat; a floppy sheet of paper adds curvature the model will faithfully fit.

Then, with `just pipeline` running:

```bash
just calibrate
```

The recipe decompresses the stream locally onto `/calib/image_raw` before the
GUI subscribes — pointing the calibrator straight at `/image_raw` would pull
~83 MB/s of raw frames over the Wi-Fi — and PATH-prefixes the GUI past the
PlatformIO venv ([troubleshooting.md](troubleshooting.md#rqt-tools-crash-with-no-module-named-yaml)).

The recipe also remaps the calibrator's service client onto the driver's
`/usb_cam/set_camera_info` (it refuses to start otherwise) and watches the
GUI window: upstream ignores the window-manager close button entirely, so the
recipe stops the node itself when the window goes away. **q**, Esc, Ctrl-C
and closing the window all shut everything down cleanly —
[troubleshooting.md](troubleshooting.md#closing-the-calibrator-window-doesnt-stop-it).

Hold the board at varied distances, angles, and positions in the frame until
all four bars go green, then **Calibrate** (expect a long pause) → **Save**.
The result lands in `/tmp/calibrationdata.tar.gz`; extract `ost.yaml`, rename
it to `config/c922_720p.yaml` in `src/piros2_camera/`, add it to `setup.py`'s
config glob (already covered by `config/*.yaml`) and point the driver at it in
`camera.yaml`:

```yaml
camera_info_url: package://piros2_camera/config/c922_720p.yaml
```

Calibration is per-camera and per-resolution. Recalibrate if you change capture
resolution — the intrinsics do not simply scale, because the C922 crops rather than
scales between some modes.
