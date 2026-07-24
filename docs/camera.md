# Publishing the C922 to ROS 2

The camera is a Logitech C922 Pro Stream on `/dev/video0`. Its capture modes are
listed in [hardware.md](hardware.md#capture-modes); the short version is **MJPG
1280×720 @ 30 fps** as the default. 30 fps is the working ceiling at every
resolution — the advertised 720p60 mode negotiates but measures ~29.7 fps.

**Confirmed working (2026-07-23):** a 1280×720 MJPG frame captured over SSH decodes
to a correct, sharp, well-exposed image, and sustained capture holds exactly
30.00 fps *with a fixed exposure*. On stock auto-exposure settings it runs at
18–21 fps instead — read [the frame-rate note](hardware.md#frame-rate-and-auto-exposure)
before benchmarking anything.

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

```bash
# on the Pi
source /opt/ros/jazzy/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=$(readlink -f /dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5461327F-video-index0) \
  -p pixel_format:=mjpeg2rgb \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p framerate:=60.0 \
  -p camera_frame_id:=camera_link
```

Two lines there are load-bearing, both found the hard way (2026-07-24):

- **`readlink -f` around the by-id symlink.** `usb_cam` naively splices the
  symlink's relative target into `/dev/../../video0` and then rejects it as
  invalid. Resolve the stable name to the real device before passing it.
- **`framerate:=60.0`, deliberately above the camera's real 30.** `usb_cam`
  polls on a ROS timer at the requested rate, and at 30 the two 33 ms clocks
  (poll timer vs frame cadence) beat against each other: measured **24.0 fps**
  steady, with the camera provably delivering 30 to a raw V4L2 capture at the
  same moment. Polling at 60 (16 ms) catches every real frame: measured
  **29.72 fps** at 720p MJPG, manual exposure 150. The camera still runs at
  its ~30 fps ceiling — the parameter only changes how often the node looks.

Check it from the dev box:

```bash
ros2 topic hz /image_raw           # should sit near 30
ros2 topic bw /image_raw/compressed
ros2 run rqt_image_view rqt_image_view
```

A launch file is the better home for these parameters once they stop changing —
that is milestone 3 in the [roadmap](roadmap.md).

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
gains `compressed` and `theora` variants of its topic. Subscribe to the compressed
one from the dev box:

```bash
ros2 run rqt_image_view rqt_image_view /image_raw/compressed
```

In RViz, set the Image display's **Transport Hint** to `compressed` rather than
`raw`. Forgetting this is the usual reason RViz shows a frozen or stuttering image
while `ros2 topic hz` on the Pi reports a healthy 30 Hz.

Tune JPEG quality at runtime:

```bash
ros2 param set /usb_cam_node .image_raw.compressed.jpeg_quality 60
```

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
```

**One `--set-ctrl` per command, in this order.** A manual value is *inactive*
while its auto mode holds it, and an inactive control in a batched
`--set-ctrl=a,b` call fails the whole batch with a misleading
`Permission denied` (observed 2026-07-24). Disable the auto mode first, then
set the manual value in a separate call.

Control names vary between kernel versions — always confirm with `--list-ctrls`
rather than assuming. `usb_cam` exposes equivalents as ROS parameters, which is the
tidier route once you have found the values you want.

## Calibration

Needed before AprilTags, visual odometry, or anything that turns pixels into metres.

```bash
sudo apt install ros-jazzy-camera-calibration    # or add it to the Ansible role

ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/image_raw -p camera:=/camera
```

Print a checkerboard, hold it at varied distances, angles, and positions in the
frame until all four bars go green, then **Calibrate** → **Save**. The result lands
in `/tmp/calibrationdata.tar.gz`; extract the `.yaml` into `config/` in this repo
and point the driver at it with `camera_info_url`.

Calibration is per-camera and per-resolution. Recalibrate if you change capture
resolution — the intrinsics do not simply scale, because the C922 crops rather than
scales between some modes.
