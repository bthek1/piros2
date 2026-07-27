# Learning roadmap

The point of this project is learning ROS 2, so the milestones are ordered by
concept rather than by feature. Each one should end with something that visibly
works before moving on.

Status at a glance: **milestones 0–4 are done** (2026-07-24 → 2026-07-27) —
provisioned machines, `piros2_hello`, the camera stack, launch files, and the
edge-detection pipeline, each verified end to end. Milestone 5 (TF and
calibration) is next.

## 0. Environment — *done 2026-07-24*

- [x] Reflash the Pi to Ubuntu Server 24.04 arm64 — done 2026-07-23, [setup.md](setup.md)
- [x] Write the Ansible roles — done 2026-07-24, all five in `ansible/roles/`
- [x] `ansible-playbook site.yml` green on both machines — idempotent
      (`changed=0`) on both as of 2026-07-24. The dev box's first run tripped
      over a pre-existing kernel/DKMS failure, resolved the same day —
      [troubleshooting.md](troubleshooting.md#apt-fails-on-linux--kernel-packages-dev-box)
- [x] `talker`/`listener` across the LAN — verified 2026-07-24: `/chatter`
      published on the Pi arrived on the dev box (domain 42, CycloneDDS pinned,
      Wi-Fi multicast worked without static peers)

**Done when:** a node on the Pi and a node on the dev box exchange messages. ✓

Read the plain-`apt` equivalent in [setup.md](setup.md#5-run-the-playbook) as you
write the roles. The point is not to avoid learning what the install does — it is
that three environment variables have to match across two machines, which is worth
doing once properly rather than repeatedly by hand.

**Concepts:** platform tiers and why Ubuntu 24.04 is not an arbitrary choice, the
ROS apt source, overlay vs underlay sourcing, and the environment variables DDS
actually reads.

## 1. First node — *done 2026-07-24*

Write a trivial Python publisher and subscriber by hand rather than running the
demo nodes. The aim is the mechanics, not the result.

- [x] `ros2 pkg create --build-type ament_python piros2_hello`
- [x] A publisher on a timer, a subscriber that logs — `src/piros2_hello/`,
      concept notes in the source
- [x] `colcon build`, source the overlay, run it — and beyond the letter of the
      milestone: built on both machines and verified cross-LAN (`just hello`),
      the Pi's listener hearing the dev box's talker on `/hello`

One trap found on the way: ROS 2 loggers write to **stderr**, so piping a
node's output through `grep` without `2>&1` shows nothing and looks like a
discovery failure.

**Concepts:** package layout, `setup.py` entry points, `rclpy` node lifecycle,
the build/source cycle, `ros2 run` vs `ros2 launch`.

## 2. Camera up — *done 2026-07-24*

- [x] `usb_cam` publishing `/image_raw` from the Pi — with two traps found and
      documented in [camera.md](camera.md#running-it): `usb_cam` mangles the
      by-id symlink (resolve with `readlink -f`), and its poll timer beats
      against the frame cadence at `framerate:=30` (24.0 fps measured; request
      60 to poll fast enough)
- [x] Viewed live in `rqt_image_view` on the dev box over compressed transport
      (`just cam` — camera + viewer + cleanup in one recipe). Getting there
      surfaced four distinct faults, each now in
      [troubleshooting.md](troubleshooting.md): the PlatformIO venv shadowing
      `python3` for env-shebang rqt tools; `image_transport_plugins` missing on
      the *subscriber* side (`desktop` does not include it); leftover manual
      exposure persisting inside the camera; and usb_cam force-setting
      `brightness` to 50. Image tuning lives in `camera.yaml` (brightness 128,
      JPEG re-encode quality 90) and the `gain:=` launch argument — gain is
      never auto-adjusted on Linux
- [x] Frame rate confirmed with `ros2 topic hz` — **29.72 fps** at 720p MJPG,
      manual exposure 150, `framerate:=60` polling; 24.0 fps at
      `framerate:=30`; ~23 fps on stock auto-exposure. Raw V4L2 capture
      confirmed the camera itself delivers 30.

**Concepts:** sensor drivers, `sensor_msgs/Image`, image transport, QoS profiles
for sensor data (`BEST_EFFORT` and why). See [camera.md](camera.md).

## 3. Launch files & parameters — *done 2026-07-24*

Replace the long `--ros-args -p ...` command line with something declarative.

- [x] `launch/camera.launch.py` starting the camera node — `src/piros2_camera/`,
      runs at the verified 29.6 fps; `just cam` uses it
- [x] Camera settings in `config/camera.yaml` — applied and checked with
      `ros2 param get` rather than assumed, which caught a real bug: the
      parameter is `frame_id`, and the `camera_frame_id` used until now was
      silently ignored (headers said `default_cam`) —
      [camera.md](camera.md#running-it)
- [x] Launch arguments for resolution and frame rate — verified end to end:
      `image_width:=640` arrived in the node and in the published frames

**Concepts:** the Python launch system, parameter files, namespaces, remapping,
`ros2 param` at runtime.

## 4. An image-processing node — *done 2026-07-27*

The first node that does something rather than plumbing data.

- [x] Subscribe to `/image_raw`, publish an annotated `/image_processed` —
      `src/piros2_vision/edge_detector.py`: Canny edges drawn green over the
      frame at ~16 fps (processing ~30–45 ms/frame, RELIABLE/KEEP_LAST-1 QoS),
      plus a hand-rolled `/compressed` variant that crosses the Wi-Fi
      (image_transport is C++-only, so Python publishers get no automatic one)
- [x] Start with something trivially verifiable — edge detection
- [x] `cv_bridge` for the ROS ↔ OpenCV conversion

Two findings along the way: a freshness gate keyed on `header.stamp` dropped
100% of frames, exposing the camera's **~0.73 s timestamping fault**
([camera.md](camera.md#timestamps)) — per-frame cost is now measured against
the node's own clock only; and **BEST_EFFORT delivered literally zero frames**
while RELIABLE worked: a 2.7 MB raw frame fragments into ~1800 UDP datagrams
even on loopback, at least one always drops with default socket buffers, and
best-effort never retransmits, so no frame ever reassembles. The textbook
"sensor data = BEST_EFFORT" assumes messages small enough to survive intact;
megabyte frames invert it. Provable either way with
`ros2 topic echo --qos-reliability {best_effort,reliable} /image_raw`.

Closed out 2026-07-27: `launch/vision.launch.py` composes the camera's launch
file via `IncludeLaunchDescription` (camera arguments pass through the shared
launch context — `just edges gain:=128`), verified 19.6 fps annotated output
from one command and eyeballed live in `rqt_image_view` on the dev box; and
`test/test_edge_detector.py` unit-tests `on_frame` directly — synthetic frame
in, captured publishers out, no camera, graph or discovery involved
(`just test`).

**Concepts:** `cv_bridge`, subscriber/publisher in one node, keeping per-frame work
off the executor thread, measuring latency end to end.

## 5. TF and coordinate frames — *in progress (2026-07-27)*

- [x] A static transform from `base_link` to `camera_link` — a
      `static_transform_publisher` in `camera.launch.py` (placeholder pose:
      5 cm up, identity rotation — re-measure when the camera is mounted
      deliberately). Verified across the LAN with
      `ros2 run tf2_ros tf2_echo base_link camera_link`; note the `At time
      0.0` — static transforms are latched on `/tf_static` and valid at any
      query time
- [ ] The camera frame visible in RViz with the image display attached —
      `just pipeline` on one terminal, `rviz2` on another (Fixed Frame
      `base_link`, add TF + Image with compressed transport)
- [ ] Camera calibrated so `camera_info` is real — [camera.md](camera.md#calibration);
      needs a printed checkerboard and a human in front of the camera

**Concepts:** `tf2`, frame conventions (REP-103, REP-105), `static_transform_publisher`,
why a correct `camera_info` matters before any geometry.

## 6. Recording and replay — *not started*

- [ ] `ros2 bag record` a camera session on the Pi
- [ ] Replay it on the dev box and run the processing node against the recording

**Concepts:** `rosbag2`, MCAP storage, `/clock` and `use_sim_time`. This is the
step that makes the rest of the project pleasant — you stop needing the hardware
powered on to iterate.

## 7. Something interactive — *not started*

Pick whichever is more interesting at the time:

- **AprilTag detection** — `apriltag_ros`, publishing tag poses into TF. Real 6-DOF
  pose estimation, and a good test of the calibration.
- **Pan/tilt servo mount** — introduces actuators, control loops, and the
  visual-servoing feedback path. Note the reflash removed the `gpio`/`i2c`/`spi`
  groups Raspberry Pi OS provided, so this now needs the group and a udev rule
  creating first — see [hardware.md](hardware.md#group-membership).
- **Web dashboard** — `rosbridge_suite` plus a browser client. Introduces the
  ROS-to-non-ROS boundary.

## A note on tooling milestones

Ansible is provisioning, not ROS. It earns a place in milestone 0 because three
environment variables have to match across two machines — not because the project
needs a configuration-management layer for its own sake. Resist growing it beyond
that; the milestones above are the point.

## Reference

- [ROS 2 Jazzy tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) — work through
  the beginner CLI and client-library sections in order; they are genuinely good.
- [ROS 2 design docs](https://design.ros2.org/) — for the *why* behind DDS and QoS.
- [REP-103](https://ros.org/reps/rep-0103.html) / [REP-105](https://ros.org/reps/rep-0105.html) —
  units and coordinate frame conventions. Worth reading once, early.
