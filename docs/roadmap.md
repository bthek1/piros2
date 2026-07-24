# Learning roadmap

The point of this project is learning ROS 2, so the milestones are ordered by
concept rather than by feature. Each one should end with something that visibly
works before moving on.

No ROS packages are written yet — the repository is still documentation only. The
one thing that *is* done is the Pi's OS.

## 0. Environment — *in progress*

- [x] Reflash the Pi to Ubuntu Server 24.04 arm64 — done 2026-07-23, [setup.md](setup.md)
- [x] Write the Ansible roles — done 2026-07-24, all five in `ansible/roles/`
- [ ] `ansible-playbook site.yml` green on both machines — **Pi done and
      idempotent (`changed=0`) 2026-07-24**; the dev box needs an interactive
      `ansible-playbook site.yml --ask-become-pass` because sudo prompts there
- [ ] `talker`/`listener` across the LAN — [networking.md](networking.md)

**Done when:** a node on the Pi and a node on the dev box exchange messages.
This is the milestone most likely to eat time; everything else depends on it.

Read the plain-`apt` equivalent in [setup.md](setup.md#5-run-the-playbook) as you
write the roles. The point is not to avoid learning what the install does — it is
that three environment variables have to match across two machines, which is worth
doing once properly rather than repeatedly by hand.

**Concepts:** platform tiers and why Ubuntu 24.04 is not an arbitrary choice, the
ROS apt source, overlay vs underlay sourcing, and the environment variables DDS
actually reads.

## 1. First node — *not started*

Write a trivial Python publisher and subscriber by hand rather than running the
demo nodes. The aim is the mechanics, not the result.

- [ ] `ros2 pkg create --build-type ament_python piros2_hello`
- [ ] A publisher on a timer, a subscriber that logs
- [ ] `colcon build`, source the overlay, run it

**Concepts:** package layout, `setup.py` entry points, `rclpy` node lifecycle,
the build/source cycle, `ros2 run` vs `ros2 launch`.

## 2. Camera up — *not started*

- [ ] `usb_cam` publishing `/image_raw` from the Pi
- [ ] Viewed live in `rqt_image_view` on the dev box over compressed transport
- [ ] Frame rate confirmed with `ros2 topic hz`

**Concepts:** sensor drivers, `sensor_msgs/Image`, image transport, QoS profiles
for sensor data (`BEST_EFFORT` and why). See [camera.md](camera.md).

## 3. Launch files & parameters — *not started*

Replace the long `--ros-args -p ...` command line with something declarative.

- [ ] `launch/camera.launch.py` starting the camera node
- [ ] Camera settings in `config/camera.yaml`
- [ ] Launch arguments for resolution and frame rate

**Concepts:** the Python launch system, parameter files, namespaces, remapping,
`ros2 param` at runtime.

## 4. An image-processing node — *not started*

The first node that does something rather than plumbing data.

- [ ] Subscribe to `/image_raw`, publish an annotated `/image_processed`
- [ ] Start with something trivially verifiable — edge detection, or a colour blob
      tracker publishing the centroid
- [ ] `cv_bridge` for the ROS ↔ OpenCV conversion

**Concepts:** `cv_bridge`, subscriber/publisher in one node, keeping per-frame work
off the executor thread, measuring latency end to end.

## 5. TF and coordinate frames — *not started*

- [ ] A static transform from `base_link` to `camera_link`
- [ ] The camera frame visible in RViz with the image display attached
- [ ] Camera calibrated so `camera_info` is real — [camera.md](camera.md#calibration)

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
