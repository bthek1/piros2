# Troubleshooting

Ordered roughly by how likely you are to hit them.

## `ssh pi` → REMOTE HOST IDENTIFICATION HAS CHANGED

The Pi's host keys were regenerated — reflashing the SD card does this.

```bash
ssh-keygen -f ~/.ssh/known_hosts -R 192.168.2.17
```

Before trusting the new key, confirm its fingerprint from the Pi's own console or
serial connection:

```bash
ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
```

Comparing against `ssh-keyscan` from the same machine proves nothing — a
man-in-the-middle would serve the same forged key to both.

Conversely, a host key that is **unchanged** after what should have been a fresh
install is evidence the machine booted the *old* system — see the next entry.

## The Pi booted the wrong OS after a reflash

Symptom: everything looks flashed correctly, but the machine comes up on the
previous install. Diagnose it rather than guessing — the firmware records which
medium it used (`01` = SD card, `04` = USB, `06` = NVMe):

```bash
ssh pi 'od -An -tx1 /proc/device-tree/chosen/bootloader/boot-mode'
ssh pi 'sudo rpi-eeprom-config | grep BOOT_ORDER'
```

`BOOT_ORDER` nibbles are read **right to left**, so `0xf14` is USB → SD → restart.
If the mode does not match the first nibble, that medium was tried and rejected.

- **USB target skipped after a warm reboot.** `reboot` restarts the SoC without
  power-cycling the USB ports, so a hot-plugged stick is often not re-enumerated
  inside the bootloader's discovery window. Cold power-cycle, or set
  `USB_MSD_PWR_OFF_TIME=3000` — see [setup.md](setup.md#booting-from-usb).
- **The image was not a Pi image.** A generic arm64 desktop or installer ISO has no
  bootloader partition and will never boot; you need the `+raspi` preinstalled image.
- **Falling through to the old system is by design** when the previous medium is
  still present and later in `BOOT_ORDER` — which is why keeping it is a good
  rollback, and why an unchanged host key is the tell.

## Two Ubuntu Pi images present, root filesystem is a coin-flip

Ubuntu's Pi image labels its partitions `system-boot` and `writable`, and
`cmdline.txt` uses `root=LABEL=writable`. Two copies of the same image carry
identical labels, filesystem UUIDs **and** PARTUUIDs, so the kernel roots into
whichever enumerated first.

```bash
ssh pi 'sudo blkid | grep -E "writable|system-boot"'   # duplicates = this bug
```

Give one a unique disk identifier and pin `cmdline.txt` and `/etc/fstab` to
PARTUUIDs — [setup.md](setup.md#the-duplicate-label-trap).

## The dev box cannot see topics published on the Pi

The most common failure in this setup. Work through in order:

1. **Domain ID mismatch.** `echo $ROS_DOMAIN_ID` on both — must both be `42`.
2. **Stale daemon.** `ros2 daemon stop && ros2 daemon start`. The daemon caches
   discovery state and will keep reporting the pre-fix view indefinitely.
3. **The environment is not actually set on the Pi.** A non-interactive
   `ssh pi 'ros2 topic list'` does **not** read the interactive part of `.bashrc`,
   so `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` silently come out empty and the node
   lands on domain 0 with the default RMW. Check what the remote shell really sees:
   ```bash
   ssh pi 'echo "domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION"'
   ```
   Empty output is the bug. Either use a login shell (`ssh pi -t 'bash -lc "..."'`)
   or set the values inline for that command.
4. **Wrong interface picked on the dev box.** Docker bridges and two VPN interfaces compete with
   `enp6s18`; DDS may bind to `172.1x.0.1` or the VPN at `10.8.0.3`. Pin the interface —
   [networking.md](networking.md#the-docker-bridge-problem-on-the-dev-box).
5. **RMW mismatch.** `echo $RMW_IMPLEMENTATION` on both. Fast DDS and Cyclone DDS
   do not interoperate for discovery in practice.
6. **`ROS_LOCALHOST_ONLY=1`** left set somewhere.

Confirm packets are actually crossing:

```bash
sudo tcpdump -i enp6s18 -n 'udp and portrange 7400-7500'
```

## Topics are visible but `ros2 topic echo` prints nothing

Discovery succeeded, data transport did not.

- **QoS incompatibility** — the usual cause. Sensor publishers typically use
  `BEST_EFFORT` reliability while `ros2 topic echo` defaults to `RELIABLE`, and
  incompatible pairs simply never connect. Force it:
  `ros2 topic echo /image_raw --qos-reliability best_effort`.
  `ros2 topic info /image_raw --verbose` shows both ends' QoS profiles.
- **A BEST_EFFORT subscriber on a large-message topic receives nothing at
  all — even on the same host.** Hit 2026-07-24: a 2.7 MB raw 720p frame
  fragments into ~1800 UDP datagrams; with default socket buffers at least one
  fragment of every frame drops, and best-effort never retransmits, so no
  frame ever reassembles. A RELIABLE subscription on the identical topic works
  instantly. Diagnose in one minute:
  ```bash
  ros2 topic echo /image_raw --qos-reliability best_effort --once   # nothing
  ros2 topic echo /image_raw --qos-reliability reliable    --once   # frame
  ```
  For megabyte messages, subscribe RELIABLE (the "sensor data = BEST_EFFORT"
  rule assumes small messages) — or raise `net.core.rmem_max` and Cyclone's
  socket buffer if best-effort semantics are genuinely needed.
- **MTU / fragmentation** — large messages (raw images) fragmented across UDP can
  be dropped wholesale by a switch. Subscribe to the compressed topic instead.

## `/dev/video0` not found, or permission denied

- **`v4l2-ctl: command not found` — check this first.** Ubuntu Server does not ship
  `v4l-utils`; the Ansible `camera` role installs it. If it is missing, the
  playbook has not run since the last reflash: `ansible-playbook site.yml --limit robot`.
- **Permission denied** means the login user is not in the `video` group. It *was*
  added by cloud-init at reflash time, so this should not happen — but if it does,
  `sudo usermod -aG video $USER`, then **log out and back in**; a group change does
  not affect the session that made it.
  ```bash
  ssh pi 'id -nG | tr " " "\n" | grep -x video'   # silence = not in the group
  ```
- **`/dev/video0` missing entirely** usually means the camera is unplugged rather
  than broken. The Pi's own ISP and decoder blocks occupy `/dev/video19`–`/dev/video37`
  and are always present, so "video nodes exist but no `video0`" is the signature of
  a detached camera:
  ```bash
  ssh pi 'ls /dev/video*; lsusb | grep -i logitech'
  ```
- Was the device re-enumerated after a replug? The node numbers can shift, and they
  shifted once already across the kernel change. Use the serial-keyed symlink in
  configs rather than `/dev/video0`:
  ```
  /dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5461327F-video-index0
  ```

## Camera launch aborts at startup: `camera not detected`

Deliberate, not a fault in the launch file (added 2026-07-31): `camera.launch.py`
pre-flight-checks the resolved `video_device` and aborts the whole launch —
and every recipe on top of it (`just cam`/`cloud`/`edges`/`depth`) — because
`usb_cam` given a missing device logs one ERROR and then idles forever,
which used to end as a viewer staring at a dead stream. The abort means the
camera genuinely is not there: work through
[`/dev/video0` not found](#devvideo0-not-found-or-permission-denied) —
unplugged cable, re-enumerated device, or a fresh reflash without the
playbook. Details of the fail-loudly rule:
[camera.md](camera.md#when-the-camera-is-missing).

## Camera opens but every frame is black or green

- **Leftover manual exposure — check this first.** V4L2 controls live *in the
  camera* and persist across processes, node restarts and reboots (until
  unplug), so a manual exposure set for a frame-rate benchmark silently
  applies to every later session. Hit 2026-07-24: `auto_exposure=1` +
  `exposure_time_absolute=136` + `gain=0` from earlier measurements produced
  pure black in an evening room. Diagnose and restore from the dev box:
  ```bash
  just camera          # every control, current vs default — mismatches are leftover state
  just camera-reset    # back to the known-good baseline
  ```
  (Equivalent by hand: `v4l2-ctl --get-ctrl=…` / `--set-ctrl=auto_exposure=3` on
  the Pi — [camera.md](camera.md#camera-state).)
- Another process already has the device. `ssh pi 'sudo fuser -v /dev/video0'`.
  UVC allows only one capture client at a time.
- The driver is pointed at `/dev/video1`, which is the UVC **metadata** node, not a
  capture device.
- Exposure was manually set far too low — see
  [camera.md](camera.md#v4l2-controls).

## Frame rate far below what was requested

- **Auto-exposure is stealing frames — check this first.** The C922 *powers on*
  with `exposure_dynamic_framerate=1` (despite the driver reporting `default=0`
  — measured 2026-08-01), letting it lengthen exposure past the frame interval
  in dim light. Measured on this camera: 18–21 fps at 720p on power-on state
  versus 30.00 fps with a fixed exposure. `just camera-reset` turns it off as
  part of the baseline; for a locked exposure on top of that:
  ```bash
  v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1
  v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=150   # separate call — batched, it fails "Permission denied"
  ```
  A frame rate that *drifts downward* over the first few seconds rather than
  sitting at a wrong-but-stable value is this, essentially every time.
- **A steady ~24 fps from `usb_cam` with exposure already fixed** is the node's
  poll timer beating against the frame cadence, not the camera: `usb_cam` grabs
  frames on a ROS timer at the requested `framerate`, and two 33 ms clocks drop
  ~1 frame in 5. Measured 2026-07-24: 24.0 fps via `usb_cam` while raw V4L2
  capture delivered 30 at the same settings. Fix: request `framerate:=60.0` so
  the poll runs at 16 ms — measured 29.72 fps —
  [camera.md](camera.md#running-it).
- **60 fps was requested at 720p.** The mode is advertised and the driver reports
  it as negotiated, but it measures ~29.7 fps. Treat 30 as the ceiling —
  [hardware.md](hardware.md#capture-modes).
- The requested resolution/format combination is not one the camera offers.
  `v4l2-ctl -d /dev/video0 --list-formats-ext` is authoritative; drivers silently
  fall back rather than erroring.
- YUYV was requested instead of MJPG. Uncompressed capture is USB-bandwidth-bound
  and will not reach 30 fps above 640×480.
- The subscriber is on the raw topic across the network. 83 MB/s does not fit;
  the publisher blocks. Use `compressed`.
- MJPG decode is saturating a core. Check with `ssh pi top`.

## RViz image display is frozen or stuttering

Set the Image display's **Transport Hint** to `compressed`. Left at `raw`, RViz
subscribes to the uncompressed topic across the LAN.

## `colcon build` fails on a missing dependency

```bash
cd ~/Documents/piros2    # on the Pi: cd ~/piros2
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` itself errors about an unknown key, the package is not in the rosdistro
index — install it manually with `apt`, then add it to the Ansible `ros2_install`
role so the machine stays reproducible.

If `rosdep` complains that it has never been initialised, `rosdep update` was run
under `sudo` at some point and wrote its cache into root's home. Re-run it as your
own user — see [ansible.md](ansible.md#gotchas-specific-to-provisioning-ros).

## Changes to a Python node have no effect

The workspace was built without `--symlink-install`, so `install/` holds a stale
copy. Rebuild:

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

Also confirm you sourced `install/setup.bash` *after* `/opt/ros/jazzy/setup.bash` —
order matters, workspace overlay last.

## An Ansible play fails or hangs

- **Hangs with no output** — a task needing `become` hit a `sudo` password prompt.
  Re-run with `--ask-become-pass`.
- **`Failed to connect to the host via ssh`** on the Pi, straight after a reflash —
  the host key changed. See the first entry on this page.
- **A task that should be a no-op reports `changed` every run** — usually
  `rosdep init`, which is not idempotent. It needs a `creates:` guard; the details
  are in [ansible.md](ansible.md#gotchas-specific-to-provisioning-ros).
- **Camera checks fail immediately after the `video` group is added** — the play is
  still on the pre-change session. The role needs a `meta: reset_connection`.

Run `ansible-playbook site.yml --check --diff` first when unsure; it shows exactly
what would change in `.bashrc` and the Cyclone DDS config without touching them.

## `apt` fails on `linux-*` kernel packages (dev box)

> **Resolved 2026-07-24** by option 1: `v4l2loopback-dkms` was installed but
> not in use (module not loaded, no virtual video devices, and noble offers no
> newer version), so it was removed and the pending kernels configured cleanly.
> Reinstate with `sudo apt install v4l2loopback-dkms` once Ubuntu ships a
> build compatible with kernel 7.x. Kept for the diagnosis pattern:

First seen 2026-07-24, when the first `site.yml` run on `ml5` reported the
`ros2_install` apt task failed even though every ROS package ended up installed
and configured (`dpkg -l` shows `ii`).

The failure is **not ROS**. `ml5` has a pending HWE kernel update
(`linux-image-7.0.0-28-generic`) whose post-install runs DKMS, and
`v4l2loopback/0.12.7` does not compile against kernel 7.0.0 — the
`v4l2_fh_del()` signature changed. dpkg leaves three `linux-*` packages
unconfigured, apt exits non-zero, and *every* subsequent apt operation
(including playbook runs) repeats the failure.

Fixes, in order of preference — this is a machine-owner decision because
`v4l2loopback` (virtual webcams, e.g. OBS) may be in use:

1. **If v4l2loopback is not needed:** `sudo apt remove v4l2loopback-dkms`,
   then `sudo dpkg --configure -a`.
2. **If it is:** install a newer v4l2loopback (0.13+) that supports the 7.0.0
   kernel API, then `sudo dpkg --configure -a`.
3. **Stopgap:** hold the new kernel (`sudo apt-mark hold linux-generic-hwe-24.04`)
   until 1 or 2 is done — the machine keeps running its current 6.17 kernel
   either way.

Until one of these is done, expect `ansible-playbook site.yml` to report the
dev box red on the apt task while actually changing nothing.

## rqt tools crash with `No module named 'yaml'` (dev box)

```
File "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/parameter.py" ...
ModuleNotFoundError: No module named 'yaml'
```

The dev box has PlatformIO's venv early in `PATH`, so `python3` resolves to
`~/.platformio/penv/bin/python3` — a Python with neither `yaml` nor `rclpy`.
ROS is built against the **system** 3.12 (see setup.md on why venvs and
`rclpy` don't mix), and the failure splits cleanly down shebang lines:

- Nodes built by colcon get `#!/usr/bin/python3` hardcoded — immune.
- The rqt/GUI tools use `#!/usr/bin/env python3` — they follow `PATH` into the
  venv and crash on import.

Fix for one command: prefix the PATH — `PATH="/usr/bin:$PATH" ros2 run
rqt_image_view rqt_image_view` (the `just cam` recipe does this). The
permanent fix is moving the PlatformIO line later in `.bashrc`/`.profile` or
dropping it, but that is a machine-owner call — other tooling may rely on it.

## `Unable to load plugin for transport 'image_transport/compressed_sub'`

`... Declared types are image_transport/raw_sub` from RViz or `rqt_image_view`
on the dev box means `image_transport_plugins` is missing **on the subscriber
side**. Compression is negotiated per endpoint: the Pi encodes, the dev box
decodes, and each needs the plugin package locally — `ros-jazzy-desktop` does
not include it (hit 2026-07-24). It is in the `ros2_install` role now, so a
machine showing this has not run the playbook since:

```bash
ros2 run image_transport list_transports   # 'compressed' must be listed
just deploy-dev                            # installs it if missing
```

## `image_transport republish` republishes nothing

Symptom: the output topic exists, `ros2 node info /image_republisher` shows
publishers but **no input subscription**, and nothing flows. Cause: the old
positional-argument form (`republish compressed raw`) is silently ignored on
Jazzy — the transports are node *parameters* now. Working form (hit
2026-07-27, used by `just replay`):

```bash
ros2 run image_transport republish --ros-args \
  -p in_transport:=compressed -p out_transport:=raw \
  -r in/compressed:=/image_raw/compressed -r out:=/image_raw
```

`ros2 node info` on the republisher is the fast diagnostic: no subscriber on
your input topic means the transport arguments never took.

## `just calibrate` dies at startup: `no camera service available`

The calibrator refuses to start until it can reach a `set_camera_info`
service. Its client is named `camera/set_camera_info`, and pointing it at the
driver is a **remap**, not a parameter — an earlier `-p camera:=/usb_cam` form
was silently ignored (hit 2026-07-27). The working form, now in the recipe:

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --ros-args -r image:=/calib/image_raw \
             -r camera/set_camera_info:=/usb_cam/set_camera_info
```

With the remap in place, this error means the service genuinely isn't there —
the camera stack isn't running. Start `just pipeline` (or `just cam`) first.

## Closing the calibrator window doesn't stop it

Upstream `cameracalibrator` only exits on **q**/**Esc** or the COMMIT button —
its display loop never checks whether the window still exists, and the next
`imshow` recreates a closed window within one frame. Since 2026-07-27 the
`just calibrate` recipe wraps it in a watchdog that kills the node when the
window's X id changes (the recreate mints a new id, so an id change *is* the
close). Two traps inside that fix, kept for the record:

- Qt5 apps in this Wayland session open **native Wayland surfaces** that
  `xwininfo` cannot see at all; the recipe forces `QT_QPA_PLATFORM=xcb` so
  the window exists in the X tree in the first place.
- mutter names its frame window after the client — "display" appears twice in
  the tree — and the frame arrives a beat after the client maps, which reads
  as an id change. The watchdog tracks only the client window (the entry with
  an empty class list `()`), whose id survives the reparenting.

## rviz2 crashes: "Unable to create the rendering window" (GLXContext, 100 tries)

Two independent layers, both hit 2026-07-28 on the first real rviz2 run:

1. **Wayland vs GLX — permanent.** rviz2's OGRE renderer creates its context
   through GLX, which only exists on X11. In this GNOME Wayland session Qt
   opens a native Wayland window, GLX has nothing to bind to, and context
   creation fails 100 times before rviz gives up. Fix: run rviz2 with
   `QT_QPA_PLATFORM=xcb` so the whole app lives on Xwayland (same class of
   trap as the calibrator's invisible window — Qt apps here prefer Wayland).
2. **NVIDIA driver mismatch — until the next reboot.** `nvidia-smi` says
   `Driver/library version mismatch`: apt upgraded the userspace to 595.84
   while the loaded kernel module is still 595.71.05, which breaks every GL
   application on the box. Diagnose:
   ```bash
   nvidia-smi                          # "Driver/library version mismatch"
   cat /proc/driver/nvidia/version     # loaded module vs dpkg -l nvidia-driver-*
   ```
   The fix is a reboot (loads the matching module). Until then, force Mesa's
   software path — and note `LIBGL_ALWAYS_SOFTWARE=1` alone is NOT enough:
   GLVND dispatches GLX to the (broken) NVIDIA vendor library regardless, so
   the vendor override is the load-bearing variable:
   ```bash
   QT_QPA_PLATFORM=xcb __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 rviz2
   ```
   llvmpipe reports OpenGL 4.5 and renders a 57k-point cloud without fuss.
   **Resolved 2026-07-30**: a reboot loaded the matching 595.84 module,
   verified by running rviz2 on hardware GL (OpenGL 4.6, no GLX errors),
   and the two mesa variables were dropped from the recipes. Layer 1 is
   permanent — `QT_QPA_PLATFORM=xcb` stays in every rviz2/Qt invocation.

## RViz opens but the mouse won't rotate/orbit the 3D view

Symptom: the scene renders, but dragging in the viewport does nothing —
no orbit, no pan, no zoom. Cause: the `-d` config had **no `Tools:`
section**. rviz2 populates its toolbar entirely from the loaded config;
with no tools there is no current tool, and mouse events in the render
panel are dispatched to nothing. A hand-minimal config (only Panels +
Displays + Views) triggers this; `rviz2` without `-d` never does, because
the stock `default.rviz` carries the tool list. Hit 2026-07-30 with
`perception.rviz`.

Fix: give every hand-written config the stock tool list (copy from
`/opt/ros/jazzy/share/rviz_common/default.rviz`) — at minimum
`Interact` + `MoveCamera` — and a fully-keyed `Views: Current:` block
with `Target Frame: <Fixed Frame>` so the Orbit controller has an anchor.

## Orphaned nodes keep logging into a terminal after a recipe ends

Symptom: `edge_detector` or `republish` output appears in a terminal that is
not running anything, whenever the camera streams. Cause: a recipe's cleanup
trap used `kill %N` — the background jobs are `bash -lc` wrappers, and killing
the wrapper orphans the actual ros2-run grandchildren. The orphans sit silent
(nothing publishes to their topic) until a live publisher comes back, then
haunt the terminal they were started from. Hit twice on 2026-07-27: a
republisher and an edge detector leaked from a `just replay`.

The `calibrate` and `replay` traps now `pkill -f` the node patterns instead,
and the fix hardened into a repo-wide rule (2026-08-12, CLAUDE.md
Conventions): **every session recipe must tear down everything it started,
on both machines, when its window closes or on Ctrl-C — no stragglers.**
The shape all the session recipes share: the viewer is the recipe's
foreground process, so closing its window ends the recipe, and a
`trap … EXIT` `pkill -f`s every node the recipe started — bash fires the
EXIT trap on Ctrl-C too, so one trap covers both exits. A session that
gains a node gets its pattern added to the trap in the same change.

The rule covers ad-hoc runs too — a camera or node started by hand (or by
Claude while verifying something) has no trap, so it must be bounded with
`timeout -s INT` up front or `pkill -f`'d when done, and the hands-on work
ends with a `just stragglers` check. The stray scratch node in the
RViz-flicker entry below is what skipping this looks like; a leaked
usb_cam additionally blocks every later session (exclusive capture —
[camera.md#handling-rules](camera.md#handling-rules), rule 10).

Diagnose survivors with `just stragglers` — it sweeps both machines and
prints `clean` per host when there are none — and clear them with
`pkill -f` on the printed patterns (`ssh pi 'pkill -f …'` for the Pi's).

## An RViz cloud flickers between two different shapes or sizes

Symptom: a PointCloud2 display alternates between two versions of the
scene at a steady rate. Cause: **two node instances publishing the same
topic**, each with its own idea of the content; RViz keeps only the
latest message, so the view flips at their combined publish rate. First
seen 2026-07-30 on the since-removed `cloud_fusion` node's map topic,
when a scratch instance (grid overridden for a bag test) outlived its
cleanup trap while `just cloud` launched the real one.

Diagnose with the publisher count, then find and kill the stray:

```bash
ros2 topic info -v /points   # Publisher count: 2 → there's a stray
pgrep -af cloud_projector
```

Same family as the orphaned-nodes entry above: ad-hoc background runs
need the same `pkill -f` hygiene as the recipes.

## onnxruntime ignores the GPU and runs on CPU

Symptom: `onnxruntime-gpu` is installed and
`ort.get_available_providers()` lists `CUDAExecutionProvider`, yet a
session created with CUDA first still reports
`get_providers() == ['CPUExecutionProvider']`, with an
`libcublasLt.so.13: cannot open shared object file` error above it.
Two causes stack (hit both on 2026-07-30):

1. **The wheel alone ships no CUDA libraries.** The CUDA 13 / cuDNN 9
   runtimes come from the pip extras —
   `pip install "onnxruntime-gpu[cuda,cudnn]"` (~1.5 GB of `nvidia-*`
   packages into the venv). The plain `onnxruntime` package has no CUDA
   provider at all.
2. **The loader can't see pip-installed CUDA libs.** They land in
   `site-packages/nvidia/*/lib`, which is on no library path. Call
   `onnxruntime.preload_dlls()` before creating the session — it dlopens
   them from inside the venv (`depth_estimator._load_model` does this).

The failure is *silent* at the API level: with a CPU fallback in the
providers list, the session simply runs slow. Always log
`session.get_providers()[0]` rather than assuming — the depth node does.

## `rtabmap-export` aborts: "The are no odometry poses!?"

Symptom: every `--opt` mode of `rtabmap-export` opens the database and
aborts (`Loaded 0 odometry poses`), even though `rtabmap-info` shows
nodes and odometry length.

The databases this repo produces (rtabmap 0.22.1, `-d` fresh-start,
bag-replay sessions) don't carry whatever table that tool reads. The
export that works is **`rtabmap-report --poses_raw ~/.ros/rtabmap.db`**,
which writes `rtabmap_odom.txt` and `rtabmap_slam.txt` next to the
database in TUM form (`t x y z qx qy qz qw` + a trailing node-id
column). `just map-headless <bag>` wraps the whole run and moves the
files next to the bag. Two facts to hold onto (2026-08-10): the poses
are **base_link**, not optical (`fuse_capture --poses-frame base`
converts), and their stamps are on the replay's clock, days away from
the capture's — `fuse_capture` removes the constant offset by median
difference.

## Exact-sync subscribers pair rarely, or not at all

Symptom: `rgbd_odometry` (or any exact-stamp `message_filters`
consumer of `/image_raw` + `/depth`) logs "Did not receive data since
5 seconds" while both topics demonstrably flow; odometry updates vary
run to run on identical input — 0 on one replay, 6 on the next.

Exact sync can only pair messages still in its queue. The depth node
publishes ~100 ms after its source frame, and at the camera's real
42–60 fps that means the matching RGB must survive ~6 newer arrivals —
a coin toss against the default queue of 5. Set
`topic_queue_size`/`sync_queue_size` ~30 (done in `mapping.launch.py`
and the `odom:=rgbd` mode, 2026-08-11): the same replay went from 0–6
to a deterministic 24 odometry updates. The 5-second watchdog warnings
themselves are normal at ~1–2 Hz paired rates (see the mapping notes in
CLAUDE.md) — the symptom is *variance* between identical runs, not the
warnings.

On the *live* camera, queue depth turned out not to be the whole story:
two independent decimators (a 60 fps republisher and the ~10 Hz depth
node) drop *different* frames, so their stamp sets rarely intersect and
no queue makes exact sync pair what never coexists. The fix
(2026-08-16, `piros2_world_mesh`) is `publish_rgb` on the depth
estimator: it republishes the exact frame it inferred on as `/depth/rgb`,
so every `/depth` stamp has its RGB twin by construction and pairing
runs at depth rate. See also the Wi-Fi entry below — the live 1–2 Hz
had a second, bigger cause.

## A live session crawls at ~2 fps while the Pi's Wi-Fi is saturated

Symptom: every consumer of the camera stream (depth, keypoints,
odometry, the mesher) limps at ~2 Hz; `rgbd_odometry` logs `delay=`
figures of 2–6 s that sawtooth; RViz's Depth3D flickers between a TF
error and rendering. Meanwhile the Pi's `wlan0` transmits 14+ MiB/s —
ten times what the compressed stream needs.

Two causes, both measured 2026-08-16, both invisible in bag replay:

- **A raw-topic collision.** usb_cam publishes raw `/image_raw`
  (2.7 MB/frame) alongside the compressed topic. Any dev-box
  subscription remapped to `/image_raw` — rgbd_odometry's `rgb/image`
  was, from the first `odom:=rgbd` session — matches the Pi's publisher
  by topic name and silently streams **raw video over the Wi-Fi**, the
  exact thing rule 8 forbids. The link saturates and every other topic
  starves. Never point a dev-box subscriber at a topic name usb_cam
  also publishes; the depth estimator's twin is named `/depth/rgb` for
  this reason.
- **One unicast copy per subscriber.** DDS sends each RELIABLE reader
  its own copy: five dev-box consumers of `/image_raw/compressed` =
  five copies over the Wi-Fi, and the retransmit storm collapses the
  link (measured: one reader receives the full rate at ~1.3 MiB/s;
  five readers each *complete* ~2 frames/s while the link burns
  14 MiB/s). `camera_relay` (in `piros2_world_mesh`) is the fix: the
  session's single Wi-Fi reader republishes the stream locally on
  `/camera_relay/compressed` and every consumer — RViz panels included —
  reads the loopback copy. Adding consumers is free; adding a direct
  Wi-Fi subscriber reintroduces the collapse.

The tell-tale diagnostic is the Pi's own TX counter, which needs no ROS
and perturbs nothing:
`ssh pi 'cat /sys/class/net/wlan0/statistics/tx_bytes'` twice, 10 s
apart. One clean compressed stream is ~1–3 MiB/s; double digits means
something is pulling raw or pulling many copies.

After the transport fixes a residual flap remained: RViz's Depth3D
status cycled OK/error every couple of seconds, because the unpaced
depth pipeline (~10 Hz in daylight) outran rgbd_odometry (~4–5 Hz), so
the odom TF stamps trailed the clouds by ~0.8 s median while rgbd
chewed queue backlog — and the display's `Depth: 1` gave each cloud
only until the next one arrived to find its transform. Two-part fix
(2026-08-16): the depth estimator's `max_rate` paces the whole
pipeline at 5 Hz (what rgbd sustains — TF stays current, GPU does half
the work), and the display queues 10 clouds. Measured: per-cloud TF
wait went from p50 813 ms with 30% dropped to p50 15 ms / max 551 ms
with none dropped. Shrinking rgbd's sync queues to bound the lag was
tried first and made it *worse* — under bursty processing the two
topics drop different stamps and exact sync starves; pace the source,
don't starve the sync.

Even paced, the flap returned once RViz itself was running — its own
load pushed rgbd's `delay=` to 1.6–2.6 s (read from the session's node
logs in `~/.ros/log/`), and the display's wait budget is only ~2 s. The
structural truth: RViz was waiting for a transform that is *computed
after* the data it poses, on a camera whose stamps are faulted — a race
that tuning can only narrow, never win. The definitive fix (2026-08-16)
moves the transform into the data path: `cloud_projector`'s
`output_frame` parameter (world_mesh sets `odom`) makes the projector
pose the cloud in the world itself using the **latest** TF — the same
latest-only rule as `cloud_mapper` and `tsdf_mesher`, adopted for the
same reason. A cloud whose `frame_id` *is* the fixed frame needs no
dynamic transform, so Depth3D can no longer error after startup.
`piros2_world` leaves the parameter unset and keeps the original
viewer-transforms behaviour for its mapper.

## RViz Marker: "Could not load resource … GLTF: Buffer view … out of range"

Symptom: a `mesh_resource` Marker pointing at a `.glb` written by
Open3D errors in the RViz log and renders nothing; the same mesh as
`.ply` loads cleanly.

RViz's assimp cannot parse Open3D's GLB layout (measured 2026-08-11).
Point `mesh_resource` at the PLY; keep `.glb` for external viewers
(Blender, web viewers). Related trap, and the reason the *live* mesh is
published as a TRIANGLE_LIST Marker instead of a file at all: RViz
caches `mesh_resource` by URI, so rewriting a file and re-publishing
the same path shows the stale mesh forever, and unique-name-per-refresh
grows RViz memory without bound.

## Open3D viewer window fails: "Failed to initialize GLEW"

Symptom: `o3d.visualization.draw_geometries` prints a GLFW Wayland
warning, "Failed to initialize GLEW", "Failed creating OpenGL window",
and returns without a window.

Same family as the rviz2/Qt entries: the dev-box session is Wayland and
this GL path needs X11. For GLFW (Open3D bundles it) unsetting
`WAYLAND_DISPLAY` is *not enough* — it still picks the Wayland backend
— it additionally needs `XDG_SESSION_TYPE=x11` to fall back to
Xwayland. `just view-mesh` carries both pins.

## `VoxelBlockGrid.integrate`: "Unsupported input data type combination"

Symptom: `Expected (float, float) or (uint16, uint8), but received
(Float32 UInt8)` — integration works from PNG files but crashes on
live images.

Open3D pairs depth/colour dtypes strictly. File-based fusion reads
uint16 depth + uint8 colour and matches; a live float32-metres depth
image forces the colour to float32 in [0, 1] (`tsdf_mesher` converts).

## TSDF surfaces drift away when depth alignment is fed back

Symptom: with per-frame depth-to-map scale alignment enabled, a wall
at a fixed distance walks outward (~+1%/frame) and fused surfaces get
*thicker*, not thinner.

`VoxelBlockGrid.ray_cast` reads the surface ~1.25 voxels behind where
`integrate` put it (measured 2026-08-11: +1.0% at 2 cm voxels on a
2.5 m scene, +0.5% at 1 cm — voxel-proportional, truncation-
independent, and it shifts as the map accumulates, so a one-shot bias
calibration fails too). Any constant error compounds in a
conform-to-map loop. The stable design is the high-pass in
`piros2_world/depth_align.py`: correct only each frame's *deviation*
from a rolling median of ratios — wobble is fast, bias is slow, and the
correction stream has median 1 by construction, so it cannot push the
map anywhere.

## The Pi vanishes from the network after replugging the camera

Symptom: `ssh pi` → `No route to host`, ARP `FAILED`; moments earlier
only the camera USB was touched.

The Pi 5's USB-C power sits next to the USB-A ports and unseats
easily — the board reboots (check `uptime -s` once it returns, ~60 s).
Knock-on effects of the power cycle: the C922 reverts to
`exposure_dynamic_framerate=1` and `gain=0` territory again, so run
`just camera-reset` (and re-raise gain in a dim room) before trusting
any frames — see camera.md's persistent-state rules.

## The Pi is unreachable but the camera LED is still on

Symptom: `ping 192.168.2.17` gets nothing, `ssh pi` times out, yet the
C922's LED is lit — and possibly has been for hours.

The Wi-Fi link died while the OS kept running. This is a different
fault from the power-unseat entry above: there the board reboots
(`uptime -s` resets, ARP goes `FAILED`); here uptime is long, the
journal never stops, and the orphaned camera session keeps streaming
to nobody — the LED is the proof the Pi is alive, not a sign it is
healthy. Seen twice (2026-08-11/12); the full incident record and
triage order live in
[networking.md#wi-fi-link-reliability](networking.md#wi-fi-link-reliability).

Since 2026-08-12 the watchdog (Ansible `wifi` role) repairs this
unaided — reassociate → driver reload → guarded reboot, drilled at
T+426 s to recovery — so first read its flight recorder:
`journalctl -t wifi-watchdog`. A Pi that is *still* unreachable after
~10 minutes means the ladder is losing: power cycle it and read the
journal for which rungs fired. Orphaned camera sessions are reaped
automatically since the same date (sshd ClientAlive + the recipes'
`ssh -tt`); if `just stragglers` still shows one, clear it before the
next launch — a held device dies with the next entry's `char*` abort.

## usb_cam dies at startup: `terminate called after throwing an instance of 'char*'`

Symptom: the camera launch on the Pi prints the device's full format
list, then usb_cam aborts (exit code −6) and the launch shuts down.

That `char*` throw is usb_cam failing to negotiate the video format,
and the two observed causes are both *state*, not code: the device is
already held by an orphaned usb_cam from an earlier session (check
`just stragglers`, or `fuser /dev/video0` on the Pi), or the camera is
in a transient bad state shortly after a Pi reboot (seen 2026-08-11;
the identical launch succeeded on retry minutes later). Clear any
holder, retry once, and only then treat it as a real bug.

Since 2026-08-16 the held-device case fails *before* usb_cam starts:
`camera.launch.py`'s pre-flight scans `/proc/*/fd` for the device and
aborts naming the holding PID and command line (a leaked session held
the camera for 37 minutes on 2026-08-15 while a second session ran
unknowingly on the leak's frames; the abort above was the only clue).
If a launch reports `camera is already in use`, believe it — run
`just stragglers`.

## `xwd -root` fails with `BadMatch`; a Qt window can still be dumped

Symptom: trying to screenshot the desktop for a check gives
`X Error of failed request: BadMatch (X_GetImage)` and an empty file.

The dev-box session is Wayland; the X server is rootless Xwayland, whose
root window has no backing pixmap. Individual X windows do — rviz2 and
rqt (the `QT_QPA_PLATFORM=xcb` pin) and Open3D's viewer
(`XDG_SESSION_TYPE=x11`) all are — so dump them by id:
`xwininfo -root -tree` for the id, `xwd -id <id> | ffmpeg -i - out.png`
(no ImageMagick here; ffmpeg decodes XWD). `just snap` does exactly this
for every rviz/rqt/`.ply` window and writes the topics alongside —
[verification.md](verification.md). A minimised or covered window dumps
whatever the server holds, possibly stale.

## A shell dies with exit code 144 the moment a session recipe ends

Symptom: a command that ran `just world_mesh` / `just gate` / `just
run-bag` and then something else never reaches the something else;
the shell reports exit code 144 (killed — the number is what the
harness prints for a signalled shell, not a bash status).

The recipe's EXIT trap runs `pkill -f` on node patterns such as
`piros2_world_mesh/[k]eypoint_detector`; if the *calling* shell's own
command line contains that text — a source path like
`src/piros2_world_mesh/piros2_world_mesh/keypoint_detector.py`, or a
`pkill -f rqt_graph` alongside a recipe that mentions rqt — the trap
matches the shell and kills it. Keep node paths and pkill patterns off
the command line that runs a session recipe (bracket-escape them:
`[k]eypoint_detector`), or run them as separate commands.

## rviz2 survives the first `pkill`

Symptom: `pkill -f "[r]viz2 -d …"` reports the kill (with `-e`), the
process is still there seconds later; a second `pkill` ends it.

Observed 2026-08-18 while scripting session closes; the cause was not
chased (rviz2's own SIGTERM handling appears to swallow the first
signal while the render thread is busy). Session recipes are unaffected
— they end when the window closes, and their traps kill nodes, not
RViz. A scripted close should send the signal, pause, send it again if
`pgrep` still finds it, and confirm with `just stragglers`.

## Loop closures disagree with each other by degrees; keyframe landmarks look misplaced

**Symptom.** In rgbd mode the detector's loop closures against the same
keyframe report drifts that differ by 0.2–0.5 m / tens of degrees
within seconds of each other, with only 20–55 PnP/rigid-fit inliers.

**Cause.** Landmark geometry was built from the *latest* `/depth` and
the *latest* odom TF at the moment the RGB frame was processed — the
relocalization plan's accepted latest-only rule. Depth lands ~80 ms and
the odometry TF ~200 ms after the frame; at hand-pan speed (30°/s)
that poses every stored keyframe a few degrees wrong, and two wrong
keyframes disagree.

**Fix (2026-08-18).** rgbd geometry runs on exact triples: the frame's
own ORB output, its own `/depth` (same stamp), and TF looked up *at
that stamp* — a small depth queue drained by a 10 Hz timer once the TF
exists (`sync_min_delay_s`). Same bag afterwards: closures agree to
±0.4° with 105–360 inliers. The latest-only rule still holds for the
mapper/mesher/projector — they pose *now*; the store poses the *past*.

## RTAB-Map's optimised poses "equal" its odometry after a loop closure

**Symptom.** `rtabmap-report --poses_raw` gives `_odom.txt` and
`_slam.txt`; lifting the per-node correction `slam ∘ odom⁻¹` onto the
recorded `/tf` odometry shows no improvement even though `map → odom`
clearly moved.

**Cause.** The DB's odometry column is re-based whenever `rgbd_odometry`
auto-resets ("Odometry automatically reset to latest computed pose" —
RTAB-Map starts a new map session), so it is not the TF you recorded;
after a reset the two differ by a constant transform.

**Fix.** Derive the correction from the recorded TF at the node stamp
— `optimised ∘ tf_odom(t_k)⁻¹` — never from the DB odometry
(`traj_check.py dense_from_graph`). Also: `/mapPath` poses share one
stamp, so read the graph from the DB after stopping the node.

## The live mesh stops growing after the first refresh

**Symptom.** `tsdf_mesher` logs `N frames integrated` and the count
barely moves after ~25 s; the surface never shows the second half of a
bag; `refresh: … in 13000 ms`.

**Cause.** At 1.5 cm voxels a close-range scene meshes to 0.7–1.6 M
triangles; the completion pass (~6.5 s) plus quadric decimation to the
120k marker budget (~5.6 s) ran inline on the single executor thread
every 15 s, and the synced depth pairs (KEEP_LAST 1) were dropped
while it ran.

**Fix (2026-08-18/19).** Extraction stays on the executor thread; the
rest (decimate first, then complete the budgeted mesh — ~1 s instead of
6.5 — then build the Marker) runs in a **separate process**
(`mesh_worker.py`, spawn context, arrays over a pipe, polled by a
0.5 s timer). A worker *thread* was tried first and did not help:
Open3D's `simplify_quadric_decimation` holds the GIL, so the executor
still sat blocked and zero frames integrated for 20 s at a time —
measured as a loop bag's frame memory of 90 outbound / 13 return
frames. With the process: 327 frames over the same 88 s bag at 41–44
ms/frame, the refresh 22–32 s in the worker at 1.4–2.6 M triangles (so
the marker updates every ~30 s at this density — a refresh is skipped
while the previous one is finishing). The saved PLY still completes at
full detail; its Poisson-closed companion is minutes at that size —
`mesh_watertight:=false` for headless runs.

