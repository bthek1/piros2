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

## Camera opens but every frame is black or green

- **Leftover manual exposure — check this first.** V4L2 controls live *in the
  camera* and persist across processes, node restarts and reboots (until
  unplug), so a manual exposure set for a frame-rate benchmark silently
  applies to every later session. Hit 2026-07-24: `auto_exposure=1` +
  `exposure_time_absolute=136` + `gain=0` from earlier measurements produced
  pure black in an evening room. Diagnose and restore:
  ```bash
  v4l2-ctl -d /dev/video0 --get-ctrl=auto_exposure,exposure_time_absolute,gain
  v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=3    # aperture priority, the camera default
  ```
- Another process already has the device. `ssh pi 'sudo fuser -v /dev/video0'`.
  UVC allows only one capture client at a time.
- The driver is pointed at `/dev/video1`, which is the UVC **metadata** node, not a
  capture device.
- Exposure was manually set far too low — see
  [camera.md](camera.md#v4l2-controls).

## Frame rate far below what was requested

- **Auto-exposure is stealing frames — check this first.** The C922 ships with
  `exposure_dynamic_framerate=1`, letting it lengthen exposure past the frame
  interval in dim light. Measured on this camera: 18–21 fps at 720p on defaults
  versus 30.00 fps with a fixed exposure. Confirm and fix:
  ```bash
  v4l2-ctl -d /dev/video0 --list-ctrls | grep -E 'auto_exposure|dynamic_framerate'
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
   The `just cloud` recipe carries all three variables; drop the two mesa
   ones after a reboot (keep `QT_QPA_PLATFORM=xcb` — layer 1 is permanent).

## Orphaned nodes keep logging into a terminal after a recipe ends

Symptom: `edge_detector` or `republish` output appears in a terminal that is
not running anything, whenever the camera streams. Cause: a recipe's cleanup
trap used `kill %N` — the background jobs are `bash -lc` wrappers, and killing
the wrapper orphans the actual ros2-run grandchildren. The orphans sit silent
(nothing publishes to their topic) until a live publisher comes back, then
haunt the terminal they were started from. Hit twice on 2026-07-27: a
republisher and an edge detector leaked from a `just replay`.

The `calibrate` and `replay` traps now `pkill -f` the node patterns instead.
Diagnose and clear survivors with:

```bash
pgrep -af 'republish|edge_detector|cameracalibrator'
```
