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
4. **Wrong interface picked on the dev box.** Eleven Docker bridges compete with
   `eth2`; DDS may bind to `172.2x.0.1`. Pin the interface —
   [networking.md](networking.md#the-docker-bridge-problem-on-the-dev-box).
5. **RMW mismatch.** `echo $RMW_IMPLEMENTATION` on both. Fast DDS and Cyclone DDS
   do not interoperate for discovery in practice.
6. **`ROS_LOCALHOST_ONLY=1`** left set somewhere.

Confirm packets are actually crossing:

```bash
sudo tcpdump -i eth2 -n 'udp and portrange 7400-7500'
```

## Topics are visible but `ros2 topic echo` prints nothing

Discovery succeeded, data transport did not.

- **QoS incompatibility** — the usual cause. Sensor publishers typically use
  `BEST_EFFORT` reliability while `ros2 topic echo` defaults to `RELIABLE`, and
  incompatible pairs simply never connect. Force it:
  `ros2 topic echo /image_raw --qos-reliability best_effort`.
  `ros2 topic info /image_raw --verbose` shows both ends' QoS profiles.
- **MTU / fragmentation** — large messages (raw images) fragmented across UDP can
  be dropped wholesale by a switch. Subscribe to the compressed topic instead.

## `/dev/video0` not found, or permission denied

- **Permission denied** means the login user is not in the `video` group. This is
  the expected state on a freshly reflashed Ubuntu — Raspberry Pi OS put the user
  there, Ubuntu does not. Fix it via the Ansible `camera` role, or by hand with
  `sudo usermod -aG video $USER`, then **log out and back in**; a group change does
  not affect the session that made it.
  ```bash
  ssh pi 'id -nG | tr " " "\n" | grep -x video'   # silence = not in the group
  ```
- Did the camera enumerate at all? `ssh pi 'ls -l /dev/video0'`
- Was the device re-enumerated after a replug? The node numbers can shift.
  `ssh pi 'v4l2-ctl --list-devices'` and look for the `C922 Pro Stream Webcam`
  block. For a stable path, use the `/dev/v4l/by-id/` symlink instead of `/dev/video0`.

## Camera opens but every frame is black or green

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
  v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1 --set-ctrl=exposure_time_absolute=150
  ```
  A frame rate that *drifts downward* over the first few seconds rather than
  sitting at a wrong-but-stable value is this, essentially every time.
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
cd ~/piros2
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
