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
- **MTU / fragmentation** — large messages (raw images) fragmented across UDP can
  be dropped wholesale by a switch. Subscribe to the compressed topic instead.

## `/dev/video0` not found, or permission denied

- **`v4l2-ctl: command not found` — check this first.** Ubuntu Server does not ship
  `v4l-utils`; Raspberry Pi OS did. `ssh pi 'sudo apt install -y v4l-utils'`.
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
