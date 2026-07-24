# Hardware & environment

Everything on this page was measured on the actual machines on 2026-07-23.
Re-run the listed commands if you suspect drift.

## Raspberry Pi 5 — the robot side

> **Reflashed to Ubuntu on 2026-07-23.** The machine now runs Ubuntu Server 24.04
> LTS from the SD card, which is what makes the native `apt` ROS install possible.
> Raspberry Pi OS is gone — how it was done, and the traps encountered, are in
> [setup.md](setup.md).

| Property | Value (measured 2026-07-23) |
| --- | --- |
| Model | Raspberry Pi 5 Model B Rev 1.0 |
| Hostname | `raspberrypi` |
| SSH alias | `pi` (see below) |
| IP | `192.168.2.17` — **on Wi-Fi, not Ethernet** |
| OS | Ubuntu 24.04.4 LTS (noble) |
| Kernel | `6.8.0-1047-raspi` |
| Architecture | `aarch64` |
| CPU | 4 cores |
| RAM | 7.8 GiB |
| Python | 3.12.3 |
| Root filesystem | `/dev/mmcblk0p2`, ext4, 234 G (222 G free) |
| Boot partition | `/dev/mmcblk0p1`, vfat, mounted at `/boot/firmware` |
| ROS installed | **No** — but `ros-jazzy-*` is now installable from apt |
| Docker installed | **No** — the container path was dropped |

```bash
ssh pi 'cat /proc/device-tree/model; cat /etc/os-release; nproc; free -h; findmnt /'
```

Ubuntu 24.04 noble is Jazzy's **Tier 1** platform, so `packages.ros.org` serves
**3361** `ros-jazzy-*` packages for `noble/arm64` where `bookworm/arm64` served
**0**. Both machines now share one install path — the reasoning, and the rejected
alternatives, are in [setup.md](setup.md).

### Networking — Wi-Fi only

`eth0` has **no cable attached** (`NO-CARRIER`); the Pi reaches the LAN over
`wlan0` on the `THEKKEL_MESH` SSID and holds the same `192.168.2.17` lease it had
under Raspberry Pi OS.

```bash
ssh pi 'ip -br addr; cat /sys/class/net/eth0/carrier'
```

This matters more than it sounds. Ubuntu Server's preinstalled image does **not**
join Wi-Fi on its own — the credentials have to be written to `network-config` on
the boot partition before first boot, or the machine comes back deaf and needs a
keyboard and monitor. See [setup.md](setup.md#2-configure-before-first-boot).

### Boot configuration

The Pi boots from the SD card. The bootloader EEPROM settings survive a reflash
because they live in the Pi's own SPI flash, not on any card:

| Setting | Value | Why |
| --- | --- | --- |
| `BOOT_ORDER` | `0xf41` | SD first, then USB, then restart. Nibbles are read **right to left**. |
| `USB_MSD_PWR_OFF_TIME` | `3000` | Needed to boot from USB at all — see [setup.md](setup.md#booting-from-usb). |
| `USB_MSD_DISCOVER_TIMEOUT` | `30000` | Widens the USB discovery window. |
| Bootloader build | **2024-09-23** | Ubuntu's bundled version — see the warning below. |

> **Ubuntu downgrades the bootloader.** Ubuntu's `rpi-eeprom` package (26.3) ships
> only `pieeprom-2024-09-23.bin` and enables `rpi-eeprom-update.service`. Any
> firmware update applied from Raspberry Pi OS is reverted the moment
> `rpi-eeprom-config --apply` runs under Ubuntu, which rewrites the EEPROM using
> whatever image Ubuntu bundles. Config keys such as `BOOT_ORDER` are preserved;
> the firmware version is not. Verify the *running* version from the device tree
> rather than trusting `rpi-eeprom-update`, which reports Ubuntu's bundle as both
> CURRENT and LATEST:
>
> ```bash
> ssh pi 'od -An -tu4 --endian=big /proc/device-tree/chosen/bootloader/build-timestamp'
> ```

The SD card's partitions are pinned by PARTUUID rather than by label — disk
identifier `0x5ec0ffee`, giving `5ec0ffee-01` and `5ec0ffee-02`. Both
`cmdline.txt` and `/etc/fstab` reference those UUIDs. The reason is in
[setup.md](setup.md#the-duplicate-label-trap): Ubuntu's Pi image labels its
partitions `system-boot` and `writable`, so any second copy of the image collides
and the kernel picks whichever it enumerates first.

### SSH access

`~/.ssh/config` on the dev box defines:

```
Host pi
    HostName 192.168.2.17
    User bthek1
    Port 22
```

Key-based auth is set up — `ssh pi` needs no password, and `BatchMode=yes` works,
so scripts and [Ansible](ansible.md) can drive the Pi non-interactively. This
survived the reflash because the username and the dev box's public key were written
into `user-data` on the boot partition before first boot; get that wrong and the
machine comes back unreachable and needs a keyboard and monitor to recover.

> **If SSH refuses with `REMOTE HOST IDENTIFICATION HAS CHANGED`:** the Pi's host
> keys were regenerated. The reflash did this, so it is expected — clear the stale
> entry with `ssh-keygen -f ~/.ssh/known_hosts -R 192.168.2.17`, then confirm the
> new fingerprint from the Pi's own console before trusting it:
> `ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub`.

There is also a **console fallback password** for `bthek1`, set from `user-data`.
On a Wi-Fi-only machine, key-only auth means a Wi-Fi failure locks you out
entirely, so a password that works on HDMI + keyboard is worth having.

### Group membership

The login user is in `adm`, `dialout`, `sudo`, `video`, `plugdev`, and `render`.
Practically: **`/dev/video0` is readable without `sudo`** — confirmed by reading the
node as `bthek1`.

```bash
ssh pi 'id -nG'
```

**`gpio`, `i2c`, and `spi` no longer exist.** They were a Raspberry Pi OS vendor
addition; Ubuntu does not create them, and nothing on the system does now:

```bash
ssh pi 'getent group | cut -d: -f1 | grep -xE "gpio|i2c|spi"'   # silence
```

That is fine for every camera milestone, which only needs `video`. GPIO access
waits until [roadmap.md](roadmap.md) step 7 actually requires it — and will mean
creating the group and a udev rule, not just adding the user to something that
already exists.

> **Do not name a non-existent group in cloud-init.** Listing `i2c`/`spi`/`gpio`
> under a user's `groups:` makes `useradd` fail, the account is never created, and
> a headless machine comes up with no way in. Declaring them under the top-level
> `groups:` key first is the safe form.

## Dev box — the visualisation side

| Property | Value (measured 2026-07-23) |
| --- | --- |
| Hostname | `proxmox-ml5` |
| SSH alias | `proxmox_main` |
| IP | `192.168.2.109/24` on interface `enp6s18` |
| OS | Ubuntu 24.04.4 LTS |
| Architecture | `x86_64` |
| CPU / RAM | 16 cores / 19 GiB |
| Disk free | 81 G |
| Display | **GNOME Shell + Xwayland on `seat0`** |
| Ansible | core 2.16.3 (`ansible.posix` 1.5.4, `community.general` 8.3.0) |
| Python | 3.12.3 |
| `sudo` | **prompts for a password** — playbook runs need `--ask-become-pass` |
| ROS installed | **No** |

> **This role moved on 2026-07-23.** The dev box was previously `test`
> (`192.168.2.106`), an LXC container. It has **no display** — `XDG_SESSION_TYPE=tty`,
> no X server — so `rviz2` and `rqt_image_view` could never have run there without
> X-forwarding gymnastics, which is precisely what the two-machine split exists to
> avoid. `ml5` is the only machine with a real desktop session, so it took over.
> `test` is no longer part of the project; ignore references to it in git history.

### Prerequisites before `ml5` can drive the Pi

Two things are **not** yet in place, and both will stop Ansible dead:

- **The Pi does not trust `ml5`'s SSH key.** The Pi's `authorized_keys` was written
  by cloud-init at reflash time and contains only the old dev box's key. Add
  `ml5`'s `~/.ssh/id_ed25519.pub` to `bthek1@192.168.2.17:~/.ssh/authorized_keys`,
  or every play fails at the connection stage.
- **`ml5` has a stale host key for the Pi.** The Pi's host keys were regenerated by
  the reflash, so `ssh 192.168.2.17` from `ml5` fails strict checking. Clear it:
  `ssh-keygen -f ~/.ssh/known_hosts -R 192.168.2.17`.

```bash
# on ml5, to verify both are fixed
ssh -o BatchMode=yes bthek1@192.168.2.17 'hostname'
```

Ubuntu 24.04 (noble) is the **Tier 1 platform for ROS 2 Jazzy**, so this machine
can install ROS natively from `apt`. That is what makes it the right place to run
`rviz2` and `rqt` — and now that the Pi has been reflashed, both machines share one
install path and one set of [Ansible](ansible.md) roles.

Ansible is already installed here, so this host is the control node and needs no
bootstrapping.

> This host carries several interfaces DDS must not bind to: three Docker bridges
> (`172.17.0.0/16` … `172.19.0.0/16`), `tailscale0`, and a WireGuard interface
> named `laptop` at `10.8.0.3`. They are harmless for normal work but they actively
> interfere with DDS discovery — see [networking.md](networking.md). Fewer than the
> eleven bridges the old dev box had, but the fix is the same: pin `enp6s18`
> explicitly rather than hoping the right one wins.

## Camera — Logitech C922 Pro Stream

| Property | Value (re-measured 2026-07-23, post-reflash) |
| --- | --- |
| USB ID | `046d:085c` |
| Bus | Bus 004 Device 003 |
| `ID_PATH` | `platform-xhci-hcd.1-usb-0:1:1.0` |
| Capture node | `/dev/video0` |
| Metadata node | `/dev/video1` |
| Stable path | `/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5461327F-video-index0` |

`/dev/video0` is the one that produces frames. `/dev/video1` is the UVC metadata
node and will **not** work as a capture device — pointing a driver at it is a
common early mistake.

**Prefer the `by-id` path in configs.** It encodes the camera's serial, so it
survives replugging and node renumbering; `/dev/video0` does not.

The camera came through the reflash unchanged, as expected: it is a standard UVC
device driven by the in-tree `uvcvideo` module, which Ubuntu's Pi kernel carries
just as the vendor kernel did. It re-enumerated as `/dev/video0` + `/dev/video1`
under Ubuntu, same as before.

> **`v4l2-ctl` comes from the Ansible `camera` role** (installed 2026-07-24) —
> Ubuntu Server does not ship `v4l-utils`, so after a reflash the `v4l2-ctl`
> commands in these docs fail until `ansible-playbook site.yml --limit robot`
> has run.

```bash
ssh pi 'v4l2-ctl --list-devices; v4l2-ctl -d /dev/video0 --list-formats-ext'
```

### Capture modes

Two pixel formats are offered:

**`MJPG` (Motion-JPEG, compressed)** — the useful one. The camera compresses
on-board, so USB bandwidth stops being the limit:

| Resolution | Advertised | **Measured** (2026-07-23) |
| --- | --- | --- |
| 1920×1080 | 30 | **30.00** |
| 1280×720 | 60 or 30 | **30.00** at 30; only **29.7** when 60 is requested |
| 640×480 | 30 | **30.00** |

Measured with a fixed manual exposure — see [the frame-rate note](#frame-rate-and-auto-exposure)
below, which matters more in practice than any of these numbers.

**720p60 does not actually deliver 60 fps.** The mode is advertised, and
`v4l2-ctl --set-parm=60` reports "Frame rate set to 60.000 fps" with `--get-parm`
confirming `60/1` — but sustained capture measures ~29.7 fps. The cause has not
been isolated. The camera enumerates on a USB 2.0 bus (`Bus 03`, 480 Mbit/s) while
the Pi 5's 5 Gbit/s bus (`Bus 04`) has a free port, so moving it to a blue USB 3
port is the obvious thing to try — though the C922 is itself a UVC 1.00 USB 2.0
device, so this may change nothing. Treat 30 fps as the working ceiling at every
resolution until measured otherwise.

**`YUYV` (raw 4:2:2)** — uncompressed. Offers larger frames (up to 2304×1536) but
USB 2.0 bandwidth caps the frame rate hard; 640×480 tops out at 30 fps and the
high resolutions are single-digit fps.

Full ladder of sizes for both formats: 160×90, 160×120, 176×144, 320×180,
320×240, 352×288, 432×240, 640×360, 640×480, 800×448, 800×600, 864×480, 960×720,
1024×576, 1280×720, 1600×896, 1920×1080 (plus 2304×1296 and 2304×1536 for YUYV only).

**Default recommendation: MJPG 1280×720 @ 30 fps.** It leaves CPU headroom on the
Pi, decodes quickly, and is a sane resolution for vision experiments. See
[camera.md](camera.md) for the decode cost that MJPG implies.

### Frame rate and auto-exposure

The camera does **not** hold 30 fps out of the box. In a normally lit indoor room
it settles at **18–21 fps** at 720p, and the rate drifts downward over the first
few seconds rather than dropping cleanly.

This is not a bug and not a bandwidth problem. The C922 defaults to
`auto_exposure = 3` (Aperture Priority) with `exposure_dynamic_framerate = 1`,
which permits the camera to lengthen exposure past the frame interval in dim
light — trading frame rate for brightness. Measured at 720p MJPG:

| Settings | FPS |
| --- | --- |
| Stock defaults | 18–21 |
| `exposure_dynamic_framerate=0` | 23.9 |
| `auto_exposure=1` (manual), `exposure_time_absolute=150` | **30.00** |

Anything time-sensitive — frame differencing, tracking, visual odometry, or
measuring a pipeline's real throughput — needs a fixed exposure, otherwise the
frame rate silently follows the room lighting. The commands are in
[camera.md](camera.md#v4l2-controls).

The `usb_cam` node sets `auto_exposure` and friends as ROS parameters, so this
belongs in `config/camera.yaml` rather than a manual `v4l2-ctl` step.

### Other notes

The `pispbe` and `rpi-hevc-dec` entries are the Pi 5's built-in ISP and HEVC
decoder blocks, not cameras. Ignore them. Under Ubuntu they occupy
**`/dev/video19`–`/dev/video37`** (they were `/dev/video19`–`/dev/video35` on the
vendor kernel), alongside `/dev/media0`–`/dev/media3`. The numbering shifting
across a kernel change is exactly why the camera should be identified by its
`/dev/v4l/by-id/` symlink rather than by node number.

With the camera unplugged, `/dev/video0` and `/dev/video1` simply vanish and only
the ISP/decoder nodes remain — a quick way to tell "camera not attached" from
"camera broken".

There is no CSI ribbon camera attached — only the USB webcam. This means
`libcamera`/`rpicam-apps`, the usual Raspberry Pi camera path, is **not** the
route to use here; standard V4L2 is. Losing that stack in the reflash duly cost
nothing.
