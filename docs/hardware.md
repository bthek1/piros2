# Hardware & environment

Everything on this page was measured on the actual machines on 2026-07-23.
Re-run the listed commands if you suspect drift.

## Raspberry Pi 5 — the robot side

| Property | Value |
| --- | --- |
| Model | Raspberry Pi 5 Model B Rev 1.0 |
| Hostname | `raspberrypi` |
| SSH alias | `pi` (see below) |
| IP | `192.168.2.17` |
| OS | Raspberry Pi OS 64-bit — Debian GNU/Linux 12 (bookworm) |
| Architecture | `aarch64` |
| CPU | 4 cores |
| RAM | 7.9 GiB |
| Python | 3.11.2 |
| ROS installed | **No** |
| Docker installed | **No** |

```bash
ssh pi 'cat /proc/device-tree/model; cat /etc/os-release; nproc; free -h'
```

### SSH access

`~/.ssh/config` on the dev box defines:

```
Host pi
    HostName 192.168.2.17
    User bthek1
    Port 22
```

Key-based auth is set up — `ssh pi` needs no password, and `BatchMode=yes` works,
so scripts can drive the Pi non-interactively.

> **If SSH suddenly refuses with `REMOTE HOST IDENTIFICATION HAS CHANGED`:** the Pi's
> host keys were regenerated (a reflash does this). Clear the stale entry with
> `ssh-keygen -f ~/.ssh/known_hosts -R 192.168.2.17`, then confirm the new
> fingerprint from the Pi's own console before trusting it:
> `ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub`.

### Group membership

The login user is in `video`, `plugdev`, `gpio`, `i2c`, `spi`, `dialout`, and
`render`. Practically: **`/dev/video0` is readable without `sudo`**, and GPIO/I²C/SPI
are available for later milestones without permission work.

## Dev box — the visualisation side

| Property | Value |
| --- | --- |
| Hostname | `test` |
| IP | `192.168.2.106/24` on interface `eth2` |
| OS | Ubuntu 24.04.4 LTS |
| Architecture | `x86_64` |
| Docker | 28.5.1 |
| Python | 3.12.3 |
| ROS installed | **No** |

Ubuntu 24.04 (noble) is the **Tier 1 platform for ROS 2 Jazzy**, so this machine
can install ROS natively from `apt`. That is what makes it the right place to run
`rviz2` and `rqt`.

> This host also has ~11 Docker bridge networks (`172.17.0.0/16` … `172.26.0.0/16`).
> They are harmless for normal work but they actively interfere with DDS discovery —
> see [networking.md](networking.md).

## Camera — Logitech C922 Pro Stream

| Property | Value |
| --- | --- |
| USB ID | `046d:085c` |
| Bus | `usb-xhci-hcd.1-1` (Bus 003 Device 002) |
| Capture node | `/dev/video0` |
| Metadata node | `/dev/video1` |
| Media node | `/dev/media3` |

`/dev/video0` is the one that produces frames. `/dev/video1` is the UVC metadata
node and will **not** work as a capture device — pointing a driver at it is a
common early mistake.

```bash
ssh pi 'v4l2-ctl --list-devices; v4l2-ctl -d /dev/video0 --list-formats-ext'
```

### Capture modes

Two pixel formats are offered:

**`MJPG` (Motion-JPEG, compressed)** — the useful one. The camera compresses
on-board, so USB bandwidth stops being the limit:

| Resolution | Max FPS |
| --- | --- |
| 1920×1080 | 30 |
| 1280×720 | **60** |
| 640×480 | 30 |

**`YUYV` (raw 4:2:2)** — uncompressed. Offers larger frames (up to 2304×1536) but
USB 2.0 bandwidth caps the frame rate hard; 640×480 tops out at 30 fps and the
high resolutions are single-digit fps.

Full ladder of sizes for both formats: 160×90, 160×120, 176×144, 320×180,
320×240, 352×288, 432×240, 640×360, 640×480, 800×448, 800×600, 864×480, 960×720,
1024×576, 1280×720, 1600×896, 1920×1080 (plus 2304×1296 and 2304×1536 for YUYV only).

**Default recommendation: MJPG 1280×720 @ 30 fps.** It leaves CPU headroom on the
Pi, decodes quickly, and is a sane resolution for vision experiments. Move to
60 fps only if you actually need the temporal resolution, and see
[camera.md](camera.md) for the decode cost that MJPG implies.

### Other notes

The `pispbe` and `rpi-hevc-dec` entries in `v4l2-ctl --list-devices`
(`/dev/video19`–`/dev/video35`) are the Pi 5's built-in ISP and HEVC decoder
blocks, not cameras. Ignore them.

There is no CSI ribbon camera attached — only the USB webcam. This means
`libcamera`/`rpicam-apps`, the usual Raspberry Pi camera path, is **not** the
route to use here; standard V4L2 is.
