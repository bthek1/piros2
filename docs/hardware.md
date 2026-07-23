# Hardware & environment

Everything on this page was measured on the actual machines on 2026-07-23.
Re-run the listed commands if you suspect drift.

## Raspberry Pi 5 — the robot side

> **The OS on this machine is scheduled to change.** The project has settled on a
> native ROS 2 install, which requires Ubuntu 24.04 — see [setup-pi.md](setup-pi.md).
> The middle column is what is on the card **now**; the reflash has not been done.

| Property | Value (measured) | After the planned reflash |
| --- | --- | --- |
| Model | Raspberry Pi 5 Model B Rev 1.0 | unchanged |
| Hostname | `raspberrypi` | unchanged |
| SSH alias | `pi` (see below) | unchanged |
| IP | `192.168.2.17` | unchanged |
| OS | Raspberry Pi OS 64-bit — Debian GNU/Linux 12 (bookworm) | **Ubuntu Server 24.04 LTS** |
| Kernel | `6.12.87+rpt-rpi-2712` (Raspberry Pi vendor kernel) | Ubuntu's Pi kernel |
| Architecture | `aarch64` | unchanged |
| CPU | 4 cores | unchanged |
| RAM | 7.9 GiB | unchanged |
| Python | 3.11.2 | 3.12.x |
| ROS installed | **No** | `ros-jazzy-ros-base` via apt |
| Docker installed | **No** | not planned — the container path was dropped |

```bash
ssh pi 'cat /proc/device-tree/model; cat /etc/os-release; nproc; free -h'
```

Why the reflash: `packages.ros.org` serves **0** `ros-jazzy-*` packages for
`bookworm/arm64` and **3361** for `noble/arm64`. The full check, the alternatives,
and what the change costs are in [setup-pi.md](setup-pi.md).

### SSH access

`~/.ssh/config` on the dev box defines:

```
Host pi
    HostName 192.168.2.17
    User bthek1
    Port 22
```

Key-based auth is set up — `ssh pi` needs no password, and `BatchMode=yes` works,
so scripts and [Ansible](ansible.md) can drive the Pi non-interactively. Preserve
this through the reflash by supplying the same username and the dev box's public
key in Raspberry Pi Imager, or the machine comes back unreachable and needs a
keyboard and monitor to recover.

> **If SSH suddenly refuses with `REMOTE HOST IDENTIFICATION HAS CHANGED`:** the Pi's
> host keys were regenerated (a reflash does this — so expect it once, deliberately).
> Clear the stale entry with
> `ssh-keygen -f ~/.ssh/known_hosts -R 192.168.2.17`, then confirm the new
> fingerprint from the Pi's own console before trusting it:
> `ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub`.

### Group membership

The login user is in `video`, `plugdev`, `gpio`, `i2c`, `spi`, `dialout`, and
`render`. Practically: **`/dev/video0` is readable without `sudo`**, and GPIO/I²C/SPI
are available for later milestones without permission work.

**This does not survive the reflash.** Ubuntu's default group layout differs, and
`gpio`/`i2c`/`spi` are a Raspberry Pi OS vendor addition rather than something
Ubuntu creates. The Ansible `camera` role re-adds `video`, which is all the camera
milestones need; GPIO access is left until [roadmap.md](roadmap.md) step 7 actually
requires it.

### Stray apt source

The Pi currently carries a Docker apt source pointing at
`download.docker.com/linux/ubuntu` with `Suites: bookworm` — a wrong-distro
combination that will fail on `apt update`. Docker is not installed. The reflash
removes it, so no action is needed before then.

## Dev box — the visualisation side

| Property | Value |
| --- | --- |
| Hostname | `test` |
| IP | `192.168.2.106/24` on interface `eth2` |
| OS | Ubuntu 24.04.4 LTS |
| Architecture | `x86_64` |
| Docker | 28.5.1 |
| Ansible | core 2.16.3 (`ansible.posix` 1.5.4, `community.general` 8.3.0) |
| Python | 3.12.3 |
| ROS installed | **No** |

Ubuntu 24.04 (noble) is the **Tier 1 platform for ROS 2 Jazzy**, so this machine
can install ROS natively from `apt`. That is what makes it the right place to run
`rviz2` and `rqt` — and once the Pi is reflashed, both machines share one install
path and one set of [Ansible](ansible.md) roles.

Ansible is already installed here, so this host is the control node and needs no
bootstrapping.

> This host also has ~11 Docker bridge networks (`172.17.0.0/16` … `172.26.0.0/16`).
> They are harmless for normal work but they actively interfere with DDS discovery —
> see [networking.md](networking.md). Note that dropping Docker from the *Pi* does
> not remove this problem: the bridges are on the **dev box**.

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

The camera is unaffected by the planned OS change: it is a standard UVC device
driven by the in-tree `uvcvideo` module, which Ubuntu's Pi kernel carries just as
the vendor kernel does. The measurements below should hold across the reflash —
re-run them afterwards to confirm rather than assuming.

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

The `pispbe` and `rpi-hevc-dec` entries in `v4l2-ctl --list-devices`
(`/dev/video19`–`/dev/video35`) are the Pi 5's built-in ISP and HEVC decoder
blocks, not cameras. Ignore them. These are exposed by the vendor kernel, so expect
the numbering to differ under Ubuntu — one more reason to identify the camera by
its `/dev/v4l/by-id/` symlink rather than by node number.

There is no CSI ribbon camera attached — only the USB webcam. This means
`libcamera`/`rpicam-apps`, the usual Raspberry Pi camera path, is **not** the
route to use here; standard V4L2 is. It also means losing that stack in the
reflash costs nothing.
