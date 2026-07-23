# Setting up the Pi (native ROS 2 Jazzy on Ubuntu 24.04)

> **Status: planned.** The Pi still runs Raspberry Pi OS (Debian 12 bookworm) as of
> 2026-07-23 — see [hardware.md](hardware.md). Nothing on this page has been carried
> out yet. It is the intended path, not a record of what is installed.

The Pi is reflashed with **Ubuntu Server 24.04 LTS for arm64** so that ROS 2 Jazzy
installs natively from `apt`, exactly as it does on the dev box. Both machines then
run the same stock ROS, provisioned by the same Ansible roles —
[ansible.md](ansible.md).

## Why reflash rather than work around Debian

ROS 2 publishes binaries per platform tier. Jazzy's Tier 1 platform is **Ubuntu
24.04 noble**, on `amd64` *and* `arm64`. Debian 12 bookworm — which Raspberry Pi OS
is built on — is Tier 3: expected to build from source, with **no binary packages
published**.

That is checkable rather than a matter of opinion. Count the `ros-jazzy-*` packages
each suite actually serves for `arm64`:

```bash
# Ubuntu 24.04 noble — the target
curl -s http://packages.ros.org/ros2/ubuntu/dists/noble/main/binary-arm64/Packages.gz \
  | gunzip | grep -c '^Package: ros-jazzy'
# → 3361

# Debian 12 bookworm — what the Pi runs today
curl -s http://packages.ros.org/ros2/ubuntu/dists/bookworm/main/binary-arm64/Packages.gz \
  | gunzip | grep -c '^Package: ros-jazzy'
# → 0
```

(Verified 2026-07-23. For scale, `noble/amd64` serves 3373 — arm64 is at parity, not
a stripped-down afterthought.)

The `bookworm` suite is misleading precisely because it exists: it returns HTTP 200
and lists `arm64` among its architectures. Its contents are only bootstrap tooling —
`python3-rosdep`, `python3-vcstool`, `python3-rosdistro`, `ros2-apt-source` — the
tools you would use to *build* ROS, not ROS itself.

The four ways out, and why this one:

| Option | Verdict |
| --- | --- |
| **Reflash to Ubuntu Server 24.04 arm64** | **Chosen.** Tier 1 platform, plain `apt`, identical to the dev box. One set of Ansible roles provisions both machines, and every tutorial and error message you will search for assumes this layout. |
| Docker (`ros:jazzy-ros-base`) | Works and keeps the host OS untouched, but adds a container boundary to every debugging session — device passthrough, `network_mode: host` for DDS, `ipc: host` for shared memory, and a rebuild whenever a dependency changes. Rejected: the indirection costs more than it saves on a learning project. |
| Build ROS 2 from source on Raspberry Pi OS | Several hours of compilation on 4 cores, repeated on every update, with Tier 3 build breakages to debug yourself. |
| RoboStack (conda/pixi) | Viable and avoids both the reflash and the compile, but ROS lands in a conda environment rather than `/opt/ros`, and it is a less-travelled path with fewer troubleshooting references. |

## What the reflash costs

Worth going in with eyes open — the current Raspberry Pi OS install is discarded:

- **The Raspberry Pi vendor kernel** (`6.12.87+rpt-rpi-2712`) is replaced by Ubuntu's
  Pi kernel. Fine for this project; the C922 is a standard UVC device driven by
  in-tree `uvcvideo`, which both kernels have.
- **`libcamera`/`rpicam-apps`** go away. Irrelevant here — there is no CSI ribbon
  camera attached, so that stack was never the route. See [camera.md](camera.md).
- **Group membership is rebuilt.** The Raspberry Pi OS user is currently in `video`,
  `gpio`, `i2c`, `spi`, `dialout`, `plugdev` and `render`. Ubuntu's default set is
  different, and the layout for GPIO/I²C/SPI access is not identical. The Ansible
  `camera` role re-adds `video` (needed for `/dev/video0` without `sudo`); the
  GPIO side is deferred to the milestone that actually needs it —
  [roadmap.md](roadmap.md) step 7.
- **SSH host keys are regenerated**, so the first connection after reflashing will
  fail loudly. That is expected and covered in
  [troubleshooting.md](troubleshooting.md#ssh-pi--remote-host-identification-has-changed).

## 1. Flash the card

Use Raspberry Pi Imager and choose **Ubuntu Server 24.04 LTS (64-bit)** under
*Other general-purpose OS → Ubuntu*. Not Ubuntu Desktop — the GUI tools (`rviz2`,
`rqt`) run on the dev box, and a desktop session on the Pi only competes with the
camera pipeline for CPU.

In the Imager's settings pane, before writing, set:

| Setting | Value | Why |
| --- | --- | --- |
| Hostname | `raspberrypi` | Keeps `~/.ssh/config` and the docs consistent |
| Username | `bthek1` | Matches the `Host pi` block already on the dev box |
| SSH | Enabled, **public-key only** | Ansible needs non-interactive access from the first boot |
| Wi-Fi | Leave unset | The Pi is on wired Ethernet at `192.168.2.17` |

Paste the dev box's `~/.ssh/id_*.pub` into the public-key field. Getting this right
here is what makes the machine Ansible-reachable with zero manual steps afterwards.

## 2. First boot

Ubuntu's first boot runs cloud-init, which takes a minute or two beyond the point
where SSH starts answering. Clear the stale host key, then wait for it to settle:

```bash
ssh-keygen -f ~/.ssh/known_hosts -R 192.168.2.17
ssh pi 'cloud-init status --wait'
```

Confirm the new fingerprint from the Pi's own console before trusting it —
comparing against `ssh-keyscan` from the same machine proves nothing, since a
man-in-the-middle would serve the same forged key to both.

Then check you landed where you expected:

```bash
ssh pi 'lsb_release -a; uname -m; nproc; free -h'
# Expect: Ubuntu 24.04.x LTS, aarch64, 4, ~7.9 GiB
```

Also confirm the camera survived the OS change:

```bash
ssh pi 'v4l2-ctl --list-devices; ls -l /dev/video0'
```

`v4l-utils` may not be present until the Ansible run installs it; `ls -l /dev/video0`
alone is enough to confirm the device enumerated.

If the IP has moved, fix it before going further — the address is baked into
[networking.md](networking.md) and the Ansible inventory.

## 3. Hand over to Ansible

Everything past this point is provisioning, and none of it is done by hand:

```bash
# from the dev box
cd ~/piros2/ansible
ansible-playbook site.yml --limit robot
```

That installs the ROS apt source, `ros-jazzy-ros-base`, the camera and image
transport packages, the Cyclone DDS RMW, the shell environment, and the
`CYCLONEDDS_URI` config — see [ansible.md](ansible.md) for what each role does and
how to run it safely.

The Pi gets `ros-jazzy-ros-base`, not `ros-jazzy-desktop`. It is a sensor head; it
has no reason to carry `rviz2` and the Qt stack.

## 4. Verify

```bash
ssh pi 'source /opt/ros/jazzy/setup.bash && ros2 doctor --report | head -30'
```

Then a talker/listener pair across the LAN:

```bash
# on the Pi
ssh pi
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker

# on the dev box
source /opt/ros/jazzy/setup.bash
ros2 topic echo /chatter
```

If nothing arrives, the problem is almost certainly DDS discovery rather than the
install — [networking.md](networking.md) covers it, and the dev box's Docker bridge
interfaces are the usual culprit.

## Keeping the workspace in sync

The repo lives on the dev box and needs to reach the Pi for `colcon build` to have
anything to build:

```bash
# from the dev box
rsync -av --delete --exclude build --exclude install --exclude log \
      ~/piros2/ pi:~/piros2/
```

The Ansible `workspace` role does the same thing as part of a run, so a full
`ansible-playbook site.yml` also syncs. Or push/pull through the GitHub remote
(`git@github.com:bthek1/piros2.git`) if you prefer every sync to be a commit.
