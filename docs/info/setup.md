# Setup

> **Status: done (2026-07-24).** The Pi was reflashed 2026-07-23 (steps 1–2 are
> the record, traps included) and both machines are now provisioned by the
> playbook: `ros-base` + camera stack on the Pi, `desktop` on the dev box, one
> shared environment. The step 6 verification passed — a talker on the Pi
> reached a listener on the dev box across the LAN — and the playbook is
> idempotent on both machines (the dev box's first-run kernel/DKMS stumble was
> resolved the same day —
> [troubleshooting.md](troubleshooting.md#apt-fails-on-linux--kernel-packages-dev-box)).

Both machines run ROS 2 Jazzy natively from `apt`, and one playbook provisions
both. Only two things are done by hand: reflashing the Pi, and bootstrapping the
control node. Everything after that is `ansible-playbook site.yml` —
[ansible.md](ansible.md).

| | Dev box `ml5` | Raspberry Pi |
| --- | --- | --- |
| Role | Ansible control node, visualisation | Managed host, sensor head |
| OS | Ubuntu 24.04.4 LTS, x86_64 | Ubuntu 24.04.4 LTS arm64 *(reflashed 2026-07-23)* |
| ROS | `ros-jazzy-desktop` | `ros-jazzy-ros-base` |
| Runs | `rviz2`, `rqt`, editing, builds | camera node, anything touching hardware |

The Pi gets `ros-base`, not `desktop`. It is a sensor head; it has no reason to
carry `rviz2` and the Qt stack.

## Why Ubuntu on both

ROS 2 publishes binaries per platform tier. Jazzy's Tier 1 platform is **Ubuntu
24.04 noble**, on `amd64` *and* `arm64`. Debian 12 bookworm — which Raspberry Pi OS
is built on — is Tier 3: expected to build from source, with **no binary packages
published**.

That is checkable rather than a matter of opinion:

```bash
# Ubuntu 24.04 noble — what the Pi runs now
curl -s http://packages.ros.org/ros2/ubuntu/dists/noble/main/binary-arm64/Packages.gz \
  | gunzip | grep -c '^Package: ros-jazzy'
# → 3361

# Debian 12 bookworm — what Raspberry Pi OS was built on
curl -s http://packages.ros.org/ros2/ubuntu/dists/bookworm/main/binary-arm64/Packages.gz \
  | gunzip | grep -c '^Package: ros-jazzy'
# → 0
```

Use `http://`, not `https://` — HTTPS to `packages.ros.org` returned nothing from
the dev box while HTTP worked. Worth remembering if `apt update` ever stalls on
that host.

(Verified 2026-07-23. `noble/amd64` serves 3373, so arm64 is at parity rather than
a stripped-down afterthought.)

The `bookworm` suite is misleading precisely because it exists: it returns HTTP 200
and lists `arm64` among its architectures. Its contents are only bootstrap tooling —
`python3-rosdep`, `python3-vcstool`, `python3-rosdistro`, `ros2-apt-source` — the
tools you would use to *build* ROS, not ROS itself.

The alternatives, and why the reflash won:

| Option | Verdict |
| --- | --- |
| **Reflash to Ubuntu Server 24.04 arm64** | **Chosen.** Tier 1 platform, plain `apt`, identical to the dev box. One set of Ansible roles provisions both machines, and every tutorial and error message you will search for assumes this layout. |
| Docker (`ros:jazzy-ros-base`) | Works and keeps the host OS untouched, but adds a container boundary to every debugging session — device passthrough, `network_mode: host` for DDS, `ipc: host` for shared memory, and a rebuild whenever a dependency changes. Rejected: the indirection costs more than it saves. |
| Build ROS 2 from source on Raspberry Pi OS | Several hours of compilation on 4 cores, repeated on every update, with Tier 3 build breakages to debug yourself. |
| RoboStack (conda/pixi) | Avoids both the reflash and the compile, but ROS lands in a conda environment rather than `/opt/ros`, and it is a less-travelled path with fewer troubleshooting references. |

### What the reflash cost

The Raspberry Pi OS install was discarded. What actually changed:

- **The vendor kernel** (`6.12.87+rpt-rpi-2712`) was replaced by Ubuntu's
  `6.8.0-1047-raspi`. Fine here — the C922 is a standard UVC device driven by
  in-tree `uvcvideo`, which both kernels have, and it re-enumerated as
  `/dev/video0` unchanged.
- **`libcamera`/`rpicam-apps`** went away. Irrelevant: there is no CSI ribbon
  camera attached, so that stack was never the route. See [camera.md](camera.md).
- **Group membership was rebuilt.** `gpio`/`i2c`/`spi` no longer exist on the
  machine. `video` was re-added via cloud-init, which is all the camera milestones
  need; GPIO waits until [roadmap.md](roadmap.md) step 7 requires it.
- **SSH host keys were regenerated**, so the first connection afterwards failed
  loudly. Expected, and covered in
  [troubleshooting.md](troubleshooting.md#ssh-pi--remote-host-identification-has-changed).
- **`v4l2-ctl` is gone** — Ubuntu Server does not ship `v4l-utils`. Install it
  before any of the camera commands in these docs will run.
- **The bootloader was downgraded** to Ubuntu's bundled 2024-09-23 build. See
  [hardware.md](hardware.md#boot-configuration).

## 1. Flash the card

Raspberry Pi Imager is the usual route, but this reflash was done by writing the
image directly — the card was driven from another Linux host, and the same
`dd`-plus-cloud-init procedure works headlessly from anywhere.

Use the **preinstalled server image for Raspberry Pi**, not a generic arm64 ISO. A
desktop or installer ISO has no Pi bootloader partition (`config.txt`, the
`start*.elf` firmware) and a Pi will not boot it at all:

```bash
IMG=ubuntu-24.04.4-preinstalled-server-arm64+raspi.img.xz
curl -sSLO https://cdimage.ubuntu.com/releases/24.04/release/$IMG
curl -sSLO https://cdimage.ubuntu.com/releases/24.04/release/SHA256SUMS
sha256sum -c SHA256SUMS --ignore-missing
```

Server, not Desktop — the GUI tools run on the dev box, and a desktop session on
the Pi only competes with the camera pipeline for CPU.

Identify the target device **by diffing `lsblk` before and after inserting it**.
Never guess: a wrong `of=` silently destroys whatever else is attached.

```bash
sudo umount /dev/sdX?* 2>/dev/null
xz -dc "$IMG" | sudo dd of=/dev/sdX bs=4M status=progress conv=fsync
sync && sudo partprobe /dev/sdX
```

The result should be an MBR (`dos`) label with partition 1 flagged bootable, type
`c` (W95 FAT32 LBA), labelled `system-boot`, plus an ext4 `writable` partition
that grows to fill the medium on first boot.

## 2. Configure before first boot

Mount the `system-boot` partition and replace two files. **Skipping this step is
how you end up with an unreachable machine** — the stock image gives you an
`ubuntu`/`ubuntu` account with a forced password change, which breaks key auth.

`user-data` — the cloud-init config:

```yaml
#cloud-config
hostname: raspberrypi
manage_etc_hosts: true

groups: [video, dialout, plugdev, render]

users:
  - name: bthek1
    groups: [adm, sudo, video, dialout, plugdev, render]
    shell: /bin/bash
    lock_passwd: false
    passwd: "<sha512 hash — mkpasswd --method=SHA-512>"
    sudo: "ALL=(ALL) NOPASSWD:ALL"
    ssh_authorized_keys:
      - <contents of the dev box's ~/.ssh/id_ed25519.pub>

ssh_pwauth: false
package_update: true
```

Two traps live in that file. **Never list a group that does not exist** —
`i2c`, `spi`, and `gpio` are Raspberry Pi OS inventions, and naming one makes
`useradd` fail, so the account is never created and a headless machine comes up
with no way in. Declaring groups under the top-level `groups:` key first is the
safe form. And **set a real password hash**: on a Wi-Fi-only machine, key-only auth
means a Wi-Fi problem locks you out completely, so a console fallback matters.

`network-config` — **required, because the Pi has no Ethernet cable attached.**
Ubuntu Server's image does not join Wi-Fi unaided:

```yaml
version: 2
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "THEKKEL_MESH":
        password: "<psk>"
```

The existing PSK can be lifted off a running Raspberry Pi OS install from
`psk=` in `/etc/NetworkManager/system-connections/preconfigured.nmconnection`.
It is stored there as a 64-hex-digit pre-shared key rather than a passphrase,
which netplan accepts directly — a passphrase is capped at 63 characters, so a
64-character value is a PSK, not a truncated password.

Validate before unmounting; a YAML typo here is only discoverable by console:

```bash
python3 -c "import yaml; yaml.safe_load(open('user-data')); yaml.safe_load(open('network-config'))"
```

### Booting from USB

If the target is a USB stick rather than the SD card, the Pi 5's boot order has to
prefer it. `BOOT_ORDER` nibbles are read **right to left**, so `0xf14` means USB
(`4`), then SD (`1`), then restart (`f`) — which keeps the SD card as a one-step
rollback:

```bash
sudo rpi-eeprom-config > /tmp/boot.conf
sed -i 's/^BOOT_ORDER=.*/BOOT_ORDER=0xf14/' /tmp/boot.conf
sudo rpi-eeprom-config --apply /tmp/boot.conf
```

**A warm reboot is not enough.** `reboot` restarts the SoC without power-cycling
the USB ports, so a hot-plugged stick often is not re-enumerated inside the
bootloader's discovery window and it falls through to the SD card. Either cold
power-cycle, or widen the window:

```
USB_MSD_PWR_OFF_TIME=3000
USB_MSD_DISCOVER_TIMEOUT=30000
```

Adding those made USB boot work on a warm reboot where it had failed repeatedly.
Confirm which medium the firmware actually used — `01` is SD, `04` is USB:

```bash
ssh pi 'od -An -tx1 /proc/device-tree/chosen/bootloader/boot-mode'
```

### The duplicate-label trap

Ubuntu's Pi image labels its partitions `system-boot` and `writable`, and
`cmdline.txt` finds the root filesystem with `root=LABEL=writable`. Write the same
image to two devices — a USB stick *and* the SD card — and **both carry identical
labels, identical filesystem UUIDs, and identical PARTUUIDs**, because the PARTUUID
derives from the MBR disk identifier baked into the image. The kernel then roots
into whichever it enumerated first, non-deterministically.

Give one of them a distinct identity and pin to it:

```bash
sudo sfdisk --disk-id /dev/mmcblk0 0x5ec0ffee   # PARTUUIDs become 5ec0ffee-01/-02
sudo tune2fs -U random /dev/mmcblk0p2           # fresh filesystem UUID
```

Then rewrite both references — `cmdline.txt` on the boot partition and `/etc/fstab`
on the root filesystem:

```
root=PARTUUID=5ec0ffee-02          # cmdline.txt

PARTUUID=5ec0ffee-02  /              ext4  defaults  0  1
PARTUUID=5ec0ffee-01  /boot/firmware vfat  defaults  0  1
```

With both pinned, the clone can stay plugged in harmlessly. Leaving it on labels
is a coin-flip every boot.

## 3. First boot

Ubuntu's first boot runs cloud-init, which takes a few minutes past the point where
SSH starts answering — it resizes the root filesystem as well as applying the
config. Clear the stale host key, then wait for it to settle:

```bash
ssh-keygen -f ~/.ssh/known_hosts -R 192.168.2.17
ssh pi 'cloud-init status --wait'
```

Confirm the new fingerprint from the Pi's own console before trusting it —
comparing against `ssh-keyscan` from the same machine proves nothing, since a
man-in-the-middle would serve the same forged key to both.

Then check you landed where you expected, and that the camera survived:

```bash
ssh pi 'lsb_release -a; uname -rm; nproc; free -h; findmnt /'
ssh pi 'ls -l /dev/video0; id -nG'
```

A `degraded` cloud-init status is worth reading rather than fearing: `cloud-init
status --long` distinguishes real errors from recoverable warnings, and
"Skipping creation of existing group" is only the latter.

If the IP has moved, fix that before going further — the address is baked into
[networking.md](networking.md) and the Ansible inventory.

## 4. Bootstrap the control node

The dev box is the control node. Ansible is already installed on it (core 2.16.3),
so this is only:

```bash
git clone git@github.com:bthek1/piros2.git ~/Documents/piros2
cd ~/Documents/piros2/ansible
ansible all -m ping        # both hosts must answer before going further
```

## 5. Run the playbook

```bash
ansible-playbook site.yml --check --diff    # dry run — read the diff first
ansible-playbook site.yml                   # both machines
```

That installs the ROS apt source, the right metapackage per host, the camera and
image-transport packages, the Cyclone DDS RMW, the shell environment, and the
`CYCLONEDDS_URI` config. [ansible.md](ansible.md) covers what each role does and
the gotchas worth knowing before the first run.

<details>
<summary>What the playbook is doing, in plain apt commands</summary>

Useful to read once — the roles are only these steps made idempotent, and knowing
them is what lets you tell a broken role from a working one.

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

# The apt source ships as a .deb that keeps its own signing key current.
export ROS_APT_SOURCE_VERSION=$(
  curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y ros-jazzy-desktop            # ros-jazzy-ros-base on the Pi
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

sudo rosdep init      # once per machine; not idempotent, hence the creates: guard
rosdep update         # as your own user, NOT under sudo
```

`rosdep update` writes to `~/.ros`, so running it with `sudo` puts the cache in
root's home where your builds will not find it. Same trap in the role —
[ansible.md](ansible.md#gotchas-specific-to-provisioning-ros).

</details>

### On sourcing ROS

Every shell gets ROS — bash and fish, login and interactive. Until **2026-08-19**
the role wrote only the four variables plus a `rosjazzy` alias and left sourcing
manual, the worry being that ROS's Python and library paths leak into unrelated
projects on the same machine. That cost is real but small; typing `ros2` and
getting `command not found` was the larger one.

Sourcing ROS's own setup scripts costs **0.40 s** of Python per shell, which is
too much to pay for every terminal tab, so no shell sources them.
`~/.config/ros2/env-delta` runs them once, diffs `env` across the source, and
prints the *delta* as shell code; both shells read the cached output instead:

```
~/.config/ros2/env-delta           # the generator: source for real, print the delta
~/.config/ros2/setup.sh            # bash: refresh the cache if stale, then source it
~/.config/fish/conf.d/ros2.fish    # fish: the same, autoloaded
~/.cache/ros2/jazzy-env.sh         # the cache — one per shell syntax
~/.cache/ros2/jazzy-env.fish
```

The dotfiles only point at the snippet:

```bash
# ~/.profile — read by all login shells, interactive or not
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/bthek1/.config/cyclonedds/cyclonedds.xml
[ -r "$HOME/.config/ros2/setup.sh" ] && . "$HOME/.config/ros2/setup.sh"

# ~/.bashrc — interactive shells, which may not be login shells
alias rosjazzy='source /opt/ros/jazzy/setup.bash'
[ -r "$HOME/.config/ros2/setup.sh" ] && . "$HOME/.config/ros2/setup.sh"
```

Both dotfiles because they cover different shells: a new terminal tab is
interactive but *not* a login shell, while `ssh host "bash -lc '...'"` — how this
project verifies over SSH — is a login shell but not interactive. The snippet
guards on `ROS_DISTRO`, so the shell that reads both files still sources once,
and on `BASH_VERSION`, because `/bin/sh` reads `.profile` at graphical login and
cannot run a bash setup script. The exports stay in `.profile` and not `.bashrc`
because Ubuntu's `.bashrc` returns immediately in non-interactive shells. The
four values **must match on both machines** — [networking.md](networking.md)
explains each. They are defined once in `group_vars/all.yml`, so change them
there, not in a dotfile; the overlay path comes from `workspace_path` the same
way.

**The delta, never a copy of the caller's environment.** Six variables ending in
`PATH` (`AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, `COLCON_PREFIX_PATH`,
`PYTHONPATH`, `LD_LIBRARY_PATH`, `PATH`) get ROS's entries *prepended* to
whatever the shell already had — `set -gx --prepend` in fish, which stores them
as real lists rather than colon-joined strings, and
`export VAR=added"${VAR:+:${VAR}}"` in sh. Three scalars (`ROS_DISTRO`,
`ROS_VERSION`, `ROS_PYTHON_VERSION`) are set outright. Writing bash's `PATH`
into fish wholesale would flatten fish's own path list; prepending the delta
leaves it intact.

**What an environment cache cannot carry** is `complete` registrations and shell
functions. The only ones that matter here are ros2/rosidl tab completion, so
interactive bash sources `ros2-argcomplete.bash` itself — that is the whole of
its ~40 ms over a login shell.

**Staleness** is a timestamp check: the cache is regenerated when
`/opt/ros/jazzy/setup.bash` or the workspace's `install/local_setup.bash` is
newer. colcon rewrites the latter on every build, so a newly built package
invalidates the cache by itself — there is no stale overlay to debug. If the
generator fails, bash falls back to sourcing the real scripts, slowly; fish has
no fallback available — it cannot source them at all — so a broken generator
costs fish its ROS environment while bash keeps working.

Measured on the dev box, best of three:

| | sourcing off | warm cache | cache regenerated |
| --- | --- | --- | --- |
| `bash -lc true` | 0.10 s | 0.12 s | 0.55 s |
| `bash -ic true` | 0.18 s | 0.22 s | — |
| `fish -c true` | 0.14 s | 0.15 s | 0.59 s |

**Opting out:** `ROS_AUTO_SOURCE=0` skips the sourcing (not the variables) — put
it in another project's `.envrc` if ROS's `PYTHONPATH` gets in the way.

**fish is dev-box-only**: the Pi runs bash and has no reason for more, so the
role's fish tasks are guarded on `/usr/bin/fish` existing rather than on the
inventory group.

## 6. Verify

Open a *fresh* shell first — the role sources ROS for new shells, not for the
one that ran the playbook:

```bash
ros2 doctor --report | head -30
```

Locally first — a talker and listener in two terminals on the dev box. If that
works, the install is sound. Then across the LAN:

```bash
# on the Pi
ssh pi -t 'bash -lc "ros2 run demo_nodes_cpp talker"'

# on the dev box
ros2 topic echo /chatter
```

Note the login shell (`bash -lc`) — a plain `ssh pi '...'` reads no dotfile at
all, so it finds neither the `ros2` command nor the ROS environment, and would
land on domain 0 if it did. That trips people up constantly; see
[troubleshooting.md](troubleshooting.md).

If nothing arrives, the problem is almost certainly DDS discovery rather than the
install — [networking.md](networking.md) covers it, and the dev box's Docker bridge
interfaces are the usual culprit.

## Workspace layout

The repository doubles as the colcon workspace:

```
~/Documents/piros2/      # on the Pi: ~/piros2/
├── src/                 # ROS 2 packages live here
├── ansible/             # provisioning for both machines
├── config/              # cyclonedds.xml, camera calibration
├── docs/
├── build/  install/  log/    # generated by colcon, git-ignored
└── CLAUDE.md  README.md
```

Build and source with:

```bash
cd ~/Documents/piros2    # on the Pi: cd ~/piros2
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash    # only for *this* shell; new ones pick it up themselves
```

`--symlink-install` means edits to Python nodes and launch files take effect
without rebuilding — worth having on from the start. The shell that ran the
build still needs `source install/setup.bash` to see a newly created package;
shells opened afterwards get it from the refreshed cache. If you ever source by
hand, the overlay goes *after* `/opt/ros/jazzy/setup.bash` — overlay last.

## Keeping the Pi in sync

The repo lives on the dev box and needs to reach the Pi for `colcon build` there to
have anything to build:

```bash
rsync -av --delete --exclude build --exclude install --exclude log \
      ~/Documents/piros2/ pi:~/piros2/
```

The Ansible `workspace` role does the same as part of a run, so a full
`ansible-playbook site.yml` also syncs.
