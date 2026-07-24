# Ansible provisioning — build plan

> **Working document.** [ansible.md](ansible.md) is the design — what the tree
> looks like and why. This is the order to build it in, what each step must prove
> before moving on, and the decisions still open. Delete this file once `ansible/`
> exists and the playbook is green.

The plan is deliberately incremental: **every step ends with something you can run
and check.** A provisioning tree written in one go and debugged afterwards is the
slowest possible route, because a failure at step 6 could have come from anywhere.

## Preconditions

Verified 2026-07-23, so the plan starts from fact rather than assumption:

| | Dev box `ml5` (192.168.2.109) | Pi (192.168.2.17) |
| --- | --- | --- |
| OS | Ubuntu 24.04.4, x86_64 | Ubuntu 24.04.4, aarch64 |
| CPU / RAM | 16 / 19 GiB | 4 / 7.8 GiB |
| Display | GNOME + Xwayland | headless |
| sudo | **prompts** → needs `--ask-become-pass` | passwordless |
| Ansible | core 2.16.3 + collections per `requirements.yml` (`ansible.posix` ≥ 2) | not needed |
| DDS interface | `enp6s18` (Docker bridges + 2 VPNs compete) | **`wlan0`** — no Ethernet cable |
| ROS | `desktop` installing 2026-07-24 | **`ros-base` installed 2026-07-24** |

> **Blockers cleared (2026-07-23).** `ml5` now reaches the Pi over key-based SSH:
> its key has been added to the Pi's `authorized_keys` and the stale pre-reflash
> host key removed — details in [ansible.md](ansible.md#prerequisites). Verified:
> `ssh -o BatchMode=yes bthek1@192.168.2.17 'hostname'` → `raspberrypi`.

The repo now lives on `ml5` at `~/Documents/piros2` (since 2026-07-23); everything
below assumes you are working from `~/Documents/piros2/ansible` on `ml5`.

All eleven packages this plan installs exist for `noble/arm64` — checked against
`packages.ros.org`, including `ros-jazzy-usb-cam`, `ros-jazzy-image-transport-plugins`
and `ros-jazzy-camera-calibration`.

## Build order

### Step 0 — Skeleton and reachability

**Done 2026-07-24** — both hosts answer `ansible all -m ping`; `ml5` uses
`ansible_connection: local`, and `workspace_path` moved into the per-group files
because the checkout lives at `~/Documents/piros2` on `ml5` but `~/piros2` on
the Pi.

`interpreter_python = auto_silent` in `ansible.cfg` suppresses a deprecation
warning on every run; `pipelining = True` roughly halves the round-trips to the Pi,
which matters over Wi-Fi.

**Proves:** `ansible all -m ping` returns `SUCCESS` for both hosts. It will fail
for `pi` until the key and host-key issues are fixed — that failure is the point of
running it first, rather than discovering it three roles later.

**Concept:** inventory groups map to `group_vars` files by name — `robot.yml`
applies to every host in the `robot` group, which is how one playbook serves two
different machines without conditionals scattered through the roles.

### Step 1 — `ros2_apt`

**Done 2026-07-24 on both hosts.** Built as planned, with one deviation: noble
ships `universe` enabled and uses deb822 sources that `apt_repository` predates,
so the role *asserts* universe rather than adding it. The bootstrap `.deb`
version is pinned in role defaults (guarded on the package being absent) —
`packages.ros.org` serves updates to `ros2-apt-source` itself afterwards.

Enable `universe`, then install the `ros2-apt-source` `.deb`. That package carries
the signing key and keeps it current, which is why it is preferred over a
hand-managed `apt-key`/keyring pair that silently expires.

Key tasks: `apt_repository` for universe → fetch the release `.deb` matching
`$VERSION_CODENAME` → `apt` install it → `apt update`.

**Proves:**

```bash
ansible all -m shell -a 'apt-cache policy ros-jazzy-ros-base | head -2'
```

Both hosts should show a candidate version rather than `(none)`.

**Concept:** ROS is not in Ubuntu's archive. Everything downstream depends on this
source being present and trusted, so it earns its own role.

### Step 2 — `ros2_install`

**Done on both hosts 2026-07-24.** On `ml5` every package installed and
configured, but the apt task reports failed until a pre-existing, unrelated
kernel/DKMS problem is cleared —
[troubleshooting.md](troubleshooting.md#apt-fails-on-linux--kernel-packages-dev-box).

Installs, per host, from `ros_metapackage`:

| Host | Packages |
| --- | --- |
| both | `{{ ros_metapackage }}`, `ros-jazzy-rmw-cyclonedds-cpp`, `python3-colcon-common-extensions`, `python3-rosdep`, `python3-vcstool` |
| dev | `ros-jazzy-desktop` (pulls `rviz2`, `rqt`, `demo_nodes_cpp`) |
| robot | `ros-jazzy-ros-base` |

Then `rosdep init` **with a `creates:` guard** and `rosdep update` **without
`become`** — both traps are spelled out in
[ansible.md](ansible.md#gotchas-specific-to-provisioning-ros).

> Budget time here. `ros-jazzy-desktop` is a large download, and `ros-base` on the
> Pi arrives over Wi-Fi. This is the slowest step by a wide margin; run it once
> before iterating on later roles.

**Proves:**

```bash
ansible all -m shell -a 'bash -lc "source /opt/ros/jazzy/setup.bash && ros2 pkg prefix demo_nodes_cpp"'
```

(Not `ros2 --version` — the Jazzy CLI has no such flag.)

**Concept:** the metapackage split is the whole dev-box/sensor-head distinction
made concrete — the Pi has no reason to carry the Qt stack.

### Step 3 — `ros2_env`

**Done on the Pi 2026-07-24**, after one real discovery: exports appended to
`.bashrc` are invisible to every non-interactive shell (Ubuntu's interactivity
guard returns first), so the env block lives in `.profile` and only the sourcing
alias stays in `.bashrc` — [ansible.md](ansible.md#gotchas-specific-to-provisioning-ros).
The role also generates the `en_US.UTF-8` locale that Ubuntu Server lacks.

Two things: the `blockinfile` in `.bashrc`, and `~/.config/cyclonedds/cyclonedds.xml`
rendered per host from `dds_interface` — outside the workspace, so the
`workspace` role's `rsync --delete` can never remove it.

This is where the project's actual risk lives. `ROS_DOMAIN_ID`,
`ROS_LOCALHOST_ONLY` and `RMW_IMPLEMENTATION` must be **byte-identical on both
hosts**; a mismatch produces silence, not an error.

The Cyclone DDS template pins the interface — `enp6s18` on the dev box so DDS
cannot bind to a Docker bridge, `tailscale0`, or the WireGuard `laptop` interface
and advertise an address the Pi cannot route to; `wlan0` on the Pi.

**Proves:**

```bash
ansible all -m shell -a 'bash -lc "printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION"'
```

Both must print `42` and `rmw_cyclonedds_cpp`. Two traps in one command:
`printenv`, not `echo $VAR`, because the remote's outer shell expands `$VAR`
(to empty) before `bash -lc` ever runs; and the **login shell**, because the
exports live in `.profile`, which non-login shells never read.

**Concept:** the environment variables DDS actually reads, and why "it works
locally but not across the LAN" is nearly always one of these three.

### Step 4 — Prove the two machines talk

**PASSED 2026-07-24** — `/chatter` from a talker on the Pi arrived at the dev
box on the first try: domain 42, CycloneDDS pinned per host, and Wi-Fi
multicast held up, so `ROS_STATIC_PEERS` stays unused (as the open-decisions
table leans). One note: Jazzy warns that `ROS_LOCALHOST_ONLY` is deprecated in
favour of `ROS_AUTOMATIC_DISCOVERY_RANGE` — ours is `0`/disabled, so the
warning is cosmetic for now.

Not a role. A checkpoint, and **the real milestone 0** —
[roadmap.md](roadmap.md) step 0 is not done until this passes.

```bash
# Pi
ssh pi -t 'bash -lc "source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_cpp talker"'
# dev box
source /opt/ros/jazzy/setup.bash && ros2 topic echo /chatter
```

`demo_nodes_cpp` comes with `desktop` on the dev box but **not** with `ros-base` —
either install `ros-jazzy-demo-nodes-cpp` on the Pi for this test, or run the
talker on the dev box and echo from the Pi.

If nothing arrives, stop and fix it here rather than building more on top.
[networking.md](networking.md) is the reference, and `ros2 daemon stop && ros2
daemon start` on **both** machines first — the daemon caches discovery state and
will keep reporting the pre-fix view.

> **Expect multicast trouble.** The Pi is on Wi-Fi, and most access points drop or
> rate-limit multicast. If discovery is flaky, this is the first suspect, not the
> last — go to [networking.md](networking.md#static-peers-if-multicast-is-unreliable)
> and try `ROS_STATIC_PEERS`. If that fixes it, promote it to a `group_vars`
> variable rather than leaving an export in one shell.

### Step 5 — `camera` (robot only)

**Done 2026-07-24** — camera stack installed, user in `video`, and the C922
asserted present at its serial-keyed symlink. One Ansible wrinkle: `meta:
reset_connection` ignores `when:` (core 2.16), so it runs unconditionally.

| Task | Why |
| --- | --- |
| install `v4l-utils` | **Ubuntu Server does not ship it.** Every `v4l2-ctl` command in the docs fails without it |
| install `ros-jazzy-usb-cam`, `ros-jazzy-image-transport-plugins`, `ros-jazzy-camera-info-manager` | the driver, compressed transport, and calibration plumbing |
| ensure user in `video` | already true via cloud-init — keep it anyway so the role is correct after a future reflash |
| `meta: reset_connection` after the group task | a group change does not affect the session that made it |
| assert `/dev/v4l/by-id/...` exists | fails loudly when the camera is unplugged, instead of later and cryptically |

`image_transport_plugins` is what makes `/image_raw/compressed` exist. Without it
the topic simply never appears and RViz's "Transport Hint: compressed" silently
shows nothing — and raw 720p RGB8 at 30 fps is ~83 MB/s, which does not fit over
Wi-Fi. This package is not optional on this project.

Identify the camera by its serial-keyed symlink, never `/dev/video0`:

```
/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5461327F-video-index0
```

**Proves:**

```bash
ssh pi 'v4l2-ctl --list-devices; id -nG | tr " " "\n" | grep -x video'
```

**Concept:** device permissions and stable device naming — the difference between
a pipeline that survives a replug and one that breaks silently.

### Step 6 — `workspace`

**Done 2026-07-24** — repo synced to `~/piros2` on the Pi, build tasks guarded
and skipped (only `src/.gitkeep` exists), and the idempotence proof passed: a
clean rerun of `site.yml` against the Pi reports `changed=0`.

Sync the repo to the Pi, then `rosdep install` and `colcon build --symlink-install`
as the **login user** — never under `become`, or `build/` ends up root-owned and
the failure surfaces much later as a confusing permission error.

> **`src/` is empty today.** `rosdep install --from-paths src` errors on an empty
> directory and `colcon build` has nothing to do. Guard both on `src/` containing a
> `package.xml`, or this role fails on a correct machine — which is exactly the
> kind of false negative that makes people stop trusting the playbook.

**Proves:** re-running `ansible-playbook site.yml` reports `changed=0`. Idempotence
is the property that makes the playbook worth having; test it explicitly.

## Open decisions

| Question | Options | Lean |
| --- | --- | --- |
| Restart the ROS daemon from a handler? | Needs ROS sourced as the login user, which is awkward from Ansible | Print a reminder instead of half-automating it |
| `ROS_STATIC_PEERS` from the start? | Wi-Fi multicast is unreliable | Wait for step 4 to fail first — do not pre-emptively work around a problem you have not observed |
| Custom udev rule for the camera? | `/dev/v4l/by-id/` already gives a stable path | Skip unless a shorter name proves genuinely useful |
| Pin ROS package versions? | Reproducibility vs. churn | Leave unpinned; this is a learning box, not production |
| Install `demo_nodes_cpp` on the Pi? | ~small, needed for step 4 | Yes, it makes the checkpoint symmetrical |

## What stays manual

Unchanged from [ansible.md](ansible.md#what-stays-manual): flashing the card,
camera calibration, anything in RViz or `rqt`, and the learning itself.

## Order of work, condensed

1. `ansible/` skeleton + inventory → `ansible all -m ping`
2. `ros2_apt` → `apt-cache policy ros-jazzy-ros-base`
3. `ros2_install` → `ros2 --version` on both
4. `ros2_env` → matching env vars on both
5. **talker/listener across the LAN** ← milestone 0 done
6. `camera` → `v4l2-ctl --list-devices` on the Pi
7. `workspace` → second run reports `changed=0`

Steps 1–4 are one afternoon if nothing fights back; step 5 is the one that can eat
a day, and it is the only one worth being patient about.
