# Provisioning with Ansible

> **Status: planned.** The `ansible/` directory does not exist yet. This page
> describes the intended layout so that the setup docs have something concrete to
> point at. Ansible itself *is* already installed on the dev box (core 2.16.3).

Both machines run the same native ROS 2 Jazzy install, so both are provisioned by
the same playbook. The dev box is the control node; the Pi is a managed host.

## Why bother, on a two-machine project

Not to save typing. The real reason is in
[networking.md](networking.md#the-three-things-that-must-match): `ROS_DOMAIN_ID`,
`ROS_LOCALHOST_ONLY` and `RMW_IMPLEMENTATION` must be **identical on both hosts**,
and most of [troubleshooting.md](troubleshooting.md) is a catalogue of what happens
when they quietly drift apart. Written by hand, those values live in two shells,
two `.bashrc` files and several docs. As Ansible `group_vars` they have exactly one
definition, and re-running the playbook is what makes drift impossible rather than
merely unlikely.

The second reason is the reflash. [setup-pi.md](setup-pi.md) wipes the Pi. A
playbook means that costs one command instead of an afternoon of remembering what
was installed.

What Ansible does **not** do is teach you ROS. Work through
[setup-dev.md](setup-dev.md) by hand first, then write the role from what you
actually ran — you will write a better role, and you will be able to tell when it
is wrong.

## Prerequisites

Already satisfied on the dev box, recorded here so it is checkable:

```bash
ansible --version | head -1        # → ansible [core 2.16.3]
ansible-galaxy collection list | grep -E 'ansible.posix|community.general'
# → ansible.posix 1.5.4 / community.general 8.3.0
```

The Pi needs only Python 3 and key-based SSH, both of which Ubuntu Server 24.04
ships by default. No agent, no bootstrap package.

## Layout

```
ansible/
├── ansible.cfg
├── inventory.yml
├── site.yml
├── group_vars/
│   ├── all.yml           # ROS distro, domain ID, RMW — the shared truth
│   ├── dev.yml           # desktop metapackage, DDS interface eth2
│   └── robot.yml         # ros-base metapackage, DDS interface eth0
└── roles/
    ├── ros2_apt/         # ros2-apt-source .deb, keyring
    ├── ros2_install/     # metapackage + colcon/rosdep/vcstool
    ├── ros2_env/         # .bashrc block, cyclonedds.xml template
    ├── camera/           # v4l-utils, video group  (robot only)
    └── workspace/        # repo sync, rosdep install, colcon build
```

### Inventory

```yaml
# ansible/inventory.yml
all:
  children:
    dev:
      hosts:
        test:
          ansible_connection: local      # no SSH round-trip to ourselves
    robot:
      hosts:
        pi:                              # resolved via ~/.ssh/config
          ansible_host: 192.168.2.17
          ansible_user: bthek1
```

`ansible_connection: local` on the dev box matters — without it Ansible tries to
SSH to `test` and you need a loopback key for no reason.

### The shared variables

```yaml
# ansible/group_vars/all.yml
ros_distro: jazzy
ros_domain_id: 42
ros_localhost_only: 0
rmw_implementation: rmw_cyclonedds_cpp
workspace_path: "{{ ansible_env.HOME }}/piros2"
```

```yaml
# ansible/group_vars/dev.yml
ros_metapackage: ros-jazzy-desktop      # rviz2, rqt, demo nodes
dds_interface: eth2                     # NOT one of the 11 docker bridges
```

```yaml
# ansible/group_vars/robot.yml
ros_metapackage: ros-jazzy-ros-base     # sensor head — no GUI stack
dds_interface: eth0
```

Templating `config/cyclonedds.xml` from `dds_interface` is the point where Ansible
earns its keep: the file is per-host (the dev box must pin `eth2`, the Pi need not
pin anything) but the surrounding structure is identical, and hand-maintaining two
near-identical XML files is how they end up disagreeing.

## Gotchas specific to provisioning ROS

These are the ones that will actually bite:

- **`rosdep init` is not idempotent.** It exits non-zero if run twice. Guard it, or
  the second play run fails on a machine that is already correct:

  ```yaml
  - name: Initialise rosdep
    command: rosdep init
    args:
      creates: /etc/ros/rosdep/sources.list.d/20-default.list
    become: true
  ```

  `rosdep update` is the opposite — it runs as the *login user*, not root, and
  running it under `become: true` puts the cache in the wrong home directory.

- **Do not source ROS globally.** [setup-dev.md](setup-dev.md#shell-setup) explains
  why. Use `blockinfile` with a marker so the block is editable and removable:

  ```yaml
  - name: ROS environment in .bashrc
    blockinfile:
      path: "{{ ansible_env.HOME }}/.bashrc"
      marker: "# {mark} ANSIBLE MANAGED — ROS 2 {{ ros_distro }}"
      block: |
        alias ros{{ ros_distro }}='source /opt/ros/{{ ros_distro }}/setup.bash'
        export ROS_DOMAIN_ID={{ ros_domain_id }}
        export ROS_LOCALHOST_ONLY={{ ros_localhost_only }}
        export RMW_IMPLEMENTATION={{ rmw_implementation }}
        export CYCLONEDDS_URI=file://{{ workspace_path }}/config/cyclonedds.xml
  ```

- **Restart the ROS daemon on change.** Any task touching the environment or the
  Cyclone DDS config should notify a handler running
  `ros2 daemon stop && ros2 daemon start`. The daemon caches discovery state, and
  skipping this is what makes a correct fix look like it did nothing.

- **`apt` needs `become: true`; `colcon build` must not have it.** Building as root
  leaves a `build/` tree the login user cannot overwrite, and the failure surfaces
  much later as a confusing permission error.

- **Adding a user to `video` does not affect the current session.** Group changes
  need a fresh login. Follow the task with `meta: reset_connection`, or the camera
  checks later in the same play fail against a stale session.

- **`sudo` may prompt.** The Pi user is in `sudo`. Either run with
  `--ask-become-pass` or configure NOPASSWD deliberately — do not let a play hang
  half-finished on a password prompt.

## Running it

```bash
cd ~/piros2/ansible

ansible all -m ping                        # reachability first
ansible-playbook site.yml --check --diff   # dry run — read the diff
ansible-playbook site.yml                  # both machines
ansible-playbook site.yml --limit robot    # just the Pi
ansible-playbook site.yml --tags env       # just re-sync the ROS env vars
```

`--check --diff` before the first real run is worth the habit, particularly for the
`blockinfile` and template tasks where you can see exactly what will change in
`.bashrc`.

## What stays manual

Automating these would be more trouble than it is worth:

- Flashing the SD card and the Pi's first boot — [setup-pi.md](setup-pi.md).
- Camera calibration. It is interactive by design: you hold a checkerboard in front
  of the lens. [camera.md](camera.md#calibration).
- Anything in RViz or `rqt`.
- The actual learning in [roadmap.md](roadmap.md). Ansible provisions the
  environment; the milestones are the point of the project.
