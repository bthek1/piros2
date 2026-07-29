# Networking & DDS discovery

ROS 2 has no master process. Nodes find each other by broadcasting on the local
network, which is elegant when it works and opaque when it does not. This page
covers what needs to be true for the Pi and the dev box to see each other.

## The topology

```
  dev box "proxmox-ml5"                       Raspberry Pi 5
  192.168.2.109/24 ─ enp6s18 ─ LAN ─ wlan0 ─ 192.168.2.17
  Ubuntu 24.04, x86_64                        Ubuntu 24.04, aarch64
  ROS 2 Jazzy, native apt                     ROS 2 Jazzy, native apt
  rviz2, rqt, editor                          camera nodes
```

Note the Pi's side of that link is **`wlan0`** — it has no Ethernet cable
attached. That matters for multicast; see [static peers](#static-peers-if-multicast-is-unreliable)
below.

Both ends run ROS directly on the host, so there is no container boundary to
reason about — a node on the Pi is an ordinary process binding ordinary sockets.

Both are on the same `/24` subnet with no router in between, so multicast
discovery works. Latency from `ml5` to the Pi measures **4.2–19.1 ms, mean 7.3**
(five pings, 2026-07-23) — the spread is Wi-Fi on the Pi's side, not the LAN.

## The three things that must match

1. **`ROS_DOMAIN_ID`** — nodes only see peers with the same value. Default is `0`,
   which is also everyone else's default; on a shared LAN that means stray nodes
   from other projects show up in your `ros2 topic list`. This project uses **42**,
   set on both machines.
2. **`ROS_LOCALHOST_ONLY=0`** — if this is `1`, traffic never leaves the machine.
3. **`RMW_IMPLEMENTATION`** — both ends must use the same DDS vendor. Jazzy
   defaults to Fast DDS; this project standardises on **Cyclone DDS**
   (`rmw_cyclonedds_cpp`) because its interface selection is easier to control,
   which matters here (see below). The package is `ros-jazzy-rmw-cyclonedds-cpp`.

**Do not set these by hand.** They are defined once in
`ansible/group_vars/all.yml` and written to both hosts by the `ros2_env` role —
[ansible.md](ansible.md). Keeping three variables in agreement across two machines
manually is the single biggest source of wasted time in this setup, and most of
[troubleshooting.md](troubleshooting.md) is a catalogue of what happens when they
drift. To change one, change the variable and re-run the playbook.

To see what a host currently has:

```bash
ssh pi -t 'bash -lc "env | grep -E \"^ROS_|^RMW_|^CYCLONEDDS\""'
```

## The Docker-bridge problem on the dev box

This is the gotcha most likely to cost an evening, and it is specific to this setup.

The dev box carries several interfaces that are not the LAN:

```
docker0          172.17.0.1/16
br-808fe3a7a7d5  172.18.0.1/16
br-b781b3d892d2  172.19.0.1/16
tailscale0       (no IPv4)
laptop           10.8.0.3/32      # WireGuard
enp6s18          192.168.2.109/24 # ← the only one that reaches the Pi
```

By default Cyclone DDS enumerates *every* interface and picks one — often a bridge
or the VPN rather than `enp6s18`. The result is a node that advertises itself at
`172.1x.0.1` or `10.8.0.3`, an address the Pi cannot route to. Symptoms: discovery
is slow or intermittent, `ros2 topic list` shows the topic but `ros2 topic echo`
sits silent, or things work in one direction only.

> The VPN interfaces make this worse than a plain Docker setup, not better —
> `tailscale0` and `laptop` are *routable-looking* but lead somewhere the Pi is
> not. Pin explicitly; do not rely on interface ordering.

Fix it by pinning the interface. Create `~/.config/cyclonedds/cyclonedds.xml` on the dev box:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="enp6s18" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

```bash
export CYCLONEDDS_URI=file:///home/proxmox-ml5/.config/cyclonedds/cyclonedds.xml
```

The Pi has only one relevant interface, so pinning is less urgent there — but note
that interface is **`wlan0`, not `eth0`**: the Pi has no Ethernet cable attached
([hardware.md](hardware.md#networking--wi-fi-only)). Pinning it anyway is harmless
and makes the two configs symmetrical. That symmetry is exactly what the Ansible
`ros2_env` role produces: one `cyclonedds.xml.j2` template rendered per host from a
`dds_interface` variable, so the two files cannot drift apart in structure while
differing in the one field that should differ.

The rendered file lives at `~/.config/cyclonedds/cyclonedds.xml`, deliberately
*outside* the workspace: the file is per-host so it cannot be committed, and the
`workspace` role syncs the repo with `rsync --delete` — a config templated into
the synced tree would be silently deleted on the next sync, after which DDS binds
whatever interface it likes with no error anywhere.

## Static peers, if multicast is unreliable

Some switches and most Wi-Fi access points drop or rate-limit multicast — and
**the Pi is on Wi-Fi**, so this is a likely failure mode here rather than a
hypothetical one. Expect to need it, and reach for it early if discovery is flaky
between the two machines. Jazzy can skip multicast entirely:

```bash
# on the dev box
export ROS_STATIC_PEERS=192.168.2.17
# on the Pi
export ROS_STATIC_PEERS=192.168.2.109
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Try it in a shell first to confirm it is the fix. If it is, make it permanent the
same way as everything else — a `static_peer` variable in each host's `group_vars`,
picked up by the `ros2_env` role — rather than leaving an export in one `.bashrc`
that the other machine knows nothing about.

`ROS_AUTOMATIC_DISCOVERY_RANGE` takes `OFF`, `LOCALHOST`, `SUBNET` (the default),
or `SYSTEM_DEFAULT`. Combining `OFF` with explicit `ROS_STATIC_PEERS` gives fully
deterministic discovery — a good fallback when you want to eliminate the network
as a variable.

## Diagnosing

```bash
# Which nodes/topics can this machine actually see?
ros2 node list
ros2 topic list

# Full environment and middleware report
ros2 doctor --report

# Is anything publishing at all, and how fast?
ros2 topic hz /image_raw
ros2 topic bw /image_raw

# Are DDS packets crossing the wire? (default discovery port range for domain 42)
sudo tcpdump -i enp6s18 -n 'udp and portrange 7400-7500'
```

`ros2 daemon stop && ros2 daemon start` after changing any of the environment
variables above — the daemon caches discovery state and will keep reporting the
old view otherwise. This alone explains a lot of "I fixed it but nothing changed".

## Bandwidth

Raw 1280×720 RGB8 at 30 fps is roughly **83 MB/s** — well beyond what the link
should be asked to carry, and it will stall the pipeline. Always stream compressed
between the machines; see [camera.md](camera.md#image-transport).
