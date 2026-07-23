# Networking & DDS discovery

ROS 2 has no master process. Nodes find each other by broadcasting on the local
network, which is elegant when it works and opaque when it does not. This page
covers what needs to be true for the Pi and the dev box to see each other.

## The topology

```
  dev box "test"                        Raspberry Pi 5
  192.168.2.106/24  ──── eth2 ──── LAN ──── 192.168.2.17
  Ubuntu 24.04                          Ubuntu 24.04 (planned — see setup-pi.md)
  ROS 2 Jazzy, native apt               ROS 2 Jazzy, native apt
  rviz2, rqt, editor                    camera nodes
```

Both ends run ROS directly on the host, so there is no container boundary to
reason about — a node on the Pi is an ordinary process binding ordinary sockets.

Both are on the same `/24` subnet with no router in between, so multicast
discovery works. Latency is ~3.6 ms.

## The three things that must match

1. **`ROS_DOMAIN_ID`** — nodes only see peers with the same value. Default is `0`,
   which is also everyone else's default; on a shared LAN that means stray nodes
   from other projects show up in your `ros2 topic list`. This project uses **42**,
   set on both machines.
2. **`ROS_LOCALHOST_ONLY=0`** — if this is `1`, traffic never leaves the machine.
3. **`RMW_IMPLEMENTATION`** — both ends must use the same DDS vendor. Jazzy
   defaults to Fast DDS; this project standardises on **Cyclone DDS**
   (`rmw_cyclonedds_cpp`) because its interface selection is easier to control,
   which matters here (see below). Install it on the dev box with
   `sudo apt install ros-jazzy-rmw-cyclonedds-cpp`.

Set on both hosts:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**Keeping these three in agreement by hand is the single biggest source of wasted
time in this setup**, and it is why the project provisions both machines from one
Ansible variable file rather than two `.bashrc` files — [ansible.md](ansible.md).
Most of [troubleshooting.md](troubleshooting.md) is a catalogue of what happens
when they drift.

## The Docker-bridge problem on the dev box

This is the gotcha most likely to cost an evening, and it is specific to this setup.

The dev box has about eleven Docker bridge interfaces:

```
docker0          172.17.0.1/16
br-567a5cd28e34  172.18.0.1/16
br-65a7afc8e318  172.19.0.1/16
...              through 172.26.0.1/16
```

By default Cyclone DDS enumerates *every* interface and picks one — often one of
the bridges rather than `eth2`. The result is a node that advertises itself at
`172.2x.0.1`, an address the Pi cannot route to. Symptoms: discovery is slow or
intermittent, `ros2 topic list` shows the topic but `ros2 topic echo` sits silent,
or things work in one direction only.

Fix it by pinning the interface. Create `~/piros2/config/cyclonedds.xml` on the dev box:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="eth2" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

```bash
export CYCLONEDDS_URI=file:///home/bthek1/piros2/config/cyclonedds.xml
```

The Pi does not need this — it has a single relevant interface — but pinning it
there too is harmless and makes the two configs symmetrical. That symmetry is
exactly what the Ansible `ros2_env` role produces: one `cyclonedds.xml.j2` template
rendered per host from a `dds_interface` variable, so the two files cannot drift
apart in structure while differing in the one field that should differ.

## Static peers, if multicast is unreliable

Some switches and most Wi-Fi access points drop or rate-limit multicast. If
discovery is flaky on wireless but fine on the wire, that is the cause. Jazzy can
skip multicast entirely:

```bash
# on the dev box
export ROS_STATIC_PEERS=192.168.2.17
# on the Pi
export ROS_STATIC_PEERS=192.168.2.106
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

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
sudo tcpdump -i eth2 -n 'udp and portrange 7400-7500'
```

`ros2 daemon stop && ros2 daemon start` after changing any of the environment
variables above — the daemon caches discovery state and will keep reporting the
old view otherwise. This alone explains a lot of "I fixed it but nothing changed".

## Bandwidth

Raw 1280×720 RGB8 at 30 fps is roughly **83 MB/s** — well beyond what the link
should be asked to carry, and it will stall the pipeline. Always stream compressed
between the machines; see [camera.md](camera.md#image-transport).
