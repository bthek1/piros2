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

## The dev box cannot see topics published on the Pi

The most common failure in this setup. Work through in order:

1. **Domain ID mismatch.** `echo $ROS_DOMAIN_ID` on both — must both be `42`.
2. **Stale daemon.** `ros2 daemon stop && ros2 daemon start`. The daemon caches
   discovery state and will keep reporting the pre-fix view indefinitely.
3. **Container not on host networking.** `docker inspect piros2 | grep NetworkMode`
   must show `host`. Bridge networking makes the container advertise an
   unroutable address.
4. **Wrong interface picked on the dev box.** Eleven Docker bridges compete with
   `eth2`; DDS may bind to `172.2x.0.1`. Pin the interface —
   [networking.md](networking.md#the-docker-bridge-problem-on-the-dev-box).
5. **RMW mismatch.** `echo $RMW_IMPLEMENTATION` on both. Fast DDS and Cyclone DDS
   do not interoperate for discovery in practice.
6. **`ROS_LOCALHOST_ONLY=1`** left set somewhere.

Confirm packets are actually crossing:

```bash
sudo tcpdump -i eth2 -n 'udp and portrange 7400-7500'
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

## `/dev/video0` not found inside the container

- Is it in the compose file's `devices:` list?
- Did the camera enumerate on the host? `ssh pi 'ls -l /dev/video0'`
- Was the device re-enumerated after a replug? The node numbers can shift.
  `ssh pi 'v4l2-ctl --list-devices'` and look for the `C922 Pro Stream Webcam`
  block. For a stable path, use the `/dev/v4l/by-id/` symlink instead of `/dev/video0`.

## Camera opens but every frame is black or green

- Another process already has the device. `ssh pi 'sudo fuser -v /dev/video0'`.
  UVC allows only one capture client at a time.
- The driver is pointed at `/dev/video1`, which is the UVC **metadata** node, not a
  capture device.
- Exposure was manually set far too low — see
  [camera.md](camera.md#v4l2-controls).

## Frame rate far below what was requested

- The requested resolution/format combination is not one the camera offers.
  `v4l2-ctl -d /dev/video0 --list-formats-ext` is authoritative; drivers silently
  fall back rather than erroring.
- YUYV was requested instead of MJPG. Uncompressed capture is USB-bandwidth-bound
  and will not reach 30 fps above 640×480.
- The subscriber is on the raw topic across the network. 83 MB/s does not fit;
  the publisher blocks. Use `compressed`.
- MJPG decode is saturating a core. Check with `top` inside the container.

## RViz image display is frozen or stuttering

Set the Image display's **Transport Hint** to `compressed`. Left at `raw`, RViz
subscribes to the uncompressed topic across the LAN.

## `colcon build` fails on a missing dependency

```bash
cd ~/piros2
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` itself errors about an unknown key, the package is not in the rosdistro
index — install it manually with `apt` and add it to the Dockerfile so the
container image stays reproducible.

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

## Shared-memory warnings from Cyclone DDS

Add `ipc: host` to the compose service. Without it the container cannot use the
host's shared-memory segments and DDS falls back to the loopback network path —
functional, but noisy and slower.
