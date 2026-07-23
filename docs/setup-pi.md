# Setting up the Pi (ROS 2 in Docker)

## Why not a native apt install

The obvious move — `apt install ros-jazzy-ros-base` on the Pi — does not work, and
it is worth understanding why before reaching for a workaround.

ROS 2 publishes binaries per platform tier. Jazzy's Tier 1 platform is **Ubuntu
24.04 noble**. Debian 12 bookworm, which Raspberry Pi OS is built on, is Tier 3:
supported in the sense that it is expected to build from source, but **no binary
packages are published for it**.

This is directly checkable rather than a matter of opinion. `packages.ros.org` does
serve a `bookworm` suite, which is misleading — it exists, it returns HTTP 200, and
`arm64` is in its architecture list. But its contents are only bootstrap tooling:

```bash
curl -s http://packages.ros.org/ros2/ubuntu/dists/bookworm/main/binary-arm64/Packages.gz \
  | gunzip | grep -c '^Package: ros-jazzy'
# → 0
```

Everything in that suite is `python3-rosdep`, `python3-vcstool`, `python3-rosdistro`,
`ros2-apt-source` and friends — the tools you would use to *build* ROS, not ROS itself.
(Verified 2026-07-23.)

That leaves four options:

| Option | Verdict |
| --- | --- |
| **Docker (`ros:jazzy-ros-base`)** | **Chosen.** The image publishes a `linux/arm64` manifest, so it runs natively on the Pi 5 with no emulation. Stock ROS environment, host OS untouched. |
| Build ROS 2 from source | Works, but several hours of compilation on 4 cores, and every future update repeats it. Not worth it for a learning project. |
| Reflash the Pi with Ubuntu 24.04 for arm64 | Perfectly valid and gives a native install. Rejected here because it discards the working Raspberry Pi OS setup (vendor kernel, camera stack, GPIO tooling). |
| RoboStack (conda/pixi ROS packages) | Viable, avoids Docker. Rejected as it is a less common path with fewer troubleshooting references. |

The container runs with `--network host`, so from the LAN's point of view the ROS
nodes inside it behave exactly like nodes running on the Pi directly.

## 1. Install Docker on the Pi

Docker is not currently installed.

```bash
ssh pi
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER"
```

Log out and back in for the group change to apply, then confirm:

```bash
ssh pi 'docker run --rm hello-world'
```

## 2. Define the image

The stock `ros:jazzy-ros-base` image has no camera driver, so add one. Create
`docker/Dockerfile` on the Pi (or in this repo and copy it over):

```dockerfile
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-usb-cam \
        ros-jazzy-v4l2-camera \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-camera-calibration-parsers \
        ros-jazzy-rqt-image-view \
        v4l-utils \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Source ROS for every interactive shell in the container.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ws
CMD ["bash"]
```

`ros-jazzy-image-transport-plugins` is what gives you `compressed` topics — without
it, streaming raw frames to the dev box will saturate the link. See
[camera.md](camera.md).

## 3. Compose file

`docker/compose.yaml`:

```yaml
services:
  ros:
    build: .
    image: piros2:jazzy
    container_name: piros2
    # Host networking is required for DDS discovery to reach the LAN.
    network_mode: host
    ipc: host
    devices:
      - /dev/video0:/dev/video0
    volumes:
      - ../:/ws
    environment:
      - ROS_DOMAIN_ID=42
      - ROS_LOCALHOST_ONLY=0
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    stdin_open: true
    tty: true
    restart: unless-stopped
```

Notes on the non-obvious settings:

- **`network_mode: host`** — bridge networking breaks DDS. Discovery relies on
  multicast and on peers advertising reachable addresses; behind a NAT bridge the
  container advertises an address the dev box cannot route to. This is the single
  most common cause of "the topic exists on the Pi but the dev box sees nothing".
- **`ipc: host`** — lets DDS use shared memory between containers/processes on the
  Pi. Without it you get a stream of shared-memory warnings and a silent fallback
  to the network path, even for node pairs on the same machine.
- **`devices:`** rather than `privileged: true` — grants exactly `/dev/video0` and
  nothing else. Only mount `/dev/video1` if you specifically need UVC metadata; it
  is not a capture device.
- **`volumes: ../:/ws`** — the repo is mounted rather than copied, so you edit on
  the dev box (over SSHFS, `rsync`, or a git push/pull loop) and build inside the
  container.

## 4. Bring it up

```bash
ssh pi
cd ~/piros2/docker
docker compose build
docker compose up -d
docker compose exec ros bash
```

Inside the container:

```bash
ros2 topic list
v4l2-ctl --list-devices        # should show the C922 on /dev/video0
```

## 5. Verify across the network

With the container running on the Pi:

```bash
# on the Pi, inside the container
ros2 run demo_nodes_cpp talker

# on the dev box
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=42 ros2 topic echo /chatter
```

If nothing arrives, do not start changing the container — the problem is almost
certainly DDS discovery, and [networking.md](networking.md) covers it.

## Keeping the workspace in sync

The repo lives on the dev box and is mounted into the container on the Pi, so the
two copies need to be kept in step. Simplest reliable option:

```bash
# from the dev box
rsync -av --delete --exclude build --exclude install --exclude log \
      ~/piros2/ pi:~/piros2/
```

Or push/pull through the GitHub remote (`git@github.com:bthek1/piros2.git`) if you
prefer every sync to be a commit.
