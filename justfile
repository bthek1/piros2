# piros2 — day-to-day commands. `just` (no args) lists them by group.
#
# Ansible recipes run from ansible/; sudo prompts on the dev box, which is why
# the deploy recipes carry --ask-become-pass (the Pi's sudo is passwordless).

set working-directory := 'ansible'

# List available recipes
default:
    @just --list --unsorted

# Reachability first: both hosts must answer before anything else is worth running
[group('provision')]
ping:
    ansible all -m ping

# Dry run — read the diff before letting a change loose
[group('provision')]
check:
    ansible-playbook site.yml --check --diff --ask-become-pass

# Provision both machines (idempotent; a correct machine reports changed=0)
[group('provision')]
deploy:
    ansible-playbook site.yml --ask-become-pass

# Provision only the Pi — no password prompt, safe to run any time
[group('provision')]
deploy-pi:
    ansible-playbook site.yml --limit robot

# Provision only the dev box
[group('provision')]
deploy-dev:
    ansible-playbook site.yml --limit dev --ask-become-pass

# Parse check without touching either host
[group('provision')]
syntax:
    ansible-playbook site.yml --syntax-check

# Push the repo to the Pi's ~/piros2 (same rsync the workspace role runs).
# The 99 MB depth model stays here: inference is dev-box-only, the Pi is a
# sensor head.
[group('sync')]
sync:
    rsync -av --delete --exclude build --exclude install --exclude log \
          --exclude src/piros2_perception/models \
          "{{ justfile_directory() }}/" pi:~/piros2/

# The three env vars that must match on both hosts, plus each host's DDS pin
[group('status')]
status:
    @echo "── dev box ──"
    @bash -lc 'printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI' || true
    @grep NetworkInterface ~/.config/cyclonedds/cyclonedds.xml || true
    @echo "── pi ──"
    @ssh pi "bash -lc 'printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI'" || true
    @ssh pi 'grep NetworkInterface ~/.config/cyclonedds/cyclonedds.xml' || true

# Topics visible from the dev box (fresh daemon so the view is not stale)
[group('status')]
topics:
    bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 daemon stop && ros2 daemon start && ros2 topic list'

# The V4L2 controls persist INSIDE the camera across processes and reboots,
# so `value` drifting from `default` is leftover state from an earlier
# session (the classic cause of black frames and 18–21 fps). Reset with
# `just camera-reset`.
# Camera state on the Pi: devices, symlink, group, every control current-vs-default
[group('status')]
camera:
    ssh pi 'v4l2-ctl --list-devices; ls -l /dev/v4l/by-id/; id -nG | tr " " "\n" | grep -x video'
    ssh pi 'v4l2-ctl -d /dev/video0 --list-ctrls'

# The baseline: all autos on, neutral image controls, gain 0, dynamic
# framerate OFF (its power-on state is 1, which trades fps for exposure in
# indoor light — camera.md#camera-state). Safe while the camera streams:
# V4L2 control writes work alongside an active capture (verified
# 2026-08-01). One --set-ctrl per call on purpose — a batched set fails
# wholesale if any control is held inactive by its auto mode. Leaves
# power_line_frequency alone; set it to the local mains (1=50Hz, 2=60Hz)
# only if indoor frames show rolling bands.
# Restore the camera's persistent V4L2 controls to the known-good baseline
[group('status')]
camera-reset:
    ssh pi 'set -e; for c in auto_exposure=3 exposure_dynamic_framerate=0 focus_automatic_continuous=1 white_balance_automatic=1 gain=0 brightness=128 contrast=128 saturation=128 sharpness=128 zoom_absolute=100 pan_absolute=0 tilt_absolute=0; do v4l2-ctl -d /dev/video0 --set-ctrl=$c; done; echo "camera baseline restored"'

# The layer below `just status`: an unreachable Pi is invisible to DDS no
# matter what the ROS env says, and the link has died twice while the OS
# ran on (networking.md#wi-fi-link-reliability). Reads association state,
# signal, this boot's ASSOC-REJECT damage count, gateway reachability and
# power-save — the numbers to quote before blaming ROS for silence.
# Wi-Fi link health on the Pi (association, signal, rejects, gateway, power-save)
[group('status')]
wifi:
    #!/usr/bin/env bash
    ssh -o BatchMode=yes -o ConnectTimeout=5 pi 'bash -s' <<'EOF' || echo "pi unreachable"
    echo "── association ──"
    sudo wpa_cli -i wlan0 status | grep -E "^(wpa_state|ssid|bssid|freq)="
    echo "── signal ──"
    sudo wpa_cli -i wlan0 signal_poll
    echo "── this boot ──"
    echo "assoc rejects: $(journalctl -b --no-pager | grep -c ASSOC-REJECT)"
    journalctl -b --no-pager | grep TEMP-DISABLED | tail -1
    echo "── gateway ──"
    gw=$(ip route show default | awk '{print $3; exit}')
    if [ -n "$gw" ]; then ping -c 3 -W 1 "$gw" | tail -2; else echo "no default route"; fi
    echo "── power-save ──"
    iw dev wlan0 get power_save 2>/dev/null || echo "unknown (iw not installed — watchdog plan P1)"
    exit 0
    EOF

# Every session recipe must leave nothing behind — window close and Ctrl-C
# both fire its EXIT trap (CLAUDE.md Conventions). This sweeps both machines
# for survivors; "clean" per host means none. Clear leftovers with
# `pkill -f` on the printed patterns (and `ssh pi 'pkill -f …'` for the Pi).
# List session processes still alive on either machine (should print clean)
[group('status')]
stragglers:
    #!/usr/bin/env bash
    pat='usb_cam|static_transform_publisher|ros2 launch|ros2 bag|image_transport/republish|piros2_vision|piros2_perception|piros2_world|rgbd_odometry|rtabmap|cameracalibrator|rviz2|rqt_image_view'
    echo "── dev box ──"
    pgrep -af "$pat" || echo "clean"
    echo "── pi ──"
    out=$(ssh -o BatchMode=yes -o ConnectTimeout=5 pi 'pgrep -af "usb_[c]am|static_transform_[p]ublisher|ros2 [l]aunch|ros2 [b]ag"'); rc=$?
    if [ $rc -eq 0 ]; then echo "$out"; elif [ $rc -eq 1 ]; then echo "clean"; else echo "pi unreachable"; fi

# Points at whatever the project's newest runnable thing is — retarget this
# as phases land. Args pass through to the underlying recipe.
# Run the latest project (currently `just world` — the composed dashboard)
[group('test')]
run *args: (world args)

# Camera on the Pi + the perception nodes here (perception.launch.py) +
# RViz on the cloud. Closing RViz stops everything. Depth runs on the GPU
# since 2026-07-30 (~13 fps in-node); the cloud updates at that pace —
# slower than the camera's 30–60 fps is the pipeline's pace, not a fault.
#
# The camera ssh carries -tt + keepalives, with stdin from /dev/null so the
# tty games stay remote: a dead link kills the local ssh in ~15 s
# (ServerAlive 5x3) instead of TCP-forever, and the forced pty means sshd
# HUPs the remote launch when the session dies — camera released, LED off,
# no orphan holding /dev/video0. Silent link deaths are caught server-side
# by sshd ClientAlive (wifi role). Same options on every camera launcher.
#
# RViz env pin: QT_QPA_PLATFORM=xcb is permanent (OGRE renders via GLX,
# which is X11-only — a Wayland Qt window can never host it). The mesa
# software-rendering stopgap for the 2026-07-28 NVIDIA driver mismatch was
# dropped 2026-07-30: the reboot landed matching 595.84 kernel + userspace,
# verified by running rviz2 on hardware GL (OpenGL 4.6, no GLX errors) —
# troubleshooting.md#rviz2-crashes-unable-to-create-the-rendering-window.
# Camera on the Pi + depth + point cloud + RViz; closing RViz stops all
[group('test')]
cloud *args:
    #!/usr/bin/env bash
    ssh -tt -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_camera camera.launch.py {{ args }}'" </dev/null &
    cam_pid=$!
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch piros2_perception perception.launch.py' &
    trap 'ssh -o BatchMode=yes -o ConnectTimeout=5 pi "pkill -f \"ros2 [l]aunch piros2_camera\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; pkill -f "ros2 [l]aunch piros2_perception" 2>/dev/null; pkill -f "piros2_perception.[d]epth_estimator" 2>/dev/null; pkill -f "[c]loud_projector" 2>/dev/null; kill %1 2>/dev/null' EXIT
    # Warm-up doubles as a health check: camera.launch.py exits nonzero when
    # the C922 is missing (pre-flight in the launch file), so if the ssh job
    # dies during these seconds, bail loudly instead of opening a viewer on
    # nothing. Same pattern in cam/edges/depth.
    for _ in $(seq 8); do
        kill -0 "$cam_pid" 2>/dev/null || { echo "camera failed to start on the Pi (see errors above) — is the Pi reachable (ping 192.168.2.17) and the C922 plugged in? Check with 'just camera'." >&2; exit 1; }
        sleep 1
    done
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && QT_QPA_PLATFORM=xcb rviz2 -d src/piros2_perception/config/perception.rviz'

# The launch file owns the symlink/framerate traps (docs/info/camera.md#running-it);
# args pass through, e.g. `just cam image_width:=640 image_height:=480`.
# The viewer subscribes to the compressed topic — raw 720p30 is ~83 MB/s and
# does not fit over the Wi-Fi.
# Camera on the Pi + rqt_image_view here; closing the viewer stops both
[group('test')]
cam *args:
    #!/usr/bin/env bash
    ssh -tt -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_camera camera.launch.py {{ args }}'" </dev/null &
    cam_pid=$!
    trap 'ssh -o BatchMode=yes -o ConnectTimeout=5 pi "pkill -f \"ros2 [l]aunch piros2_camera\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; kill %1 2>/dev/null' EXIT
    # warm-up + health check — see `cloud` for the why
    for _ in $(seq 4); do
        kill -0 "$cam_pid" 2>/dev/null || { echo "camera failed to start on the Pi (see errors above) — is the Pi reachable (ping 192.168.2.17) and the C922 plugged in? Check with 'just camera'." >&2; exit 1; }
        sleep 1
    done
    # PATH override: rqt tools use `#!/usr/bin/env python3`, and the PlatformIO
    # venv earlier in PATH shadows the system python ROS is built against —
    # docs/info/troubleshooting.md#rqt-tools-crash-with-no-module-named-yaml
    bash -lc 'source /opt/ros/jazzy/setup.bash && PATH="/usr/bin:$PATH" ros2 run rqt_image_view rqt_image_view /image_raw/compressed'

# Camera + detector on the Pi with no viewer attached — for RViz sessions or
# hz/latency measurements. Ctrl-C stops it.
[group('test')]
pipeline *args:
    ssh -t pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_vision vision.launch.py {{ args }}'"

# Camera + edge detector via the composed vision.launch.py, viewer here on
# the annotated stream; closing the viewer stops everything. Camera args pass
# through the included launch: `just edges gain:=128`
[group('test')]
edges *args:
    #!/usr/bin/env bash
    ssh -tt -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_vision vision.launch.py {{ args }}'" </dev/null &
    cam_pid=$!
    trap 'ssh -o BatchMode=yes -o ConnectTimeout=5 pi "pkill -f \"ros2 [l]aunch piros2_vision\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[e]dge_detector\"; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; kill %1 2>/dev/null' EXIT
    # warm-up + health check — see `cloud` for the why
    for _ in $(seq 6); do
        kill -0 "$cam_pid" 2>/dev/null || { echo "camera failed to start on the Pi (see errors above) — is the Pi reachable (ping 192.168.2.17) and the C922 plugged in? Check with 'just camera'." >&2; exit 1; }
        sleep 1
    done
    bash -lc 'source /opt/ros/jazzy/setup.bash && PATH="/usr/bin:$PATH" ros2 run rqt_image_view rqt_image_view /image_processed/compressed'

# Run while `just pipeline`/`just cam` is up. Print docs/info/checkerboard-8x6-25mm.svg
# at 100% first. Decompresses the stream locally onto /calib/image_raw so the
# calibrator never pulls raw over the Wi-Fi, and PATH-prefixes the GUI (env-shebang
# Python tool, PlatformIO venv trap). Save lands in /tmp/calibrationdata.tar.gz —
# see docs/info/camera.md#calibration for where the yaml goes.
#
# The calibrator's OpenCV window ignores the window-manager close button — its
# loop only exits on q/Esc or COMMIT — so the recipe watches the window via
# xwininfo and puts the node down when it disappears. QT_QPA_PLATFORM=xcb
# forces the (Qt5) window onto Xwayland: in this Wayland session it would
# otherwise be a native Wayland surface no X tool can observe. Cleanup is by
# pkill pattern, not `kill %N`: the job is a bash wrapper and killing it
# orphans the actual ros2-run grandchildren (observed leak).
# Camera calibration GUI against the live stream (needs the printed board)
[group('test')]
calibrate:
    #!/usr/bin/env bash
    # fail loudly up front rather than opening a calibrator on a dead stream
    ssh pi 'pgrep -f usb_cam_[n]ode_exe >/dev/null' || { echo "no camera running on the Pi — start 'just cam' or 'just pipeline' first" >&2; exit 1; }
    bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=/image_raw/compressed -r out:=/calib/image_raw' >/dev/null 2>&1 &
    trap 'pkill -f "/calib/[i]mage_raw" 2>/dev/null; pkill -f "camera_calibration/[c]ameracalibrator" 2>/dev/null' EXIT
    sleep 2
    bash -lc 'source /opt/ros/jazzy/setup.bash && PATH="/usr/bin:$PATH" QT_QPA_PLATFORM=xcb ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --ros-args -r image:=/calib/image_raw -r camera/set_camera_info:=/usb_cam/set_camera_info' &
    cal=$!
    seen_id=""
    # watch the wrapper PID, not pgrep — the node takes seconds to spawn and a
    # pattern match here would see nothing and fall straight through.
    # Closing the window destroys it but the next imshow recreates it within a
    # frame, so absence-polling races; the recreated window has a NEW X id,
    # and an id change (or empty) is the reliable user-closed-it signal.
    # Match only the CLIENT window — empty class "()" — because mutter also
    # names its frame window "display", frames appear a beat after the client,
    # and that reparenting churn reads as an id change on the frame.
    while kill -0 "$cal" 2>/dev/null; do
        id=$(xwininfo -root -tree 2>/dev/null | awk '/"display": \(\)/{print $1; exit}')
        if [ -n "$seen_id" ] && [ "$id" != "$seen_id" ]; then
            break
        fi
        [ -n "$id" ] && seen_id="$id"
        sleep 1
    done

# Inference runs on the GPU since 2026-07-30 (~13 fps in-node; ~3 fps on
# CPU before, and the node falls back to CPU silently — check its provider
# log line). The estimator runs under the perception venv (PyPI
# onnxruntime-gpu; colcon's hardcoded shebang would miss it) —
# src/piros2_perception/README.md.
# Camera on the Pi + neural depth here + preview viewer; closing viewer stops all
[group('test')]
depth *args:
    #!/usr/bin/env bash
    ssh -tt -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_camera camera.launch.py {{ args }}'" </dev/null &
    cam_pid=$!
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && PYTHONUNBUFFERED=1 ~/.venvs/piros2-perception/bin/python -m piros2_perception.depth_estimator --ros-args --params-file src/piros2_perception/config/perception.yaml' &
    trap 'ssh -o BatchMode=yes -o ConnectTimeout=5 pi "pkill -f \"ros2 [l]aunch piros2_camera\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; pkill -f "piros2_perception.[d]epth_estimator" 2>/dev/null; kill %1 2>/dev/null' EXIT
    # warm-up + health check — see `cloud` for the why
    for _ in $(seq 6); do
        kill -0 "$cam_pid" 2>/dev/null || { echo "camera failed to start on the Pi (see errors above) — is the Pi reachable (ping 192.168.2.17) and the C922 plugged in? Check with 'just camera'." >&2; exit 1; }
        sleep 1
    done
    bash -lc 'source /opt/ros/jazzy/setup.bash && PATH="/usr/bin:$PATH" ros2 run rqt_image_view rqt_image_view /depth/preview/compressed'

# ORB is cheap enough to eat the full camera rate on CPU (~14 ms/frame) — the
# deliberate contrast with the ~13 fps neural depth node. The detector
# subscribes the compressed stream directly, so nothing raw crosses the
# Wi-Fi and no republisher is needed.
# Camera on the Pi + ORB keypoint detector here + viewer; closing viewer stops all
[group('test')]
keypoints *args:
    #!/usr/bin/env bash
    ssh -tt -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_camera camera.launch.py {{ args }}'" </dev/null &
    cam_pid=$!
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && PYTHONUNBUFFERED=1 ros2 run piros2_world keypoint_detector --ros-args --params-file src/piros2_world/config/world.yaml' &
    trap 'ssh -o BatchMode=yes -o ConnectTimeout=5 pi "pkill -f \"ros2 [l]aunch piros2_camera\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; pkill -f "piros2_world/[k]eypoint_detector" 2>/dev/null; kill %1 2>/dev/null' EXIT
    # warm-up + health check — see `cloud` for the why
    for _ in $(seq 6); do
        kill -0 "$cam_pid" 2>/dev/null || { echo "camera failed to start on the Pi (see errors above) — is the Pi reachable (ping 192.168.2.17) and the C922 plugged in? Check with 'just camera'." >&2; exit 1; }
        sleep 1
    done
    bash -lc 'source /opt/ros/jazzy/setup.bash && PATH="/usr/bin:$PATH" ros2 run rqt_image_view rqt_image_view /keypoints/compressed'

# The whole world stack, one window (world combined plan; the rqt
# mosaic and map windows retired 2026-08-05 — everything lives in the
# orientation window now): camera on the Pi + world.launch.py here
# (depth estimator under the perception venv, keypoint detector,
# dashboard, cloud projector, cloud mapper) + one RViz window: TF axes,
# live cloud and the accumulated map panorama in the 3D scene (each
# toggleable in Displays), with raw camera / keypoints / depth / stats
# image panels docked alongside. Pan the camera and the axes/cloud turn
# while the map paints. /keypoint_detector/reset re-zeros the
# orientation, /cloud_mapper/clear the map. Closing RViz stops
# everything.
[group('test')]
world *args:
    #!/usr/bin/env bash
    ssh -tt -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_camera camera.launch.py {{ args }}'" </dev/null &
    cam_pid=$!
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && PYTHONUNBUFFERED=1 ros2 launch piros2_world world.launch.py' &
    trap 'pkill -f "ros2 [l]aunch piros2_world" 2>/dev/null; pkill -f "piros2_perception.[d]epth_estimator" 2>/dev/null; pkill -f "piros2_world/[k]eypoint_detector" 2>/dev/null; pkill -f "piros2_world/[d]ashboard" 2>/dev/null; pkill -f "[c]loud_projector" 2>/dev/null; pkill -f "piros2_world/[c]loud_mapper" 2>/dev/null; pkill -f "piros2_world.[t]sdf_mesher" 2>/dev/null; pkill -f "[r]viz2 -d src/piros2_world/config" 2>/dev/null; ssh -o BatchMode=yes -o ConnectTimeout=5 pi "pkill -f \"ros2 [l]aunch piros2_camera\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; kill %1 2>/dev/null' EXIT
    # warm-up + health check — see `cloud` for the why
    for _ in $(seq 8); do
        kill -0 "$cam_pid" 2>/dev/null || { echo "camera failed to start on the Pi (see errors above) — is the Pi reachable (ping 192.168.2.17) and the C922 plugged in? Check with 'just camera'." >&2; exit 1; }
        sleep 1
    done
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && QT_QPA_PLATFORM=xcb rviz2 -d src/piros2_world/config/world.rviz'

# The orientation compass in 3D (world 3D plan P1): camera on the Pi +
# keypoint detector here (it estimates rotation and broadcasts
# odom → base_link) + RViz with fixed frame odom — the axes tilt and pan
# live as the camera is moved by hand; /keypoint_detector/reset re-zeros.
# Closing RViz stops everything.
[group('test')]
orient *args:
    #!/usr/bin/env bash
    ssh -tt -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_camera camera.launch.py {{ args }}'" </dev/null &
    cam_pid=$!
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run piros2_world keypoint_detector --ros-args --params-file src/piros2_world/config/world.yaml' &
    trap 'ssh -o BatchMode=yes -o ConnectTimeout=5 pi "pkill -f \"ros2 [l]aunch piros2_camera\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; pkill -f "piros2_world/[k]eypoint_detector" 2>/dev/null; kill %1 2>/dev/null' EXIT
    # warm-up + health check — see `cloud` for the why
    for _ in $(seq 8); do
        kill -0 "$cam_pid" 2>/dev/null || { echo "camera failed to start on the Pi (see errors above) — is the Pi reachable (ping 192.168.2.17) and the C922 plugged in? Check with 'just camera'." >&2; exit 1; }
        sleep 1
    done
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && QT_QPA_PLATFORM=xcb rviz2 -d src/piros2_world/config/world.rviz'

# No Pi needed: plays a bag ONCE (looping would teleport the odometry back
# to the start pose and wreck the map), decompresses it to /image_raw, and
# runs mapping.launch.py — depth estimator + RTAB-Map's rgbd_odometry +
# rtabmap — with rtabmap_viz watching the map build. Closing the viz stops
# everything. Needs ros-jazzy-rtabmap-ros (`just deploy-dev`). The default
# bag is the static desk bag (plumbing check); record a sweep with
# `just record 45 sweep1` (fix exposure first — camera.md#v4l2-controls)
# and pass it: `just map bags/sweep1`.
# Replay a bag through depth + RTAB-Map + map viewer, all on the dev box
[group('test')]
map bag='bags/static1':
    #!/usr/bin/env bash
    cd "{{ justfile_directory() }}"
    bash -lc "source /opt/ros/jazzy/setup.bash && ros2 bag play '{{ bag }}'" >/dev/null 2>&1 &
    bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=/image_raw/compressed -r out:=/image_raw' >/dev/null 2>&1 &
    bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch piros2_perception mapping.launch.py' &
    trap 'pkill -f "ros2 bag [p]lay" 2>/dev/null; pkill -f "out:=/[i]mage_raw" 2>/dev/null; pkill -f "ros2 [l]aunch piros2_perception" 2>/dev/null; pkill -f "piros2_perception.[d]epth_estimator" 2>/dev/null; pkill -f "rgbd_[o]dometry" 2>/dev/null; pkill -f "rtabmap_[s]lam" 2>/dev/null' EXIT
    sleep 5
    # Same display pins as rviz2: xcb because the GL view is X11-only, mesa
    # software rendering until the NVIDIA userspace/module mismatch is
    # rebooted away — troubleshooting.md#rviz2-crashes-unable-to-create-the-rendering-window-glxcontext-100-tries
    # Subscribes the same RGB-D pair as the SLAM nodes so the viz shows the
    # camera + depth panels, not just the odometry track. `|| true`: Ctrl-C
    # gives rtabmap_viz a nonzero exit (255) that would otherwise make the
    # whole recipe report failure after a perfectly good session.
    bash -lc 'source /opt/ros/jazzy/setup.bash && QT_QPA_PLATFORM=xcb ros2 run rtabmap_viz rtabmap_viz --ros-args -p frame_id:=base_link -p subscribe_depth:=true -p approx_sync:=false -r rgb/image:=/image_raw -r rgb/camera_info:=/camera_info -r depth/image:=/depth' || true

# Run while `just pipeline` or `just cam` is up. Records the compressed
# stream (raw 720p is ~83 MB/s — neither the SD card nor the Wi-Fi wants
# that), plus camera_info and the latched static transforms, then pulls the
# bag back here into bags/ (git-ignored). Name each bag for its purpose
# (`just record 45 sweep1`) — session1 is the milestone-6 bag, whose
# camera_info predates the P0 intrinsics and is all zeros.
# Record a camera session on the Pi and fetch it to bags/<name>
[group('test')]
record secs='20' name='session1':
    ssh pi 'pgrep -f usb_cam_[n]ode_exe >/dev/null' || { echo "no camera running on the Pi — start 'just cam' or 'just pipeline' first (an empty bag would record silently otherwise)" >&2; exit 1; }
    ssh -o BatchMode=yes -o ConnectTimeout=5 pi "bash -lc 'source /opt/ros/jazzy/setup.bash && mkdir -p ~/bags && rm -rf ~/bags/{{ name }} && timeout -s INT {{ secs }} ros2 bag record -o ~/bags/{{ name }} /image_raw/compressed /camera_info /tf_static'" || true
    rsync -a pi:~/bags/{{ name }} "{{ justfile_directory() }}/bags/"
    bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 bag info "{{ justfile_directory() }}/bags/{{ name }}"'

# No Pi needed: loops the bag, decompresses it back to /image_raw
# (image_transport republish — parameters, not positional args, in Jazzy),
# runs the edge detector against it locally, and shows the annotated result
# in rqt_image_view. Closing the viewer stops everything.
# Replay bags/session1 through the edge detector + viewer, all on the dev box
[group('test')]
replay bag='bags/session1':
    #!/usr/bin/env bash
    cd "{{ justfile_directory() }}"
    bash -lc "source /opt/ros/jazzy/setup.bash && ros2 bag play --loop '{{ bag }}'" >/dev/null 2>&1 &
    bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=/image_raw/compressed -r out:=/image_raw' >/dev/null 2>&1 &
    bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && PYTHONUNBUFFERED=1 ros2 run piros2_vision edge_detector' &
    # pkill by pattern, not `kill %N` — the jobs are bash wrappers and killing
    # them orphans the ros2-run grandchildren, which then haunt the terminal
    # whenever a live camera feeds them (observed twice)
    trap 'pkill -f "ros2 bag [p]lay" 2>/dev/null; pkill -f "out:=/[i]mage_raw" 2>/dev/null; pkill -f "piros2_vision/[e]dge_detector" 2>/dev/null' EXIT
    sleep 3
    bash -lc 'source /opt/ros/jazzy/setup.bash && PATH="/usr/bin:$PATH" ros2 run rqt_image_view rqt_image_view /image_processed/compressed'

# (2>&1 on the listener because ROS logs go to stderr)
# Milestone 1 in one command: our talker here, our listener on the Pi
[group('test')]
hello:
    #!/usr/bin/env bash
    bash -lc 'source /opt/ros/jazzy/setup.bash && source "{{ justfile_directory() }}/install/setup.bash" && timeout 20 ros2 run piros2_hello talker >/dev/null 2>&1' &
    sleep 3
    ssh -o BatchMode=yes -o ConnectTimeout=5 pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && timeout 10 ros2 run piros2_hello listener' 2>&1" | grep heard
    wait

# Milestone 0 in one command: talker on the Pi, one message received here
[group('test')]
chatter:
    ssh -f pi "bash -lc 'source /opt/ros/jazzy/setup.bash && timeout 30 ros2 run demo_nodes_cpp talker >/dev/null 2>&1'"
    bash -lc 'source /opt/ros/jazzy/setup.bash && timeout 25 ros2 topic echo /chatter std_msgs/msg/String --once'

# Restart the ROS daemons on both machines — after ANY ROS_*/DDS env change
[group('test')]
daemon-restart:
    bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 daemon stop && ros2 daemon start'
    ssh -o BatchMode=yes -o ConnectTimeout=5 pi "bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 daemon stop && ros2 daemon start'"

# Checksum-pinned and idempotent: verifies and skips when the file is
# already good. Weights are git-ignored; the depth node resolves them from
# this path — src/piros2_perception/README.md.
# Fetch Depth Anything V2 Small ONNX weights (~99 MB, HuggingFace)
[group('build')]
fetch-model:
    #!/usr/bin/env bash
    set -euo pipefail
    file="{{ justfile_directory() }}/src/piros2_perception/models/depth_anything_v2_small.onnx"
    sha="afb6a5c28f3b6bf1618c6e43f02073ef9dfdc70e937502d51603e57b0a1df10c"
    if [ -f "$file" ] && echo "$sha  $file" | sha256sum --check --quiet 2>/dev/null; then
        echo "model present and verified"
        exit 0
    fi
    mkdir -p "$(dirname "$file")"
    curl -L --progress-bar -o "$file" \
        "https://huggingface.co/onnx-community/depth-anything-v2-small/resolve/main/onnx/model.onnx"
    echo "$sha  $file" | sha256sum --check

# colcon build on the dev box (the Pi builds via `just deploy-pi`)
[group('build')]
build:
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'

# colcon test + aggregated results (linter tests today — flake8/pep257 per package)
[group('build')]
test *args:
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && colcon test {{ args }} && colcon test-result --verbose'

# ---------------------------------------------------------------- recon

# Idempotent: skips when the sequence directory already exists. TUM RGB-D
# sequences are the known-good captures the fusion tooling learns on —
# calibrated K, ground-truth poses (world fusion plan P1).
# Download a TUM RGB-D sequence into datasets/ (~550 MB)
[group('recon')]
fetch-tum sequence='rgbd_dataset_freiburg1_desk':
    #!/usr/bin/env bash
    set -euo pipefail
    dir="{{ justfile_directory() }}/datasets"
    if [ -d "$dir/{{ sequence }}" ]; then
        echo "{{ sequence }} already present"
        exit 0
    fi
    mkdir -p "$dir"
    curl -L --progress-bar -o "$dir/{{ sequence }}.tgz" \
        "https://cvg.cit.tum.de/rgbd/dataset/freiburg1/{{ sequence }}.tgz"
    tar xzf "$dir/{{ sequence }}.tgz" -C "$dir"
    rm "$dir/{{ sequence }}.tgz"

# Runs under the perception venv (open3d is PyPI-only, the repo's documented
# escape hatch) with the workspace overlay sourced so piros2_world.se3
# imports. Mesh lands in meshes/; pass -- flags through to the script
# (e.g. `just fuse-tum datasets/... --voxel-size 0.004`).
# TSDF-fuse a TUM sequence into a mesh (world fusion plan P1)
[group('recon')]
fuse-tum sequence='datasets/rgbd_dataset_freiburg1_desk' *args:
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ~/.venvs/piros2-perception/bin/python tools/recon/fuse_tum.py {{ sequence }} {{ args }}'

# Depth is derived (regenerated by the ONNX model at export time), poses are
# a separate rewritable file (rotation-only here; P4 overwrites it with
# RTAB-Map's) — the capture layer argument in executable form (plan P3).
# Export a bag to a TUM-layout keyframe dir in captures/<name>
[group('recon')]
export-capture bag name *args:
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ~/.venvs/piros2-perception/bin/python tools/recon/export_capture.py {{ bag }} {{ name }} {{ args }}'

# The P4 experiment in one recipe: rotation-only groundtruth.txt gives the
# panorama TSDF; pass --trajectory <file> to re-fuse the same pixels under
# RTAB-Map's optimised 6-DoF poses and watch the walls lock into place.
# TSDF-fuse an exported capture into a mesh (world fusion plan P4)
[group('recon')]
fuse-capture name *args:
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ~/.venvs/piros2-perception/bin/python tools/recon/fuse_capture.py captures/{{ name }} {{ args }}'

# RANSAC planes off a fused mesh, gravity from the floor normal, walls
# snapped to a Manhattan frame; writes <mesh>_room.json + <mesh>.glb next
# to the mesh and prints the measurable spans (world fusion plan P5).
# Extract the structural room layer from a fused mesh
[group('recon')]
room-layer mesh *args:
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ~/.venvs/piros2-perception/bin/python tools/recon/room_layer.py {{ mesh }} {{ args }}'

# The P4 pose path without the GUI: replay the bag through depth + RTAB-Map
# (mapping.launch.py), then dump the trajectory with rtabmap-report into
# bags/<name>_odom.txt / _slam.txt (TUM form + an id column; poses are
# base_link, in replay wall time — `just fuse-capture ... --poses-frame
# base` converts the frame and removes the clock offset). rtabmap-export
# aborts on these databases ("no odometry poses"); rtabmap-report is the
# tool that works against 0.22.1.
# Headless RTAB-Map over a bag; poses land next to the bag
[group('recon')]
map-headless bag:
    #!/usr/bin/env bash
    cd "{{ justfile_directory() }}"
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    cleanup() {
        pkill -f "out:=/[i]mage_raw" 2>/dev/null
        pkill -f "ros2 [l]aunch piros2_perception" 2>/dev/null
        pkill -f "piros2_perception.[d]epth_estimator" 2>/dev/null
        pkill -f "rgbd_[o]dometry" 2>/dev/null
        pkill -f "rtabmap_[s]lam" 2>/dev/null
    }
    trap cleanup EXIT
    ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=/image_raw/compressed -r out:=/image_raw > /dev/null 2>&1 &
    ros2 launch piros2_perception mapping.launch.py > /tmp/map-headless.log 2>&1 &
    sleep 8   # model load + rtabmap init (-d wipes the old db)
    ros2 bag play "{{ bag }}" > /dev/null 2>&1
    sleep 5   # let the last synced pairs flush
    cleanup; sleep 2
    quality=$(grep -c "Odom: quality" /tmp/map-headless.log || true)
    echo "odometry updates: $quality (log: /tmp/map-headless.log)"
    (cd "$(dirname "{{ bag }}")" && rtabmap-report --poses_raw ~/.ros/rtabmap.db)
    mv ~/.ros/rtabmap_odom.txt "{{ bag }}_odom.txt" 2>/dev/null
    mv ~/.ros/rtabmap_slam.txt "{{ bag }}_slam.txt" 2>/dev/null
    ls -la "{{ bag }}"_*.txt

# Meshes are offline artifacts — `just world` deliberately never publishes
# them (live session = dashboard; reconstruction stays offline), so this
# window is how you look at one. Wayland caveat: Open3D's viewer needs
# X11, hence the unset (troubleshooting.md).
# View a fused mesh interactively (default: the newest in meshes/)
[group('recon')]
view-mesh mesh='':
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ justfile_directory() }}"
    mesh="{{ mesh }}"
    if [ -z "$mesh" ]; then
        mesh=$(ls -t meshes/*.ply | head -1)
        echo "viewing newest: $mesh"
    fi
    env -u WAYLAND_DISPLAY XDG_SESSION_TYPE=x11 ~/.venvs/piros2-perception/bin/python tools/recon/view_mesh.py "$mesh"
