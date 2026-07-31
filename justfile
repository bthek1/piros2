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

# Camera state on the Pi: devices, group membership, the serial-keyed symlink
[group('status')]
camera:
    ssh pi 'v4l2-ctl --list-devices; ls -l /dev/v4l/by-id/; id -nG | tr " " "\n" | grep -x video'

# Points at whatever the project's newest runnable thing is — retarget this
# as phases land. Args pass through to the underlying recipe.
# Run the latest project (currently `just cloud` — 3D point cloud, perception P2)
[group('test')]
run *args: (cloud args)

# Camera on the Pi + the perception nodes here (perception.launch.py) +
# RViz on the cloud. Closing RViz stops everything. Depth runs on the GPU
# since 2026-07-30 (~13 fps in-node); the cloud updates at that pace —
# slower than the camera's 30 fps is the pipeline's pace, not a fault.
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
    ssh pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_camera camera.launch.py {{ args }}'" &
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch piros2_perception perception.launch.py' &
    trap 'ssh pi "pkill -f \"ros2 [l]aunch piros2_camera\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; pkill -f "ros2 [l]aunch piros2_perception" 2>/dev/null; pkill -f "piros2_perception.[d]epth_estimator" 2>/dev/null; pkill -f "[c]loud_projector" 2>/dev/null; kill %1 2>/dev/null' EXIT
    sleep 8
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && QT_QPA_PLATFORM=xcb rviz2 -d src/piros2_perception/config/perception.rviz'

# The launch file owns the symlink/framerate traps (docs/info/camera.md#running-it);
# args pass through, e.g. `just cam image_width:=640 image_height:=480`.
# The viewer subscribes to the compressed topic — raw 720p30 is ~83 MB/s and
# does not fit over the Wi-Fi.
# Camera on the Pi + rqt_image_view here; closing the viewer stops both
[group('test')]
cam *args:
    #!/usr/bin/env bash
    ssh pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_camera camera.launch.py {{ args }}'" &
    trap 'ssh pi "pkill -f \"ros2 [l]aunch piros2_camera\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; kill %1 2>/dev/null' EXIT
    sleep 4
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
    ssh pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_vision vision.launch.py {{ args }}'" &
    trap 'ssh pi "pkill -f \"ros2 [l]aunch piros2_vision\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[e]dge_detector\"; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; kill %1 2>/dev/null' EXIT
    sleep 6
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
    ssh pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_camera camera.launch.py {{ args }}'" &
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && PYTHONUNBUFFERED=1 ~/.venvs/piros2-perception/bin/python -m piros2_perception.depth_estimator --ros-args --params-file src/piros2_perception/config/perception.yaml' &
    trap 'ssh pi "pkill -f \"ros2 [l]aunch piros2_camera\"; pkill -f usb_cam_[n]ode_exe; pkill -f \"[s]tatic_transform_publisher\"" 2>/dev/null; pkill -f "piros2_perception.[d]epth_estimator" 2>/dev/null; kill %1 2>/dev/null' EXIT
    sleep 6
    bash -lc 'source /opt/ros/jazzy/setup.bash && PATH="/usr/bin:$PATH" ros2 run rqt_image_view rqt_image_view /depth/preview/compressed'

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
    ssh pi "bash -lc 'source /opt/ros/jazzy/setup.bash && mkdir -p ~/bags && rm -rf ~/bags/{{ name }} && timeout -s INT {{ secs }} ros2 bag record -o ~/bags/{{ name }} /image_raw/compressed /camera_info /tf_static'" || true
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
    ssh pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && timeout 10 ros2 run piros2_hello listener' 2>&1" | grep heard
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
    ssh pi "bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 daemon stop && ros2 daemon start'"

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
