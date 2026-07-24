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

# Push the repo to the Pi's ~/piros2 (same rsync the workspace role runs)
[group('sync')]
sync:
    rsync -av --delete --exclude build --exclude install --exclude log \
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

# The launch file owns the symlink/framerate traps (docs/camera.md#running-it);
# args pass through, e.g. `just cam image_width:=640 image_height:=480`.
# The viewer subscribes to the compressed topic — raw 720p30 is ~83 MB/s and
# does not fit over the Wi-Fi.
# Camera on the Pi + rqt_image_view here; closing the viewer stops both
[group('test')]
cam *args:
    #!/usr/bin/env bash
    ssh pi "bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/piros2/install/setup.bash && ros2 launch piros2_camera camera.launch.py {{ args }}'" &
    trap 'ssh pi "pkill -f \"ros2 [l]aunch piros2_camera\"; pkill -f usb_cam_[n]ode_exe" 2>/dev/null; kill %1 2>/dev/null' EXIT
    sleep 4
    # PATH override: rqt tools use `#!/usr/bin/env python3`, and the PlatformIO
    # venv earlier in PATH shadows the system python ROS is built against —
    # docs/troubleshooting.md#rqt-tools-crash-with-no-module-named-yaml
    bash -lc 'source /opt/ros/jazzy/setup.bash && PATH="/usr/bin:$PATH" ros2 run rqt_image_view rqt_image_view /image_raw/compressed'

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

# colcon build on the dev box (the Pi builds via `just deploy-pi`)
[group('build')]
build:
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'

# colcon test + aggregated results (linter tests today — flake8/pep257 per package)
[group('build')]
test *args:
    bash -lc 'cd "{{ justfile_directory() }}" && source /opt/ros/jazzy/setup.bash && colcon test {{ args }} && colcon test-result --verbose'
