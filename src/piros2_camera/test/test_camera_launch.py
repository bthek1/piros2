"""
Unit tests for camera.launch.py's pre-flight helpers.

The launch file is not an importable package module — it is installed as a
data file — so it is loaded here by path, anchored on __file__ (the same
CWD-independence rule the linter tests follow). The busy-device scan is
pure filesystem inspection, so a fake /proc tree in tmp_path exercises it
without touching a real camera.
"""

import importlib.util
import os
import pathlib

_LAUNCH_PATH = (
    pathlib.Path(__file__).resolve().parent.parent
    / 'launch' / 'camera.launch.py')

_spec = importlib.util.spec_from_file_location('camera_launch', _LAUNCH_PATH)
camera_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(camera_launch)


def _fake_proc(tmp_path, pid, fd_targets, cmdline='usb_cam_node_exe\0'):
    """Build /proc/<pid> with fd symlinks and a cmdline file."""
    proc_pid = tmp_path / str(pid)
    fd_dir = proc_pid / 'fd'
    fd_dir.mkdir(parents=True)
    for i, target in enumerate(fd_targets):
        os.symlink(target, fd_dir / str(i))
    (proc_pid / 'cmdline').write_bytes(cmdline.encode())


def test_free_device_has_no_holders(tmp_path):
    device = tmp_path / 'video0'
    device.touch()
    _fake_proc(tmp_path, 101, [tmp_path / 'unrelated'])
    (tmp_path / 'not-a-pid').mkdir()  # e.g. /proc/self — must be skipped
    assert camera_launch._device_holders(
        str(device), proc=str(tmp_path)) == []


def test_holder_is_named_with_pid_and_cmdline(tmp_path):
    device = tmp_path / 'video0'
    device.touch()
    _fake_proc(tmp_path, 101, [tmp_path / 'unrelated', device],
               cmdline='usb_cam_node_exe\0--ros-args\0')
    holders = camera_launch._device_holders(str(device), proc=str(tmp_path))
    assert holders == ['101 usb_cam_node_exe --ros-args']


def test_holder_via_symlinked_device_path(tmp_path):
    # The launch resolves the by-id symlink, but a holder may have opened
    # either name — realpath on both sides must still match them up.
    device = tmp_path / 'video0'
    device.touch()
    by_id = tmp_path / 'by-id-link'
    by_id.symlink_to(device)
    _fake_proc(tmp_path, 202, [device])
    holders = camera_launch._device_holders(str(by_id), proc=str(tmp_path))
    assert len(holders) == 1 and holders[0].startswith('202 ')


def test_own_process_is_not_a_holder(tmp_path):
    device = tmp_path / 'video0'
    device.touch()
    _fake_proc(tmp_path, os.getpid(), [device])
    assert camera_launch._device_holders(
        str(device), proc=str(tmp_path)) == []
