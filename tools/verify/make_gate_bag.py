"""
Cut a recorded hand sweep into a *gate bag*: a synthetic session that
reproduces, deterministically, a motion the plans wanted a human hand for.

The relocalization plan's two open gates were "flick the camera away and
back" (kp mode) and "cover the lens, uncover on a known view" (rgbd
mode). Neither needs a hand — both are edits to a bag we already have:

    flick    A → blur gap → B → blur gap → A'    (kp mode)
    occlude  A → blur gap → A'                    (rgbd mode)
    loop     OUT → BACK (OUT played in reverse)   (rgbd mode; SLAM plan P0)

where A/B/A' are windows cut from the source sweep and A' *repeats part
of A*, so the pose the pipeline reports during A' has a ground truth:
whatever it reported for the same frames during A. `gate_check.py`
compares the two passes and decides.

The "blur gap" (--fill noise) stands in for motion blur / a hand: frames
with plenty of ORB keypoints that match nothing — including each other —
so the detector's pair tracking breaks the same way it breaks live and
its `relocalize_after` counter runs. --fill black is the other failure
shape — a covered lens: near-black frames with no keypoints at all, so
the detector has nothing to match rather than matches that fail. The
two fills exercise different branches of the loss detector; run both. rgbd_odometry loses
tracking on noise too and, with Odom/ResetCountdown=1, resets.

`loop` is the SLAM plan's loop bag: the sweep played forward and then
*backward*, so the camera retraces its own path to the start view — a
genuine loop with genuinely accumulated drift (tracking never breaks:
the turnaround is two near-identical frames), where every frame of the
return leg has the same source frame's outbound pose as its reference.
Loop closure should recognise the start at the end, and a backend that
optimises the graph should pull the two legs together (`traj_check.py
loop`). No fill, no gap.

Timeline surgery: every kept message keeps its own header→receive
offset (this camera's stamps lag ~0.73 s by fault — preserving the
offset keeps the replay honest) but is shifted so the output timeline is
continuous. /tf_static is copied once at the start; camera_info follows
the frames. Beside the bag a `gate.json` records the segments, the odom
mode to run under, the log lines the run must print and the thresholds
— the check reads it, so bag and check cannot drift apart.

Run through `just gate-bags` (needs the ROS environment: rosbag2_py,
rclpy, sensor_msgs; /usr/bin/python3, not the PlatformIO venv).
"""

import argparse
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage

IMAGE_TOPIC = '/image_raw/compressed'
INFO_TOPIC = '/camera_info'
STATIC_TOPIC = '/tf_static'
NS = 1_000_000_000

# What each gate is made of. Windows are seconds into the source bag
# (receive-time based, like `ros2 bag info`'s duration).
MODES = {
    'flick': {
        'odom': 'kp',
        'windows': {'a': (0.0, 8.0), 'b': (32.0, 40.0), 'back': (3.0, 8.0)},
        'expect_log': ['tracking lost for', 'relocalized against keyframe'],
        # A few degrees is the plan's own wording for the flick gate; the
        # kp compass carries no translation, so no position threshold.
        'thresholds': {'angle_deg': 5.0},
    },
    'occlude': {
        'odom': 'rgbd',
        # A runs long enough that the pose at the cover moment differs
        # from the pose at the uncover view by well over the detector's
        # min_correction (10°/0.3 m) — otherwise "recognised, no snap"
        # is the correct answer and the gate has nothing to test.
        'windows': {'a': (0.0, 14.0), 'back': (3.0, 9.0)},
        'expect_log': ['tracking lost for',
                       'relocalized against keyframe',
                       'snapping odometry'],
        'thresholds': {'angle_deg': 5.0, 'translation_m': 0.2},
    },
    'loop': {
        'odom': 'rgbd',
        # The whole sweep out and back; clipped to the source's length.
        'windows': {'a': (0.0, 3600.0)},
        'expect_log': [],
        # Not an absolute pass/fail on the odometry — drift is expected.
        # The verdict compares corrected against raw (traj_check.py loop).
        'thresholds': {},
    },
}


def parse_window(text):
    a, b = text.split(':')
    a, b = float(a), float(b)
    if b <= a:
        raise argparse.ArgumentTypeError(f'empty window {text!r}')
    return a, b


def read_bag(path):
    """Load the source bag fully — sweeps are ~250 MB, RAM is not the issue."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    # Keep the recorded topic metadata whole: /tf_static's offered QoS is
    # TRANSIENT_LOCAL, and a tf2 listener (which requests that
    # durability) would never match a plain volatile replay of it.
    types = {t.name: t for t in reader.get_all_topics_and_types()}
    msgs = {IMAGE_TOPIC: [], INFO_TOPIC: [], STATIC_TOPIC: []}
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in msgs:
            msgs[topic].append((t, data))
    if not msgs[IMAGE_TOPIC]:
        raise SystemExit(f'{path}: no {IMAGE_TOPIC} messages')
    return types, msgs


def fill_frame(kind, shape, rng):
    """One synthetic 'blurred' frame — texture that matches nothing."""
    h, w = shape
    if kind == 'black':
        # Near-black with faint sensor-like noise, not pure zeros: a
        # covered lens still differs frame to frame, and the detector
        # CRC-skips byte-identical frames whole (usb_cam duplicates), so
        # true zeros would never even reach it. Amplitude stays under
        # ORB's FAST threshold — no keypoints, which is the point.
        img = rng.integers(0, 6, (h, w, 3), np.uint8)
    else:
        # Coarse blobs, not per-pixel noise: JPEG would flatten
        # single-pixel noise into a grey mush with few corners. Blobs at
        # ~8 px keep ORB busy (hundreds of keypoints) and unrepeatable.
        small = rng.integers(0, 256, (h // 8, w // 8, 3), np.uint8)
        img = cv2.resize(small, (w, h), interpolation=cv2.INTER_NEAREST)
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    assert ok
    return buf.tobytes()


def stamp_ns(header):
    return header.stamp.sec * NS + header.stamp.nanosec


def set_stamp(header, ns):
    header.stamp.sec = int(ns // NS)
    header.stamp.nanosec = int(ns % NS)


def build(args):
    src = Path(args.src)
    out = Path(args.out)
    if out.exists():
        if not args.force:
            raise SystemExit(f'{out} exists — pass --force to replace it')
        shutil.rmtree(out)
    types, msgs = read_bag(src)
    images = msgs[IMAGE_TOPIC]
    infos = msgs[INFO_TOPIC]
    t_base = images[0][0]
    # Frame cadence, for the synthetic gap: median inter-frame gap.
    gaps = np.diff([t for t, _ in images])
    frame_dt = int(np.median(gaps))
    first = deserialize_message(images[0][1], CompressedImage)
    probe = cv2.imdecode(np.frombuffer(bytes(first.data), np.uint8),
                         cv2.IMREAD_COLOR)
    shape = probe.shape[:2]
    last_info = deserialize_message(infos[-1][1], CameraInfo)
    rng = np.random.default_rng(args.seed)

    spec = MODES[args.mode]
    src_len = gaps.sum() / NS
    if args.mode == 'flick':
        plan = [('A', args.a), ('gap', args.gap), ('B', args.b),
                ('gap', args.gap), ('A2', args.back)]
    elif args.mode == 'loop':
        window = (args.a[0], min(args.a[1], src_len))
        plan = [('OUT', window), ('BACK', window)]
    else:
        plan = [('A', args.a), ('gap', args.gap), ('A2', args.back)]

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for topic in (IMAGE_TOPIC, INFO_TOPIC, STATIC_TOPIC):
        meta = types[topic]
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=topic, type=meta.type, serialization_format='cdr',
            offered_qos_profiles=meta.offered_qos_profiles))
    for t, data in msgs[STATIC_TOPIC]:
        writer.write(STATIC_TOPIC, data, t_base)

    cursor = t_base          # output receive time of the next segment
    segments = []
    n_frames = 0
    for name, window in plan:
        if name == 'gap':
            duration = int(window * NS)
            t = cursor
            while t < cursor + duration:
                img = CompressedImage()
                img.header.frame_id = first.header.frame_id
                # header lags receive by the source's own fault offset
                set_stamp(img.header, t - (images[0][0] - stamp_ns(first.header)))
                img.format = first.format
                img.data = fill_frame(args.fill, shape, rng)
                writer.write(IMAGE_TOPIC, serialize_message(img), t)
                info = CameraInfo()
                info.header = img.header
                info.height, info.width = last_info.height, last_info.width
                info.distortion_model = last_info.distortion_model
                info.d, info.k, info.r, info.p = (
                    last_info.d, last_info.k, last_info.r, last_info.p)
                writer.write(INFO_TOPIC, serialize_message(info), t)
                t += frame_dt
                n_frames += 1
            segments.append({'name': name, 'kind': 'fill',
                             'out_t0': (cursor - t_base) / NS,
                             'out_t1': (t - t_base) / NS})
            cursor = t
            continue
        src_t0, src_t1 = window
        lo, hi = t_base + int(src_t0 * NS), t_base + int(src_t1 * NS)
        shift = cursor - lo
        reverse = name == 'BACK'
        count = 0
        for topic, cls in ((IMAGE_TOPIC, CompressedImage),
                           (INFO_TOPIC, CameraInfo)):
            for t, data in msgs[topic]:
                if not lo <= t < hi:
                    continue
                m = deserialize_message(data, cls)
                h = stamp_ns(m.header)
                if reverse:
                    # Mirror the *header* timeline inside the window (the
                    # frame stamped lo + x replays stamped hi - x) and
                    # rebuild receive time from each message's own
                    # header→receive lag. Mirroring headers, not receive
                    # times, keeps an image and its camera_info on the
                    # identical stamp they shared in the source — the
                    # exact sync downstream pairs on nothing else — and
                    # keeps the replayed stamps increasing.
                    h_new = lo + hi - h
                    t_new = h_new + (t - h)
                else:
                    h_new, t_new = h, t
                set_stamp(m.header, h_new + shift)
                writer.write(topic, serialize_message(m), t_new + shift)
                count += topic == IMAGE_TOPIC
        if count == 0:
            raise SystemExit(f'window {name} {src_t0}:{src_t1} is empty — '
                             f'the source lasts {gaps.sum() / NS:.1f} s')
        n_frames += count
        segments.append({'name': name, 'kind': 'rev' if reverse else 'src',
                         'src_t0': src_t0,
                         'src_t1': src_t1, 'out_t0': (cursor - t_base) / NS,
                         'out_t1': (cursor - t_base) / NS + (src_t1 - src_t0),
                         'frames': count})
        cursor = hi + shift
    writer.close()

    gate = {
        'mode': args.mode,
        'odom': spec['odom'],
        'source': str(src),
        'fill': args.fill,
        'image_topic': IMAGE_TOPIC,
        'duration_s': (cursor - t_base) / NS,
        'segments': segments,
        # A2 replays part of A: compare pipeline poses on A2 against the
        # poses the same source frames earned during A. (loop: BACK
        # replays OUT in reverse — kind 'rev' maps bag time t back to
        # source time src_t1 - (t - out_t0).)
        'compare': ({'reference': 'OUT', 'trial': 'BACK'}
                    if args.mode == 'loop'
                    else {'reference': 'A', 'trial': 'A2'}),
        'expect_log': spec['expect_log'],
        'thresholds': spec['thresholds'],
    }
    (out / 'gate.json').write_text(json.dumps(gate, indent=2) + '\n')
    print(f'{out}: {n_frames} frames, {gate["duration_s"]:.1f} s, '
          f'odom={spec["odom"]}, fill={args.fill}')
    for s in segments:
        src_txt = (f'src {s["src_t0"]:.0f}–{s["src_t1"]:.0f} s'
                   if s['kind'] == 'src'
                   else f'src {s["src_t1"]:.0f}–{s["src_t0"]:.0f} s reversed'
                   if s['kind'] == 'rev' else f'{args.fill} fill')
        print(f'  {s["name"]:<4} {s["out_t0"]:5.1f}–{s["out_t1"]:5.1f} s  '
              f'{src_txt}')


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('mode', choices=sorted(MODES))
    p.add_argument('src', help='source bag directory (a hand sweep)')
    p.add_argument('out', help='output bag directory')
    p.add_argument('--a', type=parse_window,
                   help='window A, seconds into the source '
                        '(default per mode: flick 0:8, occlude 0:14)')
    p.add_argument('--b', type=parse_window,
                   help='flick only: the away view B (default 32:40)')
    p.add_argument('--back', type=parse_window,
                   help="A', the return — must lie inside A "
                        '(default per mode: flick 3:8, occlude 3:9)')
    p.add_argument('--gap', type=float, default=3.0,
                   help='seconds of fill between segments (default 3)')
    p.add_argument('--fill', choices=('noise', 'black'), default='noise')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--force', action='store_true')
    args = p.parse_args()
    for key, window in MODES[args.mode]['windows'].items():
        if getattr(args, key) is None:
            setattr(args, key, window)
    if args.mode != 'loop' and not (
            args.a[0] <= args.back[0] and args.back[1] <= args.a[1]):
        p.error("--back must lie inside --a (A' repeats part of A)")
    build(args)


if __name__ == '__main__':
    main()
