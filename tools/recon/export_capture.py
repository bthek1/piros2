"""
Export a recorded bag to a TUM-RGB-D-layout keyframe directory.

World fusion plan P3 — the capture layer, translated to this repo: the
bag is already the raw capture (`just record` keeps the compressed
frames, camera_info and TF), and everything else is *derived*. This
script materialises the derived interchange form existing tooling reads:

    captures/<name>/
    ├── rgb/<t>.png        # decoded keyframes
    ├── depth/<t>.png      # 16-bit millimetres, from the ONNX depth model
    ├── rgb.txt, depth.txt # TUM timestamped file lists
    ├── groundtruth.txt    # T_wc per keyframe: t tx ty tz qx qy qz qw
    └── K.txt              # the 3x3 intrinsics from the bag

Run it through `just export-capture` — it needs the perception venv
(onnxruntime) with the workspace overlay sourced (rosbag2_py, rclpy, and
the piros2 packages).

The choices that carry the phase's lessons:

- Depth is derived, so its honest home is the export, not the bag: each
  kept frame runs through Depth Anything V2 here, using the estimator's
  own constants and conversion (relative inverse depth -> depth_scale/x
  metres). The scale is unpinned until P4's tape-measure check — stated
  in --depth-scale's help rather than hidden.
- Poses live in a separate rewritable file. groundtruth.txt is computed
  by re-running the ORB -> Kabsch estimator over consecutive keyframes
  (the same pure functions the live detector uses), composed exactly as
  the detector composes: R_wc = R_wc @ R_frame.T, world = the first
  keyframe's optical frame. Translation is zero — **rotation-only**, the
  live odometry's honest scope; P4 overwrites this one file with
  RTAB-Map's optimised 6-DoF trajectory and re-fuses, touching nothing
  else. That asymmetry is the whole argument for the layout.
- Timestamps are the bag's receive times: this camera's header stamps
  lag ~0.73 s by fault (docs/info/camera.md#timestamps) and order is all
  association needs.
- usb_cam's byte-identical duplicate republishes are CRC-skipped before
  decoding (the detector's trick), then every Nth distinct frame is
  kept: fusion wants coverage, not 60 fps.
"""

import argparse
from pathlib import Path
import zlib

import cv2
import numpy as np
from piros2_perception.depth_estimator import (
    DEFAULT_MODEL_PATH,
    IMAGENET_MEAN,
    IMAGENET_STD,
    MODEL_SIZE,
)
from piros2_world.keypoint_detector import (
    estimate_rotation,
    rays_from_pixels,
)
from piros2_world.se3 import quaternion_from_rotation
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CameraInfo, CompressedImage

MATCH_MAX_DISTANCE = 64  # Hamming bits; the live detector's gate


def read_bag(bag_path):
    """Yield (topic, message, t_sec) for the two topics the export reads."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path)),
                rosbag2_py.ConverterOptions('', ''))
    types = {'/image_raw/compressed': CompressedImage,
             '/camera_info': CameraInfo}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in types:
            yield topic, deserialize_message(data, types[topic]), t_ns / 1e9


def load_depth_session(model_path):
    """ONNX session, CUDA preferred — the estimator's loading, headless."""
    import onnxruntime

    onnxruntime.preload_dlls()
    session = onnxruntime.InferenceSession(
        model_path,
        providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
    print(f'inference provider: {session.get_providers()[0]}')
    return session


def infer_depth_mm(session, frame_bgr, depth_scale):
    """BGR frame -> uint16 depth in millimetres, the TUM-ish storage form.

    (TUM proper uses 5000 units/m; we use 1000 and say so in fuse_capture
    — millimetres are the doc's recommendation and rounder to reason
    about.)
    """
    rgb = cv2.cvtColor(
        cv2.resize(frame_bgr, (MODEL_SIZE, MODEL_SIZE),
                   interpolation=cv2.INTER_AREA),
        cv2.COLOR_BGR2RGB)
    x = (rgb.astype(np.float32) / 255.0 - IMAGENET_MEAN) / IMAGENET_STD
    x = x.transpose(2, 0, 1)[np.newaxis]
    relative = np.squeeze(
        session.run(None, {session.get_inputs()[0].name: x})[0])
    metres = np.clip(depth_scale / np.maximum(relative, 1e-3), 0.0, 65.0)
    metres = cv2.resize(metres, (frame_bgr.shape[1], frame_bgr.shape[0]),
                        interpolation=cv2.INTER_LINEAR)
    return np.round(metres * 1000.0).astype(np.uint16)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('bag', type=Path)
    parser.add_argument('name', help='capture name; writes captures/<name>')
    parser.add_argument('--every', type=int, default=4,
                        help='keep every Nth distinct frame (default 4: '
                             '~10 keyframes/s at the camera\'s 40-60 fps)')
    parser.add_argument('--depth-scale', type=float, default=2.69,
                        help='metres = this / model output; pinned '
                             '2026-08-10 by the tape-measure check '
                             '(2.50 m wall; perception.yaml has the '
                             'working)')
    parser.add_argument('--model', default=DEFAULT_MODEL_PATH)
    args = parser.parse_args()

    out = Path('captures') / args.name
    (out / 'rgb').mkdir(parents=True, exist_ok=True)
    (out / 'depth').mkdir(parents=True, exist_ok=True)

    session = load_depth_session(args.model)
    orb = cv2.ORB_create(nfeatures=500)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    k_matrix = None
    orientation = np.eye(3)  # R_wc; world = first keyframe's optical frame
    prev_crc = None
    prev_points = None
    prev_descriptors = None
    distinct = kept = untrusted = 0
    rgb_lines, depth_lines, pose_lines = [], [], []

    for topic, msg, t in read_bag(args.bag):
        if topic == '/camera_info':
            k = np.array(msg.k).reshape(3, 3)
            if k_matrix is None and k[0, 0] > 0.0:
                k_matrix = k
            continue

        crc = zlib.crc32(bytes(msg.data))
        if crc == prev_crc:
            continue
        prev_crc = crc
        distinct += 1
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                             cv2.IMREAD_COLOR)
        if frame is None:
            continue

        # Rotation integrates over EVERY distinct frame — the live
        # detector's cadence — because the per-step motion (and blur
        # tolerance) is what the estimator's gates were tuned for; only
        # the *saving* is strided. Estimating keyframe-to-keyframe
        # instead lost 41% of steps on the first real sweep.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = orb.detectAndCompute(gray, None)
        points = np.array([kp.pt for kp in keypoints or ()],
                          dtype=np.float64).reshape(-1, 2)
        if (k_matrix is not None and descriptors is not None
                and prev_descriptors is not None):
            pairs = [m for m in matcher.match(descriptors, prev_descriptors)
                     if m.distance <= MATCH_MAX_DISTANCE]
            rotation = estimate_rotation(
                rays_from_pixels(points[[m.queryIdx for m in pairs]],
                                 k_matrix),
                rays_from_pixels(prev_points[[m.trainIdx for m in pairs]],
                                 k_matrix)) if pairs else None
            if rotation is not None:
                orientation = orientation @ rotation.T
            else:
                untrusted += 1  # orientation carried forward, said aloud
        prev_points, prev_descriptors = points, descriptors

        if (distinct - 1) % args.every:
            continue
        stamp = f'{t:.6f}'
        cv2.imwrite(str(out / 'rgb' / f'{stamp}.png'), frame)
        depth_mm = infer_depth_mm(session, frame, args.depth_scale)
        cv2.imwrite(str(out / 'depth' / f'{stamp}.png'), depth_mm)
        rgb_lines.append(f'{stamp} rgb/{stamp}.png')
        depth_lines.append(f'{stamp} depth/{stamp}.png')
        qx, qy, qz, qw = quaternion_from_rotation(orientation)
        pose_lines.append(f'{stamp} 0 0 0 {qx:.9f} {qy:.9f} {qz:.9f} '
                          f'{qw:.9f}')
        kept += 1

    if k_matrix is None:
        raise SystemExit('no valid K in the bag (all-zero camera_info — '
                         'a pre-P0-intrinsics recording?); nothing usable')

    header = '# t filename — exported from ' + str(args.bag)
    (out / 'rgb.txt').write_text('\n'.join([header] + rgb_lines) + '\n')
    (out / 'depth.txt').write_text('\n'.join([header] + depth_lines) + '\n')
    (out / 'groundtruth.txt').write_text('\n'.join(
        ['# t tx ty tz qx qy qz qw — T_wc, world = first keyframe '
         'optical frame; ROTATION-ONLY (translation zero), see the '
         'world fusion plan', ] + pose_lines) + '\n')
    np.savetxt(out / 'K.txt', k_matrix, fmt='%.6f')

    print(f'{kept} keyframes ({distinct} distinct frames in the bag, '
          f'every {args.every}th kept), {untrusted} untrusted rotation '
          f'steps')
    print(f'K: fx={k_matrix[0, 0]:.1f} fy={k_matrix[1, 1]:.1f}')
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
