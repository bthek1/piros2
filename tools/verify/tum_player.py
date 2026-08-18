"""
Play a TUM RGB-D sequence into the world_mesh session as if it were the
camera *and* the depth net — real depth, real ground truth (SLAM plan P0).

The session normally gets JPEG frames from the Pi and depth from the
monocular network. This node publishes, from a sequence directory
(`just fetch-tum`), everything downstream expects, at the sequence's own
cadence:

    /image_raw/compressed     the RGB frame as JPEG (the relay fans it out
                              to detector / dashboard / mesher / projector)
    /camera_info              the freiburg calibration (K, plumb_bob D)
    /depth                    32FC1 metres from the 16-bit PNG (÷5000)
    /depth/rgb                the same frame raw bgr8 — the estimator's
                              stamp-twin contract, so rgbd_odometry's
                              exact sync pairs every frame
    /tf_static                base_link → camera_link → camera_optical_frame
                              (the camera launch's chain, translation zero
                              so base_link *is* the camera for the ATE)

Every message of a frame carries the TUM stamp — the ground truth is
associated by it (`traj_check.py ate`), and rgbd_odometry / rtabmap
stamp their TF with the image stamp, so the recorded trajectory lands
straight on the truth's clock. Run with `depth_source:=external` so the
depth estimator stays out of the session (two publishers on /depth
otherwise). ROS environment, /usr/bin/python3 (cv2 + numpy, no
cv_bridge needed).
"""

import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import StaticTransformBroadcaster

NS = 1_000_000_000
TUM_DEPTH_SCALE = 5000.0
OPTICAL_FRAME = 'camera_optical_frame'

# TUM benchmark calibrations (fx, fy, cx, cy, d[5]) per Kinect.
INTRINSICS = {
    'fr1': (517.306408, 516.469215, 318.643040, 255.313989,
            [0.262383, -0.953104, -0.005358, 0.002628, 1.163314]),
    'fr2': (520.908620, 521.007327, 325.141442, 249.701764,
            [0.231222, -0.784899, -0.003257, -0.000105, 0.917205]),
    'fr3': (535.4, 539.2, 320.1, 247.6, [0.0, 0.0, 0.0, 0.0, 0.0]),
}


def read_stamped(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        stamp, name = line.split()[:2]
        rows.append((float(stamp), name))
    return rows


def associate(rgb, depth, tol):
    """rgb → nearest depth within tol (one-to-one, greedy in time)."""
    d_stamps = np.array([s for s, _ in depth])
    used, pairs = set(), []
    for stamp, name in rgb:
        i = int(np.searchsorted(d_stamps, stamp))
        best, best_d = None, tol + 1
        for j in (i - 1, i):
            if 0 <= j < len(depth) and j not in used:
                dd = abs(d_stamps[j] - stamp)
                if dd < best_d:
                    best, best_d = j, dd
        if best is not None and best_d <= tol:
            used.add(best)
            pairs.append((stamp, name, depth[best][1]))
    return pairs


def guess_camera(sequence):
    name = Path(sequence).name
    for key in INTRINSICS:
        if f'freiburg{key[-1]}' in name:
            return key
    return 'fr1'


def stamp_msg(header, t):
    header.stamp.sec = int(t)
    header.stamp.nanosec = int(round((t - int(t)) * NS))
    header.frame_id = OPTICAL_FRAME


class TumPlayer(Node):

    def __init__(self, args):
        super().__init__('tum_player')
        seq = Path(args.sequence)
        rgb = read_stamped(seq / 'rgb.txt')
        depth = read_stamped(seq / 'depth.txt')
        pairs = associate(rgb, depth, args.tol)
        if args.start > 0:
            pairs = [p for p in pairs if p[0] - pairs[0][0] >= args.start]
        if args.duration > 0:
            pairs = [p for p in pairs if p[0] - pairs[0][0] <= args.duration]
        self.pairs = pairs[::args.every]
        self.seq = seq
        self.speed = args.speed
        cam = args.camera or guess_camera(seq)
        self.fx, self.fy, self.cx, self.cy, self.d = INTRINSICS[cam]
        self.jpeg_quality = args.jpeg_quality
        self.get_logger().info(
            f'{seq.name}: {len(rgb)} rgb, {len(depth)} depth, '
            f'{len(pairs)} paired, playing {len(self.pairs)} frames '
            f'(every {args.every}) at {args.speed}x, camera {cam}')

        big = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_jpeg = self.create_publisher(
            CompressedImage, '/image_raw/compressed', big)
        self.pub_info = self.create_publisher(CameraInfo, '/camera_info', big)
        self.pub_depth = self.create_publisher(Image, '/depth', big)
        self.pub_rgb = self.create_publisher(Image, '/depth/rgb', big)
        self.static = StaticTransformBroadcaster(self)
        self.publish_static()

    def publish_static(self):
        mount = TransformStamped()
        mount.header.frame_id = 'base_link'
        mount.child_frame_id = 'camera_link'
        mount.transform.rotation.w = 1.0
        optical = TransformStamped()
        optical.header.frame_id = 'camera_link'
        optical.child_frame_id = OPTICAL_FRAME
        # rpy (-90°, 0, -90°): body x-forward/z-up → optical z-forward/y-down
        optical.transform.rotation.x = -0.5
        optical.transform.rotation.y = 0.5
        optical.transform.rotation.z = -0.5
        optical.transform.rotation.w = 0.5
        now = self.get_clock().now().to_msg()
        mount.header.stamp = now
        optical.header.stamp = now
        self.static.sendTransform([mount, optical])

    def camera_info(self, t, h, w):
        info = CameraInfo()
        stamp_msg(info.header, t)
        info.height, info.width = h, w
        info.distortion_model = 'plumb_bob'
        info.d = list(self.d)
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def play(self):
        if not self.pairs:
            self.get_logger().error('nothing to play')
            return 0
        t0_seq = self.pairs[0][0]
        t0_wall = time.monotonic()
        sent = 0
        for stamp, rgb_name, depth_name in self.pairs:
            if not rclpy.ok():
                break
            due = t0_wall + (stamp - t0_seq) / self.speed
            delay = due - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            bgr = cv2.imread(str(self.seq / rgb_name), cv2.IMREAD_COLOR)
            raw = cv2.imread(str(self.seq / depth_name), cv2.IMREAD_UNCHANGED)
            if bgr is None or raw is None:
                self.get_logger().warn(f'unreadable frame {rgb_name}')
                continue
            depth = raw.astype(np.float32) / TUM_DEPTH_SCALE
            h, w = bgr.shape[:2]

            jpeg = CompressedImage()
            stamp_msg(jpeg.header, stamp)
            jpeg.format = 'jpeg'
            ok, buf = cv2.imencode('.jpg', bgr,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            jpeg.data = buf.tobytes()

            rgb_msg = Image()
            stamp_msg(rgb_msg.header, stamp)
            rgb_msg.height, rgb_msg.width = h, w
            rgb_msg.encoding = 'bgr8'
            rgb_msg.step = w * 3
            rgb_msg.data = bgr.tobytes()

            depth_msg = Image()
            stamp_msg(depth_msg.header, stamp)
            depth_msg.height, depth_msg.width = h, w
            depth_msg.encoding = '32FC1'
            depth_msg.step = w * 4
            depth_msg.data = depth.tobytes()

            # Same order as the estimator: RGB twin first, then depth, so an
            # exact sync completes the moment /depth lands.
            self.pub_info.publish(self.camera_info(stamp, h, w))
            self.pub_jpeg.publish(jpeg)
            self.pub_rgb.publish(rgb_msg)
            self.pub_depth.publish(depth_msg)
            sent += 1
            rclpy.spin_once(self, timeout_sec=0)
        return sent


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('sequence', help='TUM sequence directory')
    p.add_argument('--every', type=int, default=1,
                   help='publish every N-th paired frame (default 1)')
    p.add_argument('--speed', type=float, default=1.0,
                   help='playback speed factor (default real time)')
    p.add_argument('--start', type=float, default=0.0,
                   help='skip this many seconds of the sequence')
    p.add_argument('--duration', type=float, default=0.0,
                   help='play at most this many seconds (0 = all)')
    p.add_argument('--tol', type=float, default=0.02,
                   help='rgb↔depth association tolerance (s)')
    p.add_argument('--camera', choices=sorted(INTRINSICS),
                   help='calibration set (default: guessed from the name)')
    p.add_argument('--jpeg-quality', type=int, default=90)
    p.add_argument('--settle', type=float, default=2.0,
                   help='seconds to wait after the static TF before frames')
    args = p.parse_args()
    rclpy.init()
    node = TumPlayer(args)
    # Let subscribers discover the publishers (and the latched static TF)
    # before the first frame — a frame published into the void is a frame
    # the recorder never sees.
    t_end = time.monotonic() + args.settle
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        sent = node.play()
    except KeyboardInterrupt:
        sent = -1
    finally:
        node.get_logger().info(f'played {sent} frames')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if sent else 1


if __name__ == '__main__':
    sys.exit(main())
