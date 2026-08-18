"""
One snapshot of what a running session is publishing — files, not eyes.

The Playwright analogue for a ROS graph is not a screenshot but the
topics: every intermediate result the session shows in RViz is also on
the wire, so a snapshot is one message from each of them written where
a person or an assistant can open it. Compressed image topics are JPEG
already (`format` says so) — the bytes are the file, no decoding — and
the scalar/geometry topics reduce to a few numbers:

    <out>/
    ├── camera_relay.jpg, keypoints.jpg, depth_preview.jpg, stats.jpg
    ├── summary.txt      # what arrived, counts, TF, mesh triangles
    └── summary.json     # the same, for scripts

Every topic is optional and reported as `missing` rather than fatal —
`just orient` runs only the detector, and an honest snapshot of that
session has most slots empty. Latched topics (`/world/mesh_live`,
`/world/keyframes`) are read with TRANSIENT_LOCAL durability so the last
message is delivered on subscribe. `/tf` is read raw and the newest
`odom → base_link` reported; the static chain is not resolved.

Run through `just snap` (ROS environment; /usr/bin/python3).
"""

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Int32
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

IMAGE_TOPICS = {
    'camera_relay': '/camera_relay/compressed',
    'camera_raw': '/image_raw/compressed',
    'keypoints': '/keypoints/compressed',
    'depth_preview': '/depth/preview/compressed',
    'stats': '/world/stats/compressed',
}
INT_TOPICS = {
    'keypoints_count': '/keypoints/count',
    'keypoints_matched': '/keypoints/matched',
    'keyframes_stored': '/keypoints/keyframes',
}


def yaw_deg(x, y, z, w):
    return float(np.degrees(np.arctan2(2 * (w * z + x * y),
                                       1 - 2 * (y * y + z * z))))


class Snapper(Node):

    def __init__(self, out):
        super().__init__('topic_snap')
        self.out = out
        self.got = {}
        reliable = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for key, topic in IMAGE_TOPICS.items():
            self.create_subscription(
                CompressedImage, topic,
                lambda m, k=key: self.on_image(k, m), reliable)
        for key, topic in INT_TOPICS.items():
            self.create_subscription(
                Int32, topic, lambda m, k=key: self.got.setdefault(k, m.data),
                reliable)
        self.create_subscription(PointCloud2, '/points', self.on_points,
                                 reliable)
        self.create_subscription(Marker, '/world/mesh_live', self.on_mesh,
                                 latched)
        self.create_subscription(Marker, '/world/keyframes',
                                 self.on_keyframes, latched)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)

    def on_image(self, key, msg):
        if key in self.got:
            return
        ext = 'jpg' if 'jpeg' in msg.format or 'jpg' in msg.format else 'png'
        path = self.out / f'{key}.{ext}'
        path.write_bytes(bytes(msg.data))
        self.got[key] = {'file': path.name, 'format': msg.format,
                         'bytes': len(msg.data),
                         'frame_id': msg.header.frame_id}

    def on_points(self, msg):
        self.got.setdefault('points', {
            'count': msg.width * msg.height, 'frame_id': msg.header.frame_id})

    def on_mesh(self, msg):
        self.got.setdefault('mesh_live', {
            'triangles': len(msg.points) // 3, 'frame_id': msg.header.frame_id,
            'action': int(msg.action)})

    def on_keyframes(self, msg):
        # one LINE_LIST marker, two points per stored viewpoint stroke
        self.got.setdefault('keyframes', {'strokes': len(msg.points) // 2})

    def on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_link':
                q, t = tf.transform.rotation, tf.transform.translation
                self.got['odom_base_link'] = {
                    'yaw_deg': round(yaw_deg(q.x, q.y, q.z, q.w), 2),
                    'xyz': [round(t.x, 3), round(t.y, 3), round(t.z, 3)]}


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('out')
    p.add_argument('--wait', type=float, default=5.0,
                   help='seconds to listen (default 5)')
    args = p.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Snapper(out)
    deadline = time.time() + args.wait
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    topics = {name for name, _ in node.get_topic_names_and_types()}
    node.destroy_node()
    rclpy.shutdown()

    keys = (list(IMAGE_TOPICS) + list(INT_TOPICS)
            + ['points', 'mesh_live', 'keyframes', 'odom_base_link'])
    summary = {k: node.got.get(k, 'missing') for k in keys}
    summary['topics_advertised'] = sorted(topics)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    lines = [f'snapshot after {args.wait:.0f} s — {out}']
    for k in keys:
        v = summary[k]
        lines.append(f'  {k:<18} {v if v != "missing" else "missing"}')
    lines.append(f'  topics advertised: {len(topics)}')
    text = '\n'.join(lines) + '\n'
    (out / 'summary.txt').write_text(text)
    print(text, end='')


if __name__ == '__main__':
    main()
