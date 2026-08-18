"""
Record what a headless session says about where the camera is — to files.

The recorder half of the SLAM gates (SLAM plan P0): it sits beside a
`world_mesh.launch.py` session while a bag (or the TUM player) feeds it,
and writes, into --out:

    frames.txt      header_ns wall_ns per frame on the image topic — the
                    two clocks every later mapping needs (this camera's
                    header stamps lag receipt by ~0.73 s by fault)
    odom.txt/.ns    odom → base_link from /tf, TUM form (+ exact ns
                    stamps and receipt wall time in the .ns twin)
    map_odom.txt/.ns  map → odom from /tf — the correction a SLAM
                    backend publishes (REP-105); empty when nothing did
    path_<topic>.txt/.ns  the LAST nav_msgs/Path seen on each --path
                    topic — an optimised trajectory (RTAB-Map's
                    /rtabmap/mapPath, the fork's /world/trajectory)
    meta.json       counts and timing

Then `traj_check.py loop|ate` reads the files and decides. Nothing here
judges anything: recording and judging are separate so a run can be
re-scored with different thresholds without replaying.

Ends when the image topic has been silent for --idle seconds (the bag
finished), on --timeout, or on Ctrl-C. Run through the `gate-*` recipes
(ROS environment; /usr/bin/python3, not the PlatformIO venv).
"""

import argparse
import json
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from tf2_msgs.msg import TFMessage

NS = 1_000_000_000


class TrajRecorder(Node):

    def __init__(self, image_topic, path_topics, frames):
        super().__init__('traj_record')
        self.frames = []                     # (header_ns, wall_ns)
        self.tf_rows = {pair: [] for pair in frames}   # pair → rows
        self.paths = {t: None for t in path_topics}    # topic → (msg, wall)
        self.last_image_wall = None
        reliable = QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CompressedImage, image_topic,
                                 self.on_image, reliable)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        for topic in path_topics:
            self.create_subscription(
                PathMsg, topic,
                lambda msg, topic=topic: self.on_path(topic, msg), 10)

    def on_image(self, msg):
        now = time.time_ns()
        self.frames.append(
            (msg.header.stamp.sec * NS + msg.header.stamp.nanosec, now))
        self.last_image_wall = now

    def on_tf(self, msg):
        now = time.time_ns()
        for tf in msg.transforms:
            pair = (tf.header.frame_id, tf.child_frame_id)
            if pair not in self.tf_rows:
                continue
            q, t = tf.transform.rotation, tf.transform.translation
            self.tf_rows[pair].append((
                tf.header.stamp.sec * NS + tf.header.stamp.nanosec, now,
                t.x, t.y, t.z, q.x, q.y, q.z, q.w))

    def on_path(self, topic, msg):
        self.paths[topic] = (msg, time.time_ns())


def write_rows(out, name, rows, header):
    with open(out / f'{name}.txt', 'w') as fh, \
            open(out / f'{name}.ns', 'w') as fn:
        fh.write(f'# {header}\n# stamp x y z qx qy qz qw\n')
        fn.write('# stamp_ns wall_ns\n')
        for stamp, wall, x, y, z, qx, qy, qz, qw in rows:
            fh.write(f'{stamp / NS:.6f} {x:.6f} {y:.6f} {z:.6f} '
                     f'{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n')
            fn.write(f'{stamp} {wall}\n')


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--out', required=True)
    p.add_argument('--image-topic', default='/image_raw/compressed')
    p.add_argument('--path', action='append', default=[],
                   help='nav_msgs/Path topic to keep the last message of '
                        '(repeatable)')
    p.add_argument('--wait', type=float, default=90.0,
                   help='give up if no frame arrives in this long')
    p.add_argument('--idle', type=float, default=6.0,
                   help='no frames for this long = the feed finished')
    p.add_argument('--timeout', type=float, default=600.0)
    p.add_argument('--settle', type=float, default=3.0,
                   help='seconds to keep listening after the feed ends '
                        '(a final graph optimisation may still land)')
    args = p.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    pairs = [('odom', 'base_link'), ('map', 'odom')]
    rclpy.init()
    node = TrajRecorder(args.image_topic, args.path, pairs)
    node.get_logger().info(f'recording {args.image_topic}, /tf '
                           f'{pairs}, paths {args.path} → {out}')
    t_start = time.time()
    finished_at = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time()
            if node.last_image_wall is None:
                if now - t_start > args.wait:
                    node.get_logger().error(
                        f'no frames within {args.wait:.0f} s')
                    break
                continue
            idle = (time.time_ns() - node.last_image_wall) / NS
            if idle > args.idle and finished_at is None:
                finished_at = now
                node.get_logger().info(
                    f'no frames for {idle:.1f} s — feed finished, settling '
                    f'{args.settle:.0f} s for a last optimisation')
            if finished_at is not None and now - finished_at > args.settle:
                break
            if now - t_start > args.timeout:
                node.get_logger().warn('timeout reached')
                break
    except KeyboardInterrupt:
        pass
    finally:
        frames = list(node.frames)
        tf_rows = {k: list(v) for k, v in node.tf_rows.items()}
        paths = dict(node.paths)
        node.destroy_node()
        rclpy.shutdown()

    with open(out / 'frames.txt', 'w') as fh:
        fh.write('# header_ns wall_ns\n')
        for h, w in frames:
            fh.write(f'{h} {w}\n')
    write_rows(out, 'odom', tf_rows[('odom', 'base_link')],
               'odom -> base_link from /tf')
    write_rows(out, 'map_odom', tf_rows[('map', 'odom')],
               'map -> odom from /tf')
    meta = {'frames': len(frames),
            'odom_poses': len(tf_rows[('odom', 'base_link')]),
            'map_odom_updates': len(tf_rows[('map', 'odom')]),
            'image_topic': args.image_topic, 'paths': {}}
    for topic, entry in paths.items():
        name = 'path_' + topic.strip('/').replace('/', '_')
        if entry is None:
            meta['paths'][topic] = 0
            continue
        msg, wall = entry
        rows = []
        for ps in msg.poses:
            q, t = ps.pose.orientation, ps.pose.position
            rows.append((ps.header.stamp.sec * NS + ps.header.stamp.nanosec,
                         wall, t.x, t.y, t.z, q.x, q.y, q.z, q.w))
        write_rows(out, name, rows,
                   f'last {topic} ({msg.header.frame_id}), '
                   f'{len(rows)} poses')
        meta['paths'][topic] = len(rows)
    (out / 'meta.json').write_text(json.dumps(meta, indent=2) + '\n')
    print(f'recorded {meta["frames"]} frames, {meta["odom_poses"]} odom '
          f'poses, {meta["map_odom_updates"]} map→odom updates, paths '
          f'{meta["paths"]} → {out}')
    if frames:
        span = (frames[-1][1] - frames[0][1]) / NS
        print(f'  feed spanned {span:.1f} s of wall time')


if __name__ == '__main__':
    main()
