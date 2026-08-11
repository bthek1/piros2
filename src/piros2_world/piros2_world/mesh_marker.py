"""
Publish the newest fused mesh into RViz as a latched Marker.

The bridge between the offline reconstruction pipeline (tools/recon/,
world fusion plan) and the live `just world` session. Two concepts:

- A **latched topic**: the publisher QoS is TRANSIENT_LOCAL, so the one
  Marker published at startup is delivered to every subscriber that
  joins later — RViz can start after this node and still receive the
  mesh. Contrast BIG_FRAME_QOS everywhere else in this package, which
  is VOLATILE on purpose (a stale camera frame is worthless; a mesh is
  not). This is ROS 2's version of ROS 1's latched publisher.
- **mesh_resource**: the Marker carries a `file://` URI, not geometry —
  RViz loads the file from disk itself, so the message is ~a hundred
  bytes where a TRIANGLE_LIST of the same mesh would be tens of MB of
  DDS traffic for a thing that never changes.

Honesty about frames: the mesh was fused in its capture's world frame
(the first keyframe's optical frame, or RTAB-Map's odom start), which
is NOT this session's live `odom` origin. The marker is pinned at the
odom origin as a *reference overlay* — expect it near, not aligned
with, the live cloud. Aligning them would need relocalisation, which is
out of scope by the plan.

The node stays up when no mesh exists (warn, publish nothing): the
world session must not die because reconstruction hasn't run yet. The
`~/reload` service re-scans and republishes, so a fresh `just
fuse-capture` shows up without restarting the session.
"""

from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

# Latched: one publish serves every late-joining RViz (see module doc).
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

# Anchored on __file__, the same convention as the linter tests and the
# model path: source tree <repo>/src/piros2_world/piros2_world/ → the
# repo root is three levels up, and --symlink-install resolves the
# install-space copy back here.
DEFAULT_MESHES_DIR = str(Path(__file__).resolve().parents[3] / 'meshes')


def newest_mesh(meshes_dir):
    """
    Return the most recently written .ply in the dir, or None.

    PLY only, on measured evidence: RViz's assimp loads our PLYs
    cleanly but rejects Open3D's GLB export ("buffer view out of
    range"), so .glb stays the format for external viewers and this
    overlay sticks to what renders.
    """
    candidates = list(Path(meshes_dir).glob('*.ply'))
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


class MeshMarker(Node):

    def __init__(self, **node_kwargs):
        # node_kwargs passes through parameter_overrides — the tests
        # point meshes_dir at a temp directory this way.
        super().__init__('mesh_marker', **node_kwargs)
        # Empty = auto-pick the newest mesh at startup/reload; set a
        # path to pin a specific one.
        self.declare_parameter('mesh_path', '')
        self.declare_parameter('meshes_dir', DEFAULT_MESHES_DIR)
        # Display frame only — see the module docstring's frame honesty.
        self.declare_parameter('frame_id', 'odom')

        self.pub = self.create_publisher(
            Marker, 'world/mesh', LATCHED_QOS)
        self.create_service(Trigger, '~/reload', self.on_reload)
        self.publish_mesh()

    def pick_mesh(self):
        configured = self.get_parameter('mesh_path').value
        if configured:
            path = Path(configured)
            return path if path.is_file() else None
        return newest_mesh(self.get_parameter('meshes_dir').value)

    def publish_mesh(self):
        """Publish the current pick; returns the path or None."""
        path = self.pick_mesh()
        if path is None:
            self.get_logger().warn(
                'no mesh to show (nothing in '
                f'{self.get_parameter("meshes_dir").value} and no '
                'mesh_path set) — run the recon pipeline, then call '
                '~/reload')
            return None
        marker = Marker()
        marker.header.frame_id = self.get_parameter('frame_id').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fused_mesh'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = f'file://{path.resolve()}'
        # All-zero colour + embedded materials = use the file's own
        # colours; a non-zero colour here would tint every vertex.
        marker.mesh_use_embedded_materials = True
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        self.pub.publish(marker)
        self.get_logger().info(f'showing {path.name} (latched)')
        return path

    def on_reload(self, request, response):
        path = self.publish_mesh()
        response.success = path is not None
        response.message = (f'showing {path.name}' if path
                            else 'no mesh found')
        return response


def main():
    rclpy.init()
    node = MeshMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
