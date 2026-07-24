"""A minimal publisher: the rclpy node lifecycle end to end.

The shape to internalise: init -> construct a Node -> spin -> destroy ->
shutdown. Everything the node *does* happens inside callbacks fired by spin();
the constructor only wires things up.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloTalker(Node):

    def __init__(self):
        # The string here is the node's name on the graph — what `ros2 node
        # list` shows — and is independent of the topic name below.
        super().__init__('hello_talker')

        # 10 is the QoS history depth: how many outgoing messages are buffered
        # per subscriber before old ones are dropped. Passing a bare int gets
        # the default profile (RELIABLE, VOLATILE) — fine for a chat topic;
        # sensor streams will want BEST_EFFORT instead, which is milestone 2's
        # lesson.
        self.publisher = self.create_publisher(String, 'hello', 10)

        # No sleep loops in ROS nodes: the executor owns the thread, and a
        # timer asks it to call us back every 0.5 s. Blocking a callback would
        # starve every other callback on this node.
        self.timer = self.create_timer(0.5, self.tick)
        self.count = 0

    def tick(self):
        msg = String()
        msg.data = f'hello from ml5: {self.count}'
        self.publisher.publish(msg)
        # The node's logger, not print(): this line is timestamped, levelled,
        # and visible to `ros2 run ... --ros-args --log-level`.
        self.get_logger().info(f'publishing: "{msg.data}"')
        self.count += 1


def main():
    rclpy.init()
    node = HelloTalker()
    try:
        # spin() blocks here, dispatching timer and subscription callbacks
        # until the process is interrupted.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
