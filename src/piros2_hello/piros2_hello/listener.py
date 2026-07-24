"""
A minimal subscriber: the receiving half of milestone 1.

Discovery does the matchmaking — this node never names a host or address. It
declares interest in the topic 'hello' with a compatible QoS, and DDS connects
it to any publisher of that topic on the same ROS_DOMAIN_ID, whether the
publisher is this machine or the Pi across the LAN.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloListener(Node):

    def __init__(self):
        super().__init__('hello_listener')

        # Type, topic, callback, QoS depth. The type and topic must match the
        # publisher's; the QoS must be *compatible* (default-vs-default is).
        # A mismatch in any of the three gives no error — just silence, which
        # is why troubleshooting.md exists.
        self.subscription = self.create_subscription(
            String, 'hello', self.on_hello, 10)

    def on_hello(self, msg):
        self.get_logger().info(f'heard: "{msg.data}"')


def main():
    rclpy.init()
    node = HelloListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
