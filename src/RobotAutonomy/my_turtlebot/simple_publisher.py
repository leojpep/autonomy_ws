"""
Script to create a simple publisher node.
Remember to add a entry_point into the package's setup.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

PUB_NAME = "simple_publisher"
PUB_MSG = String
PUB_TOPIC = "/pub_topic"
PUB_FREQ = 5


class SimplePublisher(Node):
    def __init__(self):
        super().__init__(PUB_NAME)
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.publisher = self.create_publisher(PUB_MSG, PUB_TOPIC, self.qos_profile)
        self.timer = self.create_timer(PUB_FREQ, self.timer_callback)

    def timer_callback(self):
        # Publish the message
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    pub = SimplePublisher()
    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
