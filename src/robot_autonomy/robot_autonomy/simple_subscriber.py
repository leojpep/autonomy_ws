"""
Script to create a simple subscriber node.
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

SUB_NAME = "simple_subscriber"
SUB_MSG = String
SUB_TOPIC = "/sub_topic"


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__(SUB_NAME)
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.subscriber = self.create_subscription(
            SUB_MSG, SUB_TOPIC, self.sub_callback, self.qos_profile
        )
        self.subscriber  # prevent unused variable warning

    def sub_callback(self, msg):
        # Process the received message
        self.get_logger().info(f"I heard: {msg.data}")


def main():
    rclpy.init()
    sub = SimpleSubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
