"""
Script to create a simple map publisher node.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

PUB_NAME = "map_publisher"
PUB_MSG = OccupancyGrid
PUB_TOPIC = "/map"
PUB_FREQ = 5


class MapPublisher(Node):
    def __init__(self):
        super().__init__(PUB_NAME)
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(PUB_MSG, PUB_TOPIC, self.qos_profile)
        self.timer = self.create_timer(1 / PUB_FREQ, self.timer_callback)

    def timer_callback(self):
        # Publish the message
        msg = OccupancyGrid()
        # msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    pub = MapPublisher()
    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
