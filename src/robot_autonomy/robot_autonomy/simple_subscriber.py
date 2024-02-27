'''
Script to create a simple subscriber node.
Remember to add a entry_point into the package's setup.py
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscriber = self.create_subscription(String,'/sub_topic',
            self.sub_callback, self.qos_profile)
        self.subscriber  # prevent unused variable warning

    def sub_callback(self, msg):
        # Process the received message
        self.get_logger().info('I heard: "%s"' % msg.data)

def main():
    rclpy.init()
    sub = SimpleSubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
