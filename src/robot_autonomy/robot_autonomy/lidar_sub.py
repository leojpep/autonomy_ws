'''
Script to subscribe to LIDAR data produced by the TurtleBot3.
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.sub_callback,
            10)
        self.subscription  # prevent unused variable warning

    def sub_callback(self, msg):
        '''
        Process the LIDAR sensor data.
        '''
        ranges = msg.ranges
        intensities = msg.intensities
        print(f"range_min: {msg.range_min}, range_max: {msg.range_max}")
        

def main():
    rclpy.init()
    sub = LidarSubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
