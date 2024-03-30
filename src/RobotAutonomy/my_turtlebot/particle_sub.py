"""
Incomplete.

Week 6 Task 2: The simulation we are using already uses a particle 
filter to perform localization.

- Set an initial starting point of the robot in RViz
- Create a ROS2 node that
  - Prints the number of particles used
  - Computes the expected position of the robot based on the particles
- Change the number of particles when you launch the simulation
"""

# nav2_msgs/msg/ParticleCloud.msg
# std_msgs/Header header
# 	builtin_interfaces/Time stamp
# 		int32 sec
# 		uint32 nanosec
# 	string frame_id
#
# # Array of particles in the cloud
# Particle[] particles
# 	geometry_msgs/Pose pose
# 		Point position
# 			float64 x
# 			float64 y
# 			float64 z
# 		Quaternion orientation
# 			float64 x 0
# 			float64 y 0
# 			float64 z 0
# 			float64 w 1
# 	float64 weight

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

SUB_NAME = "particle_subscriber"
SUB_MSG = ParticleCloud
SUB_TOPIC = "/particle_cloud"


class ParticleSubscriber(Node):
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
        count = len(msg.particles)
        self.get_logger().info(f"No. of particles: {count}")


def main():
    rclpy.init()
    sub = ParticleSubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
