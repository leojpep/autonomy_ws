"""
Script to overlay laser scans LIDAR data produced by the TurtleBot3.

- Create an empty 2D map
- Subscribe to the LIDAR topic and convert the LIDAR scan to Euclidean coordinates
- Add them to your internal map representation
- Publish the updated map
"""
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData

SUB_NAME = "map_lidar"
SUB_MSG = LaserScan
SUB_TOPIC = "/scan"

PUB_MSG = OccupancyGrid
PUB_TOPIC = "/map2"
PUB_FREQ = 0.5


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__(SUB_NAME)
        scan_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        map_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.subscription = self.create_subscription(
            SUB_MSG, SUB_TOPIC, self.sub_callback, scan_profile
        )

        self.publisher = self.create_publisher(PUB_MSG, PUB_TOPIC, map_profile)

        self.grid = None

    def convert_scan_to_cloud(self, scan: LaserScan):
        """
        Convert the LIDAR scan data to euclidean np.arrays.

        Args:
            scan (LaserScan): The old LIDAR scan data.

        Returns:
            p (np.array): The point cloud data.
        """
        n_scans = len(scan.ranges)  # 360
        ranges = np.array(scan.ranges, np.float32)

        # Find indices where the scan data is inf
        inf_idx = np.isinf(ranges)
        ranges[inf_idx] = 0

        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        p = np.zeros((n_scans, 2))
        for i in range(n_scans):
            p[i, 0] = ranges[i] * np.cos(angle_min + angle_increment * i)
            p[i, 1] = ranges[i] * np.sin(angle_min + angle_increment * i)

        # Remove values from inf range
        p = np.delete(p, inf_idx, axis=0)
        return p

    def draw_point(self, r, theta, map_meta: MapMetaData):
        # Draw free points from robot to point
        if r > 300:
            return
        for t in np.arange(0, theta, map_meta.resolution):
            x = r * np.cos(t)
            y = r * np.sin(t)
            grid_x = int(x / map_meta.resolution)
            grid_y = int(y / map_meta.resolution)
            self.grid[grid_x, grid_y] = 0

        # Draw obstacle at the end of ray
        self.grid[grid_x, grid_y] = 100

    def sub_callback(self, scan: LaserScan):
        """
        Process the LIDAR sensor data.
        """
        # Convert the LIDAR scan data to euclidean array
        # p = self.convert_scan_to_cloud(scan)

        # Prepare map metadata
        map_meta = MapMetaData()
        map_meta.height = 100
        map_meta.width = 100
        map_meta.resolution = 0.1  # m / cell
        map_meta.origin.orientation.x = 0.0
        map_meta.origin.orientation.y = 0.0
        map_meta.origin.orientation.z = -0.7071068
        map_meta.origin.orientation.w = 0.7071068
        map_meta.origin.position.x = 7.0
        map_meta.origin.position.y = -5.0
        map_meta.origin.position.z = 0.0

        # Initialise robot position in the map in grid coordinates
        robot_x = int(map_meta.origin.position.x / map_meta.resolution)
        robot_y = int(map_meta.origin.position.y / map_meta.resolution)
        # robot_x = 0
        # robot_y = 0

        # Initialise grid as unexplored
        map_size = (map_meta.width, map_meta.height)
        self.grid = np.full(map_size, -1, dtype=np.int8)  # 2D array

        # Draw points
        ranges = scan.ranges
        inc = scan.angle_increment
        angles = [inc * i for i in range(len(ranges))]
        for r, a in zip(ranges, angles):
            self.draw_point(r, a, map_meta)

        # # Denote obstacle in a 2D grid
        # for x, y in p:
        #     # x,y are in metres
        #     # self.get_logger().info(f"Processing point {x}, {y}", once=True)

        #     # Convert to grid coordinates
        #     grid_x = int(x / map_meta.resolution)
        #     grid_y = int(y / map_meta.resolution)

        #     self.get_logger().info(f"({x:3f},{y:3f} -> ({grid_x},{grid_y})", once=True)

        #     # Mark robot position
        #     self.grid[robot_x,  robot_y] = 0

        #     # Mark free area along the ray
        #     free_area = bresenham((robot_x, robot_y), (grid_x, grid_y))
        #     for free_x, free_y in free_area:
        #         if x < map_meta.width and y < map_meta.height:
        #             self.grid[robot_x+free_x, robot_y+free_y] = 0

        #     # Mark obstacle at end of ray
        #     if grid_x < map_meta.width and grid_y < map_meta.height:
        #         self.grid[robot_x + grid_x, robot_y + grid_y] = 100

        # Publish message
        msg = OccupancyGrid()
        msg.info = map_meta
        msg.data = self.grid.flatten().astype(np.int8).tolist()
        msg.header.frame_id = "map"
        self.publisher.publish(msg)
        self.get_logger().info("Published map from LIDAR", throttle_duration_sec=1)


# def bresenham(start, end):
#     """
#     Implementation of Bresenham's line drawing algorithm
#     See en.wikipedia.org/wiki/Bresenham's_line_algorithm
#     Bresenham's Line Algorithm
#     Produces a np.array from start and end including.

#     (original from roguebasin.com)
#     >> points1 = bresenham((4, 4), (6, 10))
#     >> print(points1)
#     np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
#     """
#     # setup initial conditions
#     x1, y1 = start
#     x2, y2 = end
#     dx = x2 - x1
#     dy = y2 - y1
#     is_steep = abs(dy) > abs(dx)  # determine how steep the line is
#     if is_steep:  # rotate line
#         x1, y1 = y1, x1
#         x2, y2 = y2, x2
#     # swap start and end points if necessary and store swap state
#     swapped = False
#     if x1 > x2:
#         x1, x2 = x2, x1
#         y1, y2 = y2, y1
#         swapped = True
#     dx = x2 - x1  # recalculate differentials
#     dy = y2 - y1  # recalculate differentials
#     error = int(dx / 2.0)  # calculate error
#     y_step = 1 if y1 < y2 else -1
#     # iterate over bounding box generating points between start and end
#     y = y1
#     points = []
#     for x in range(x1, x2 + 1):
#         coord = [y, x] if is_steep else (x, y)
#         points.append(coord)
#         error -= abs(dy)
#         if error < 0:
#             y += y_step
#             error += dx
#     if swapped:  # reverse the list if the coordinates were swapped
#         points.reverse()
#     points = np.array(points)
#     return points


def main():
    rclpy.init()
    sub = LidarSubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
