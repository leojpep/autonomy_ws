'''
Script for TurtleBot3 to localize with LIDAR using 
iterative closest point (ICP) algorithm.

1. Use LIDAR scanner
2. Assume constant velocity model
3. Publish TF with result of odometry
4. Compare with the TF provided by ROS
'''

import rclpy
import copy
import numpy as np
import array as arr

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
from scipy.spatial import KDTree

from .point_cloud2 import *


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_localization')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.sub_callback, self.qos_profile)

        self.laser_projector = LaserProjection()
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/cloud', self.qos_profile)

        # LaserScan messages
        self.old_raw_scan = None
        self.old_proc_scan = None

    def convert_scan_to_cloud(self, scan):
        '''
        Convert the LIDAR scan data to numpy arrays.

        Args:
            scan (LaserScan): The LIDAR scan data.

        Returns:
            clouds: List of numpy arrays of the point cloud data.
        '''
        # # Create variables to store new processed ranges
        # old_proc_scan = self.old_raw_scan
        # new_proc_scan = copy.deepcopy(scan)

        # # Keep values if both scans has finite values
        # old_raw_ranges = np.array(self.old_raw_scan.ranges, np.float32)
        # new_raw_ranges = np.array(scan.ranges, np.float32)
        # old_inf_idx = np.isfinite(old_raw_ranges)  # (360,)
        # new_inf_idx = np.isfinite(new_raw_ranges)  # (360,)
        # indices = [old_inf_idx[i] and new_inf_idx[i]
        #            for i in range(len(old_inf_idx))]    # len=360
        # old_ranges = old_raw_ranges[indices]    # numpy array, about (212,)
        # new_ranges = new_raw_ranges[indices]    # numpy array, about (212,)
        # self.get_logger().debug(f"old_ranges: {old_ranges.shape}")
        # self.get_logger().debug(f"new_ranges: {new_ranges.shape}")

        # # Modify ranges of LaserScan messages
        # old_proc_scan.ranges = arr.array('f', old_ranges)
        # new_proc_scan.ranges = arr.array('f', new_ranges)

        # # Convert sensor_msgs::LaserScan scan -> sensor_msgs::PointCloud2
        # new_cloud = self.laser_projector.projectLaser(new_proc_scan)
        # old_cloud = self.laser_projector.projectLaser(old_proc_scan)
        # BUG: old_cloud_np may defer from old_proc_scan by 1 point

        # Convert sensor_msgs::LaserScan scan -> sensor_msgs::PointCloud2
        new_cloud = self.laser_projector.projectLaser(scan)
        old_cloud = self.laser_projector.projectLaser(self.old_raw_scan)

        # Convert PointCloud2 -> generator -> np arrays
        old_cloud_arr = read_points(
            old_cloud, field_names=("x", "y", "z"), skip_nans=False)  # generator
        new_cloud_arr = read_points(
            new_cloud, field_names=("x", "y", "z"), skip_nans=False)
        old_cloud_np = np.array(list(old_cloud_arr))
        new_cloud_np = np.array(list(new_cloud_arr))
        print(f"old_cloud_np: {old_cloud_np.shape}")
        print(f"new_cloud_np: {new_cloud_np.shape}")

        return [old_cloud_np, new_cloud_np]

    def find_closest_points(self, p1, p2):
        """
        For each point in p2, find the closest point in p1.

        Args:
            p1 (np.array): The old point cloud.
            p2 (np.array): The new point cloud.

        Returns:
            selected_points (np.array): Closest points in p1 to p2.
        """
        tree = KDTree(p1)
        distances, indices = tree.query(p2, k=1)
        # indices[i]: 1-idx of the closest point in p1 to a point in p2

        # self.get_logger().debug(f"indices:\n {indices}")
        # print(f"indices:\n {indices}")
        selected_points = p1[indices]
        return selected_points

    def icp(self, p1, p2):
        """
        Implement the ICP algorithm.

        Args:
            old_cloud (np.array): The old point cloud.
            new_cloud (np.array): The new point cloud.

        Returns:
            R (np.array): The rotation matrix.
            t (np.array): The translation vector.
        """
        # Find closest points
        selected_points = self.find_closest_points(p1, p2)

        # Compute the mean
        p1_mu = np.mean(selected_points, axis=0)
        p2_mu = np.mean(p2, axis=0)

        # Center the point clouds
        # TODO: subtract from mean of selected points or p1?
        p1_c = selected_points - p1_mu
        p2_c = p2 - p2_mu

        # Compute the cross-covariance matrix, 3x3
        cov_matrix = p1_c.T @ p2_c      # TODO: is this correct?

        # Compute the SVD of the cross-covariance matrix
        U, S, Vt = np.linalg.svd(cov_matrix)

        # Compute the rotation matrix
        R = U @ Vt

        # Compute the translation vector
        t = p1_mu - R @ p2_mu

        return R, t

    def sub_callback(self, scan):
        '''
        Apply ICP algorithm to the LIDAR scan data.
        '''
        # Allow for the first scan to be saved in 1st iteration
        if self.old_raw_scan is None:
            self.old_raw_scan = scan
            print("Initialized")
            return

        # Convert the LIDAR scan data to point cloud data
        clouds = self.convert_scan_to_cloud(scan)
        p1 = clouds[0]
        p2 = clouds[1]

        i = 1
        max_iter = 50
        while i <= max_iter:

            # Iterate through the ICP algorithm
            R, t = self.icp(p1, p2)

            # Apply transformation to align the old point cloud
            p1 = p1 @ R.T + t

            # Compute the change in the computed transformation
            change = np.linalg.norm(t)
            print(f'Iteration {i}, change: {change}')

            # Check for convergence
            i += 1
            if change < 1e-7:
                print(f'Converged after {i} iterations')
                break
        
        if i > max_iter:
            print(f'Failed to converge after {max_iter} iterations')

        # Save for next iteration
        self.old_raw_scan = scan
        print('Processed LIDAR data')


def main():
    rclpy.init()
    sub = LidarSubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
