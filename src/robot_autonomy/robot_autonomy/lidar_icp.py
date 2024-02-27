"""
Script for TurtleBot3 to localize with LIDAR using 
iterative closest point (ICP) algorithm.

1. Use LIDAR scanner
2. Assume constant velocity model
3. Publish TF with result of odometry
4. Compare with the TF provided by ROS
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
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_icp")
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscriber = self.create_subscription(
            LaserScan, "/scan", self.sub_callback, self.qos_profile
        )

        self.laser_projector = LaserProjection()
        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud", self.qos_profile)

        # LaserScan messages
        self.old_scan: LaserScan = None

    def convert_scan_to_cloud(self, old_scan: LaserScan, new_scan: LaserScan):
        """
        Filter out inf values and convert the LIDAR scan data to np.arrays.

        Args:
            old_scan (LaserScan): The old LIDAR scan data.
            new_scan (LaserScan): The new LIDAR scan data.

        Returns:
            p1, p2 (np.array): point cloud data of old and new scan.
        """
        n_scans = len(new_scan.ranges)  # 360
        p1_ranges = np.array(old_scan.ranges, np.float32)
        p2_ranges = np.array(new_scan.ranges, np.float32)

        # Find indices where the scan data is inf in either scan
        p1_inf_idx = np.isinf(p1_ranges)  # (360,)
        p2_inf_idx = np.isinf(p2_ranges)  # (360,)
        indices = [p1_inf_idx[i] or p2_inf_idx[i] for i in range(n_scans)]

        # Make both ranges zero if either scan contains inf
        p1_ranges[indices] = 0
        p2_ranges[indices] = 0
        # Zero values cannot be deleted yet as they are needed to convert
        # for the angle increment when convering to cartesian.

        # Convert the polar LIDAR scan data to euclidean point cloud data
        angle_min = new_scan.angle_min
        angle_increment = new_scan.angle_increment
        p1 = np.zeros((n_scans, 2))
        p2 = np.zeros((n_scans, 2))
        for i in range(n_scans):
            p1[i, 0] = p1_ranges[i] * np.cos(angle_min + angle_increment * i)
            p1[i, 1] = p1_ranges[i] * np.sin(angle_min + angle_increment * i)
            p2[i, 0] = p2_ranges[i] * np.cos(angle_min + angle_increment * i)
            p2[i, 1] = p2_ranges[i] * np.sin(angle_min + angle_increment * i)

        # Find the indices of the zero points that were inf
        p1_zero_idx = np.where(p1_ranges == 0)[0]
        p2_zero_idx = np.where(p2_ranges == 0)[0]

        # Remove zero points that were inf
        # print("before", p1.shape, p2.shape)
        p1 = np.delete(p1, p1_zero_idx, axis=0)
        p2 = np.delete(p2, p2_zero_idx, axis=0)
        # print("after", p1.shape, p2.shape)

        # 2xN
        p1 = p1.reshape(2, -1)
        p2 = p2.reshape(2, -1)
        assert p1.shape == p2.shape, "p1 and p2 must have the same shape"

        return p1, p2  # (2,n)

    def icp(self, p1, p2):
        """
        Implement the ICP algorithm.

        Args:
            old_cloud (np.array): The old point cloud, 2xN.
            new_cloud (np.array): The new point cloud, 2xN.

        Returns:
            R (np.array): The rotation matrix, 2x2.
            t (np.array): The translation vector, 2x1.
        """
        # Compute the mean
        p1_mu = np.mean(p1, axis=1).reshape(2, 1)
        p2_mu = np.mean(p2, axis=1).reshape(2, 1)

        # Center the point clouds
        p1_c = p1 - p1_mu
        p2_c = p2 - p2_mu

        # Compute the 3x3 cross-covariance matrix
        cov_matrix = p1_c @ p2_c.T
        assert cov_matrix.shape == (2, 2), "Covariance matrix must be 2x2"

        # Compute the SVD of the cross-covariance matrix
        U, S, Vt = np.linalg.svd(cov_matrix)

        # Compute the rotation matrix
        R = U @ Vt

        # Compute the translation vector
        t = p1_mu - R @ p2_mu

        return R, t

    def sub_callback(self, scan):
        """
        Apply ICP algorithm to the LIDAR scan data.
        """
        # Allow for the first scan to be saved in 1st iteration
        if self.old_scan is None:
            self.old_scan = scan
            print("Initialized")
            return

        # Convert the LIDAR scan data to point cloud data
        p1, p2 = self.convert_scan_to_cloud(self.old_scan, scan)

        i = 0
        max_iter = 50
        tolerance = 1e-3
        while i < max_iter:
            i += 1
            # Iterate through the ICP algorithm
            R, t = self.icp(p1, p2)

            # Apply transformation to align the old point cloud
            p1 = R @ p1 + t  # (2,2) @ (2,n) + (2,1) = (2,n)

            # Compute the change in the computed transformation
            change = np.linalg.norm(t)
            if i % 10 == 0:
                self.get_logger().info(f"Iteration {i}, change: {change}")
                self.get_logger().info(f'R:\n"{R}"')

            # Check for convergence
            if change < tolerance:
                self.get_logger().info(
                    f"Converged after {i} iterations, change: {change}"
                )
                break

        if i == max_iter:
            self.get_logger().info(f"Failed to converge after {max_iter} iterations")

        # Save for next iteration
        self.old_scan = scan
        print("Processed LIDAR data")


def main():
    rclpy.init()
    sub = LidarSubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
