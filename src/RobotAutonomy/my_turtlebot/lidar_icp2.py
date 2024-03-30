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
from scipy.spatial import KDTree
from sklearn.neighbors import NearestNeighbors


CONVERGE_TOLERANCE = 1e-5


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
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/cloud", self.qos_profile)

        # LaserScan messages
        self.old_scan: LaserScan = None

    def add_homogeneous(self, p):
        """
        Add homogeneous coordinate to a 2xN point cloud data.

        Returns (3,n) array.
        """
        return np.vstack((p, np.ones(p.shape[1])))  # (3,n)

    def convert_scan_to_cloud(self, old_scan: LaserScan, new_scan: LaserScan):
        """
        Filter out inf values and convert the LIDAR scan data to np.arrays.
        Returns inhomogenous arrays of the cloud scans.

        Args:
            old_scan (LaserScan): The old LIDAR scan data.
            new_scan (LaserScan): The new LIDAR scan data.

        Returns:
            p1, p2 (np.array): point cloud data of old and new scan, 2xN.
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
        p1 = np.delete(p1, p1_zero_idx, axis=0)
        p2 = np.delete(p2, p2_zero_idx, axis=0)

        # Homogenous 2xN
        p1 = p1.reshape(2, -1)
        p2 = p2.reshape(2, -1)
        assert p1.shape == p2.shape, "p1 and p2 must have the same shape"

        return p1, p2  # (2,n)
    

    def icp(self, A, B):
        '''
        Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
        Input:
        A: Nxm numpy array of corresponding points
        B: Nxm numpy array of corresponding points
        Returns:
        T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
        R: mxm rotation matrix
        t: mx1 translation vector
        '''

        # get number of dimensions
        m = A.shape[1]

        # translate points to their centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B

        # rotation matrix
        H = np.dot(AA.T, BB)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # translation
        t = centroid_B.T - np.dot(R,centroid_A.T)

        # homogeneous transformation
        T = np.identity(m+1)
        T[:m, :m] = R
        T[:m, m] = t

        return T, R, t
        
    def nearest_neighbor(self, src, dst):
        '''
        Find the nearest (Euclidean) neighbor in dst for each point in src
        Input:
            src: Nxm array of points
            dst: Nxm array of points
        Output:
            distances: Euclidean distances of the nearest neighbor
            indices: dst indices of the nearest neighbor
        '''

        neigh = NearestNeighbors(n_neighbors=1)
        neigh.fit(dst)
        distances, indices = neigh.kneighbors(src, return_distance=True)
        return distances.ravel(), indices.ravel()


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
        m = p1.shape[0]
        p1 = self.add_homogeneous(p1)
        p2 = self.add_homogeneous(p2)

        # Main ICP loop
        T_overall = np.eye(3)
        i = 0
        max_iter = 100
        prev_error = 0
        while i < max_iter:
            i += 1
            # Iterate through the ICP algorithm
            T_temp, R, t = self.icp(p1[:m,].T, p2[:m,].T)

            # Apply transformation to align the old point cloud
            p1 = np.dot(T_temp, p1)

            # Update the overall transformation
            T_overall = T_temp @ T_overall

            # find the nearest neighbors between the current source and destination points
            distances, indices = self.nearest_neighbor(p1[:m,:].T, p2[:m,:].T)

            # check error
            mean_error = np.mean(distances)

            if np.abs(prev_error - mean_error) < CONVERGE_TOLERANCE:
                self.get_logger().info(
                    f"Converged after {i} iterations, mean_error: {mean_error:8f}"
                )
                break
            prev_error = mean_error

        if i == max_iter:
            self.get_logger().info(
                f"Failed to converge after {max_iter} iterations\nT:\n{T_overall}")

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
