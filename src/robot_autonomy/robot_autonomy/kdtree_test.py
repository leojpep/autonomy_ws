"""
Script to test the KDTree algorithm.
tree.query(x) Finds the closest point in the tree to each point in the query x.

python3 src/RobotAutonomy/my_turtlebot/kdtree_test.py 
"""

import numpy as np
from scipy.spatial import KDTree

# Create a KDTree object
x, y = np.mgrid[0:5, 2:8]
old_cloud = np.c_[x.ravel(), y.ravel()]
print(f"old_arr:\n{old_cloud}")
tree = KDTree(old_cloud)

# Query the nearest neighbor
new_cloud = np.array([[1, 3], [4, 4.6], [0, 3]])
dd, ii = tree.query(new_cloud, k=1)

# tree.query(x) Finds the closest point in the tree to each point in the query x.

print(f"distances: {dd}")
print(f"indices: {ii}")
# print(f"selected: {old_arr[ii]}")

matched_points = old_cloud[ii]
print(f"matched_points:\n{matched_points}")

"""
Output:

old_arr: 
[[0 2]
 [0 3]
 [0 4]
 [0 5]
 [0 6]
 [0 7]
 [1 2]
 [1 3]
 [1 4]
 [1 5]
 [1 6]
 [1 7]
 [2 2]
 [2 3]
 [2 4]
 [2 5]
 [2 6]
 [2 7]
 [3 2]
 [3 3]
 [3 4]
 [3 5]
 [3 6]
 [3 7]
 [4 2]
 [4 3]
 [4 4]
 [4 5]
 [4 6]
 [4 7]]
distances: [0.  0.4 0. ]
indices: [ 7 27  1]
matched_points:
[[1 3]
 [4 5]
 [0 3]]
"""
