"""
Next-best-view exploration with probabilistic roadmap.

- Reject nodes generated in unknown space
- Compute visible cells with ray casting
- Assume straight path between robot and next node
- Use ros action to move robot:
    ros2 action send_goal /navigate_to_pose <topic> <msg>
"""
import rclpy
import numpy as np
import math
from collections import defaultdict
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from skimage.morphology import dilation, disk
from nav_msgs.msg import OccupancyGrid, Odometry
import matplotlib.pyplot as plt
from nav2_msgs.action import NavigateToPose

np.random.seed(8)

NODE_NAME = "prm"

MAP_MSG = OccupancyGrid
MAP_TOPIC = "/map"

ODOM_MSG = Odometry
ODOM_TOPIC = "/odom"


class PRM(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        map_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        odom_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            MAP_MSG, MAP_TOPIC, self.map_callback, map_profile
        )

        self.odom_sub = self.create_subscription(
            ODOM_MSG, ODOM_TOPIC, self.odom_callback, odom_profile
        )

        # Variables to create map
        self.nodes = [] # [(x1, y1), (x2, y2), ...]
        self.edges = defaultdict(list)  # {node_idx: [(neighbor_idx, distance), ...]}

        # PRM parameters
        self.num_nodes = 20    # Number of nodes to generate
        self.num_neighbors = 3  # Number of nearest neighbors
        self.weight = 1         # Constant cost value

        # Start and goal positions
        self.start = [50, 30]
        self.start_idx = None
        self.goal = [270, 40]
        self.goal_idx = None

        # Next-best-view parameters
        self.i_view = 0
        self.num_views = 10
        self.prev_gain = 0
        self.lambda_ = 1
        self.max_range = 3.5/2      # max_range of LaserScan msg
        self.history = []

        self.robot_x = None
        self.robot_y = None
        self.navigating = False  # flag to check if robot is navigating/moving

        # Action client for /navigate_to_pose
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.action_client.wait_for_server()

    ########################################################
    ################ MAIN CALLBACK FUNCTION ################
    ########################################################

    def map_callback(self, msg: OccupancyGrid) -> None:
        # print("[PRM] Map received.")
        # Process the received message
        self.map = msg
        self.map_data = msg.data  # contains the original map_data
        self.map_info = msg.info
        self.map_metadata = msg.info
        self.map_resolution = self.map_metadata.resolution
        self.map_width = self.map_metadata.width
        self.map_height = self.map_metadata.height
        self.map_origin = self.map_metadata.origin

        self.dilated_map = self.dilate_map(self.map_data, clearance=0.2)  # for path clearance

        self.current_node = (int(-7.5/self.map_resolution), 
                             (-7.5/self.map_resolution))

        # Execute the first time the map is received
        while (self.i_view < self.num_views and
               self.robot_x is not None and
               self.navigating is False):
            # self.create_prm()
            self.i_view += 1
            self.next_best_view()

    def odom_callback(self, msg: Odometry) -> None:
        """
        Update the robot's pose.
        """
        # Convert from world to grid coordinates
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    #######################################################
    ################ PROBABILISTIC ROADMAP ################
    #######################################################
    
    def create_prm(self) -> None:
        """
        Create the Probabilistic Roadmap (PRM) by generating random nodes
        and connecting the k nearest neighbors.
        """
        print("[PRM] Creating roadmap...")
        self.generate_random_nodes(self.num_nodes)
        # self.connect_k_nearest_neighbors(self.num_neighbors)  # choose either this
        self.create_k_nearest_edges(self.num_neighbors)     # or this
        # self.add_start_and_goal(self.start, self.goal)
        # distances, path = dijkstra(self.edges, self.start_idx, self.goal_idx)
        # self.plot_prm(path)
    
    def node_distance(self, node1, node2) -> float:
        """
        Calculate the Euclidean distance between two nodes.

        Args:
            node1 (Tuple[int, int]): The first node position
            node2 (Tuple[int, int]): The second node position

        Returns:
            dist (float): The Euclidean distance between the nodes
        """
        graph_dist = math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)
        real_dist = self.map_resolution * graph_dist
        return real_dist

    def node_is_unknown(self, node) -> bool:
        """
        Check if the node is in unknown space in the map.
        """
        x, y = node
        if self.map_data[y * self.map_width + x] == -1:
            return True
        return False

    def node_is_free(self, node) -> bool:
        """
        Check if the node is in free space in the map.
        """
        x, y = node
        if self.map_data[y * self.map_width + x] == 0:
            return True
        return False
        
    def node_collides_obstacle(self, node) -> bool:
        """
        Check if the node collides with an obstacle in the map.
        """
        x, y = node
        if self.dilated_map[y * self.map_width + x] > 0:
            return True
        return False
    
    def path_collides_obstacle(self, node1:tuple, node2:tuple) -> bool:
        """
        Check if the path between two nodes collides with an obstacle 
        in the map using Bresenham's line algorithm.
        """
        path = bresenham(node1, node2)
        for point in path:
            if self.dilated_map[point[1] * self.map_width + point[0]] > 0:
                return True
        return False

    def generate_random_nodes(self, num_nodes:int) -> None:
        """
        Uniformly generate random nodes in the map space.

        self.nodes = [(x1, y1), (x2, y2), ...]

        Args:
            num_nodes (int): Number of nodes to generate
        """
        self.get_logger().debug("Generating random nodes...")
        nodes = []
        for _ in range(num_nodes):
            valid = False
            while not valid:
                x = np.random.randint(0, self.map_width)
                y = np.random.randint(0, self.map_height)
                node = (x, y)
                if (not self.node_collides_obstacle(node) and 
                   not self.node_is_unknown(node)):
                    valid = True
            nodes.append(node)
        self.nodes = nodes
    
    def sorted_neighbors(self, node) -> list:
        """
        Sort neighbors based on increasing euclidean distance
        from a given node.

        Returns:
            List[Tuple[float, int]] : List of distances and indices of the neighbors 
                                      of length n
        """
        distances = []
        for i, other_node in enumerate(self.nodes):
            if node != other_node:
                dist = math.sqrt((node[0] - other_node[0]) ** 2 + (node[1] - other_node[1]) ** 2)
                distances.append((dist, i))
        return sorted(distances)
    
    def find_k_nearest_neighbors(self, nodes, k:int) -> dict:
        """
        Finds indices of the K nearest neighbors for each node.

        Args:
            nodes (List[Tuple[float, float]]): List of the nodes positions
            k (int): Number of nearest nodes to connect to

        Returns:
            dict: dictionary where the key-value pairs are:  
                node_idx: [neighbor_1_idx, neighbor_2_idx, ..., neighbor_k_idx]
        """
        node_neighbors = defaultdict(list)

        for i, node1 in enumerate(nodes):
            k_nearest = self.sorted_neighbors(node1)[:k]
            for _, neighbor_idx in k_nearest:
                node_neighbors[i].append(neighbor_idx)
        return node_neighbors

    def connect_k_nearest_neighbors(self, k:int) -> None:
        """
        Connect k nearest neighbors with obstacle free paths.

        self.edges = {node_idx: [(neighbor_idx, distance), ...]}

        Args:
            nodes (List[Tuple[float, float]]): List of the nodes positions
            k (int): Number of nearest nodes to connect to
        """
        self.get_logger().debug("Creating edges...")
        for i, node1 in enumerate(self.nodes):
            k_nearest = self.sorted_neighbors(node1)[:k]
            for _, neighbor_idx in k_nearest:
                # If path is free, calculate distance and create the edge
                if not self.path_collides_obstacle(node1, self.nodes[neighbor_idx]):
                    dist = self.weight
                    if (neighbor_idx, dist) not in self.edges[i]:
                        self.edges[i].append((neighbor_idx, dist))
                    if (i, dist) not in self.edges[neighbor_idx]:
                        self.edges[neighbor_idx].append((i, dist))
    
    def create_k_nearest_edges(self, k:int) -> None:
        """
        Create k edges between the nearest neighbors with obstacle free paths.

        self.edges = {node_idx: [(neighbor_idx, distance), ...]}

        Args:
            k (int): Number of shortest edges to connect to
        """
        self.get_logger().debug("Creating edges...")
        node_neighbors = defaultdict(list)
        for i, node1 in enumerate(self.nodes):
            sorted_neighbors = self.sorted_neighbors(node1)

            # while there are less than k neighbors & unvisited neighbors remaining
            while len(node_neighbors[i]) < k and sorted_neighbors:
                neighbor_idx = sorted_neighbors.pop(0)[1]
                # If path is free, calculate distance and create the edge
                if not self.path_collides_obstacle(node1, self.nodes[neighbor_idx]):
                    dist = self.weight
                    if (neighbor_idx, dist) not in self.edges[i]:
                        self.edges[i].append((neighbor_idx, dist))
                    if (i, dist) not in self.edges[neighbor_idx]:
                        self.edges[neighbor_idx].append((i, dist))
                    node_neighbors[i].append(neighbor_idx)


    def add_start_and_goal(self, start, goal=None) -> None:
        """
        Add start and goal points to the roadmap.

        Args:
            start (Tuple[float, float]): Start position
            goal (Tuple[float, float]): Goal position
        """
        self.get_logger().debug("Adding start and/or goal nodes...")

        if self.node_collides_obstacle(start):
            print(f"start: {start}")
            raise ValueError("Start node collides with an obstacle.")
        self.start_idx = len(self.nodes)
        self.nodes.append(start)

        if goal is not None:
            if self.node_collides_obstacle(goal):
                raise ValueError("Goal node collides with an obstacle.")
            self.goal_idx = len(self.nodes)
            self.nodes.append(goal)

        # Find nearest node
        neighbours = self.find_k_nearest_neighbors(self.nodes, self.num_neighbors)
        
        for neighbour_idx in neighbours[self.start_idx]:
            d_start = self.weight
            self.edges[neighbour_idx].append((self.start_idx, d_start))
            self.edges[self.start_idx].append((neighbour_idx, d_start))

        if goal is not None:
            for neighbour_idx in neighbours[self.goal_idx]:
                d_goal = self.weight
                self.edges[neighbour_idx].append((self.goal_idx, d_goal))
                self.edges[self.goal_idx].append((neighbour_idx, d_goal))
    
    def dilate_map(self, map_data, clearance) -> list:
        """
        Dilate the obstacles on the map to replicate path clearance.

        Args:
            map_data (List[int]): The map data representing obstacles.
            clearance (float): The clearance value to add around obstacles in metres.

        Returns:
            List[int]: The dilated map data.
        """

        # Convert map_data to a 2D array
        map_array = np.array(map_data).reshape((self.map_height, self.map_width))
        
        # Dilate to add clearance around obstacles
        disk_radius = int(clearance / self.map_resolution)
        dilated_map = dilation(map_array, disk(disk_radius))
 
        return dilated_map.ravel().tolist() # back to a 1D array

    def plot_prm(self, path=None) -> None:
        """
        Plot the roadmap with the nodes, edges, clearance, obstacles,
        start, and goal positions.
        """
        print("[PRM] Plotting roadmap...")
        plt.figure(dpi=120)
        
        # Plot edges
        for node_idx, edges in self.edges.items():
            for edge in edges:
                node1 = self.nodes[node_idx]
                node2 = self.nodes[edge[0]]
                plt.plot([node1[0], node2[0]], [node1[1], node2[1]], "b-", label="edges")

        # Plot path
        if path:
            for i in range(len(path)-1):
                node1 = self.nodes[path[i]]
                node2 = self.nodes[path[i+1]]
                plt.plot([node1[0], node2[0]], [node1[1], node2[1]], "r-", linewidth=2)

        # Plot nodes
        for i, node in enumerate(self.nodes):
            plt.plot(node[0], node[1], "ro", markersize=3)
            # plt.text(node[0], node[1], str(i), ha='center', va='center')  # index
        
        # Plot clearance
        x_list, y_list = [], []
        for i,value in enumerate(self.dilated_map):
            if value > 0:
                x_list.append(i % self.map_width)
                y_list.append(i // self.map_width)
        plt.scatter(x_list, y_list, color='yellow', s=1)

        # Plot obstacles
        x_list, y_list = [], []
        for i,value in enumerate(self.map_data):
            if value > 0:
                x_list.append(i % self.map_width)
                y_list.append(i // self.map_width)
        plt.scatter(x_list, y_list, color='black', s=1)
        
        # Plot start and goal
        if self.start_idx is not None:
            plt.plot(self.start[0], self.start[1], color='green', marker='x', markersize=10)
        if self.goal_idx is not None:    
            plt.plot(self.goal[0], self.goal[1], color='#fca103', marker='x', markersize=10)

        # plt.title(f"Probabilistic Roadmap, n={len(self.nodes)} nodes, k={self.num_neighbors} neighbors")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.show()
        print("[PRM] Plotting done.")
    
    ################################################
    ################ NEXT BEST VIEW ################
    ################################################

    def node_visibility(self, node, plot=False) -> int:
        """
        Calculate the number unknown cells in view, with an option 
        to plot the map with unknown, free, and obstacle cells.

        Args:
            node (Tuple[float, float]): The node position

        Returns:
            gain (int): Number of unknown cells reached
        """
        x, y = node
        unknown_cells = set()
        free_cells = set()
        obstacle_cells = set()
        for angle in range(0, 360):
            point_x = x + self.max_range/self.map_resolution * math.cos(math.radians(angle))
            point_y = y + self.max_range/self.map_resolution * math.sin(math.radians(angle))
            point = (int(point_x), int(point_y))
            ray = bresenham(node, point)
            for point in ray:
                point = (int(point[0]), int(point[1]))
                try:
                    if self.map_data[point[1] * self.map_width + point[0]] == -1:
                        unknown_cells.add(point)
                    if self.map_data[point[1] * self.map_width + point[0]] == 0:
                        free_cells.add(point)
                    if self.map_data[point[1] * self.map_width + point[0]] > 0:
                        obstacle_cells.add(point)
                        break    # stop ray casting, go to next angle
                except IndexError:  # when the point is out of the map, go to next angle
                    break
        # print(f"[NBV] {node}: {len(unknown_cells)} unknown, {len(free_cells)} free, {len(obstacle_cells)} obstacles")
        
        if plot:
            self.plot_cell_types(node, unknown_cells, free_cells)
        
        return len(unknown_cells)

    def plot_cell_types(self, node, unknown_cells, free_cells) -> None:
        """
        Plot the information gain at a node in a map for next best view
        by highlighting the cell types as unknown, free, or obstacle.

        Args:
            node (Tuple[float, float]): The node position
            unknown_cells (set): Set of unknown cells
            free_cells (set): Set of free cells
            obstacle_cells (set): Set of obstacle cells
        """
        plt.figure(dpi=120)
        for cell in unknown_cells:
            plt.plot(cell[0], cell[1], color="grey", marker=".", markersize=1)
        for cell in free_cells:
            plt.plot(cell[0], cell[1], color="blue", marker=".", markersize=1)
        # Circle to visualize max range
        radius = self.max_range/self.map_resolution
        circle = plt.Circle(node, radius, color='red', fill=False)
        plt.gca().add_artist(circle)
        # Add all obstacles
        x_list = []
        y_list = []
        for i,value in enumerate(self.map_data):
            if value > 0:
                x = i % self.map_width
                y = i // self.map_width
                x_list.append(x)
                y_list.append(y)
        plt.scatter(x_list, y_list, color='black', s=1)        
        
        plt.plot(node[0], node[1], "ro", markersize=3)
        plt.title(f"Node {node}")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.axis('scaled')
        plt.show()

    def plot_all_visibility(self) -> None:
        """
        Plot the visibility for all nodes.
        """
        print(f"[NBV] Finding visibility for all nodes...")
        free_cells = dict()
        for x in range(self.map_width):
            for y in range(self.map_height):
                node = (x, y)
                if self.map_data[y * self.map_width + x] == 0:
                    free_cells[node] = self.node_visibility(node)

        print("[NBV] Plotting visibility heatmap...")
        plt.figure(dpi=120)

        # Plot obstacles
        x_list, y_list = [], []
        for i,value in enumerate(self.map_data):
            if value > 0:
                x_list.append(i % self.map_width)
                y_list.append(i // self.map_width)
        plt.scatter(x_list, y_list, color='black', s=1)

        # Plot the heatmap
        x_values = [node[0] for node in free_cells.keys()]
        y_values = [node[1] for node in free_cells.keys()]
        info_gain = list(free_cells.values())
        plt.scatter(x_values, y_values, c=info_gain, cmap='RdYlGn_r', s=1)
        plt.colorbar(label='Visibility')
        plt.title('Visibility Heatmap')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.show()
        print(f"[NBV] Visibility heatmap plotted.")

    def plot_info_gain(self, gains) -> None:
        """
        Plot the information gain for all nodes.
        """
        print("[NBV] Plotting information gain heatmap...")
        plt.figure(dpi=120)

        # Plot obstacles
        x_list, y_list = [], []
        for i,value in enumerate(self.map_data):
            if value > 0:
                x_list.append(i % self.map_width)
                y_list.append(i // self.map_width)
        plt.scatter(x_list, y_list, color='black', s=1)

        # Plot the heatmap
        x_values = [node[0] for node in self.nodes]
        y_values = [node[1] for node in self.nodes]
        plt.scatter(x_values, y_values, c=gains, cmap='RdYlGn_r', s=10)
        plt.colorbar(label='Information Gain')
        plt.title('Information Gain Heatmap')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.show()
        print("[NBV] Information gain heatmap plotted.")

    def plot_history(self) -> None:
        """
        Plot the path history of the robot from NBV iterations.
        """
        print("[NBV] Plotting history...")
        plt.figure(dpi=120)

        # Plot obstacles
        x_list, y_list = [], []
        for i,value in enumerate(self.map_data):
            if value > 0:
                x_list.append(i % self.map_width)
                y_list.append(i // self.map_width)
        plt.scatter(x_list, y_list, color='black', s=1)

        # Plot the history
        x_values = [node[0] for node in self.history]
        y_values = [node[1] for node in self.history]
        plt.plot(x_values, y_values, "r-", linewidth=2)
        plt.scatter(x_values, y_values, color='red', s=10)

        # Plot the first node
        plt.plot(x_values[0], y_values[0], color='green', marker='x', markersize=10)

        plt.title('Path History')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.show()
        print("[NBV] History plotted.")
    
    def next_best_view(self) -> None:
        """
        Find the next best view node by maximizing the information gain.
        """
        print(f"[NBV] Finding next best view node... Iter: {self.i_view}/{self.num_views}")
        
        # Prepare PRM graph
        self.generate_random_nodes(self.num_nodes)
        self.create_k_nearest_edges(self.num_neighbors)

        # Convert robot pose to grid coordinates
        robot_grid_x = int((self.robot_x - self.map_origin.position.x) / self.map_resolution)
        robot_grid_y = int((self.robot_y - self.map_origin.position.y) / self.map_resolution)
        self.start = (robot_grid_x, robot_grid_y)
        self.add_start_and_goal(self.start)

        # TODO: use RRT instead of BFS
        # NBV algorithm with BFS search
        best_gain = 0
        visited = set()
        queue = []
        gains = [None]*len(self.nodes) 
        parents = [None]*len(self.nodes) # store indices of parent nodes

        # Initialize start node
        parents[self.start_idx] = self.start_idx
        gains[self.start_idx] = 0
        visited.add(self.start_idx)

        # Initialise queue and root as parent
        for neighbor_idx, _ in self.edges[self.start_idx]:
            queue.append(neighbor_idx)
            parents[neighbor_idx] = self.start_idx
            visited.add(neighbor_idx)

        while queue:
            current_idx = queue.pop(0)
            current_node = self.nodes[current_idx]

            # Calculate info gain for current node
            parent_idx = parents[current_idx]
            parent_gain = gains[parent_idx]
            vis = self.node_visibility(current_node)
            weight = self.node_distance(current_node, self.nodes[parent_idx])
            gain = parent_gain + vis*np.exp(-self.lambda_ * weight)
            gains[current_idx] = gain

            # Update best
            if gain > best_gain:
                best_gain = gain
                best_idx = current_idx
            
            # Add neighbors to queue, set current as parent
            for neighbor_idx, _ in self.edges[current_idx]:
                if neighbor_idx not in visited:
                    queue.append(neighbor_idx)
                    visited.add(neighbor_idx)
                    parents[neighbor_idx] = current_idx

        print(f"[NBV] Next best view node: {self.nodes[best_idx]} with gain {best_gain}")

        # Find path to best node
        path = []
        current_idx = best_idx
        while current_idx != self.start_idx:
            # print(f"Parent of {current_idx}: {parents[current_idx]}")
            path.append(current_idx)
            current_idx = parents[current_idx]
        path.append(self.start_idx)
        path.reverse()
        next_node = self.nodes[path[1]] # next node in path
        self.history.append(next_node)
        print(f"[NBV] Path to best node: {path}")

        # Plots
        # self.plot_info_gain(gains)
        # self.plot_prm(path)
        # self.plot_all_visibility()
        if self.i_view == self.num_views:
            self.plot_history()

        self.navigate_to_cell(next_node)

    ##################################################
    ################ NAVIGATE TO POSE ################
    ##################################################
    
    def feedback_callback(self, feedback_msg: NavigateToPose.Feedback):
        # NavigateToPose should have no feedback
        self.get_logger().debug('[NBV] Received feedback')

    def get_result_callback(self, future: NavigateToPose.Result):
        """
        Executed when the navigation movement is completed.
        """
        result = future.result().result
        # Expecting empty result (std_msgs::Empty) for NavigateToPose
        self.get_logger().debug(f'[NBV] Result: {result.result}')
        self.get_logger().info("[NBV] Navigation done.")
        self.navigating = False
    
    def goal_response_callback(self, future):
        """
        Executed when the goal is received.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('[NBV] Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def navigate_to_cell(self, cell) -> None:
        """
        Send a goal to the robot to navigate to the given cell.

        Args:
            cell (Tuple[float, float]): The cell position
        """
        self.navigating = True
        action_goal = NavigateToPose.Goal()
        x = cell[0] * self.map_resolution
        y = cell[1] * self.map_resolution
        action_goal.pose.pose.position.x = x
        action_goal.pose.pose.position.y = y
        action_goal.pose.pose.position.z = 0.0
        print(f"[NBV] Navigating to ({x:.2f}, {y:.2f})...")
    
        # asynchronous call: no blocking (recommended)
        self._send_goal_future = self.action_client.send_goal_async(action_goal,
                                                                    feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        ## synchronous call: block calling thread until response received
        # future = self.action_client.send_goal(action_goal)  


def bresenham(start, end) -> np.ndarray:
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end including.

    (original from roguebasin.com)
    >> points1 = bresenham((4, 4), (6, 10))
    >> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # recalculate differentials
    dy = y2 - y1  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    y_step = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:  # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points


def dijkstra(edges, start_idx, goal_idx):
    '''
    Use dijkstra algorithm to find the shortest path from start to goal.

    Args:
        edges (dict): dictionary where the key-value pairs are:  node_idx: [(neighbor_idx, distance), ...]
        start_idx (int): index of the start node
        goal_idx (int): index of the goal node

    Returns:
        dict: dictionary where the key-value pairs are:  node_idx: distance
        path: list of the node indexes in the shortest path
    '''
    print(f"[Dijkstra] Finding path from {start_idx} to {goal_idx} using Dijkstra algorithm.")

    class Node:
        def __init__(self, idx):
            self.idx = idx
            self.parent = None

    # Initialize the distance to all nodes to be infinity
    distances = {}
    nodes = {}
    for node in edges:
        distances[node] = float("inf")
        nodes[node] = Node(node)

    # Initialize start node
    distances[start_idx] = 0
    nodes[start_idx].parent = start_idx

    queue = [(start_idx, 0)]  # priority queue: store nodes and their distances
    visited = set()           # store visited nodes

    # Iterate through the queue
    while queue:
        current_node, current_distance = queue.pop(0)

        if current_node in visited:
            continue

        # Check if the current node is the goal node
        if current_node == goal_idx:
            print(f"[Dijkstra] Visited {len(visited)} nodes")
            print(f"[Dijkstra] Goal node {goal_idx} found with distance {current_distance}")

            # Reconstruct the path
            path = []
            while current_node != start_idx:
                path.append(current_node)
                current_node = nodes[current_node].parent
            path.append(start_idx)
            path.reverse()
            print(f"[Dijkstra] Path: {path}")
            break

        # Explore the neighbors of the current node
        for neighbor, distance in edges[current_node]:
            # Relax the current node if a lower distance estimate is found
            new_estimate = current_distance + distance
            if new_estimate < distances[neighbor]:
                distances[neighbor] = new_estimate
                nodes[neighbor].parent = current_node
            queue.append((neighbor, distances[neighbor]))

        visited.add(current_node)

        # Sort the queue based on the distances
        queue.sort(key=lambda x: x[1])
    
    return distances, path


def main():   
    rclpy.init()
    prm = PRM()
    rclpy.spin(prm)
    rclpy.shutdown()    

if __name__ == "__main__":
    main()
