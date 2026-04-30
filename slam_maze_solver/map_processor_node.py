import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, PoseArray
from visualization_msgs.msg import Marker
import numpy as np
from scipy.ndimage import label, center_of_mass

class MapProcessorNode(Node):
    def __init__(self):
        super().__init__('map_processor_node')

        # Publishers and Subscribers
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
        self.target_publisher = self.create_publisher(Point, '/active_target', 10)
        self.graph_publisher = self.create_publisher(PoseArray, '/graph_nodes', 10)
        self.edge_publisher = self.create_publisher(Marker, '/graph_edges', 10)

        # Map Data
        self.grid = None
        self.resolution = -1 # cell size in meters, initialize on first map callback
        self.origin = None 

        # Graph for path planning
        self.nodes = []
        self.graph = {}
        self.visited = set()
        self.cell_threshold = 10 # cluster radius to group cells into a graph node
        self.arrival_threshold = 4 # robot arrives at node when it is this many cells away

        # Robot Info
        self.robot_radius = 6 # Turtlebot radius in cells (for avoiding wall collision)
        self.robot_x = 0.0
        self.robot_y = 0.0

    def map_callback(self, msg: OccupancyGrid):
        # Initialize cell size, origin
        if self.resolution == -1:
            self.resolution = msg.info.resolution
        if self.origin is None:
            self.origin = msg.info.origin
        
        # Update map grid
        w, h = msg.info.width, msg.info.height
        self.grid = np.array(msg.data, dtype=np.int8).reshape((h, w))

        # Get robot position on graph

        # If graph is empty, create the first node
        current_node_index = self.get_current_node_index()
        if current_node_index is None and not self.nodes:
            self.nodes.append((self.robot_x, self.robot_y))
            self.graph[0] = []
            current_node_index = 0

        # Only make new nodes when at a node
        if current_node_index is None:
            return 

        # Find frontier cells (empty cells bordering undiscovered cells)
        # Frontier cells are at least robot radius cells away from a wall (can be moved to)
        frontier_cells = []

        rows, cols = self.grid.shape
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if self.grid[r][c] == 0 and (
                        self.grid[r - 1][c] == -1 or
                        self.grid[r + 1][c] == -1 or
                        self.grid[r][c - 1] == -1 or
                        self.grid[r][c + 1] == -1
                ):
                    if self.is_safe_distance(r, c):
                        frontier_cells.append((r, c))

        # Group nearby frontier cells into clusters
        clusters = []
        for r, c in frontier_cells:
            added_to_cluster = False
            for cluster in clusters:
                center_y = cluster['sum_r'] / cluster['count']
                center_x = cluster['sum_c'] / cluster['count']

                dist = np.hypot(r - center_y, c - center_x)

                if dist < self.cell_threshold:
                    cluster['sum_r'] += r
                    cluster['sum_c'] += c
                    cluster['count'] += 1
                    added_to_cluster = True
                    break
            # Make a new cluster if grid[r][c] didn't belong to any
            if not added_to_cluster:
                clusters.append({'sum_r': r, 'sum_c': c, 'count': 1})

        # Convert cluster centroids to world coordinates and update graph
        for cluster in clusters:
            final_r = cluster['sum_r'] / cluster['count']
            final_c = cluster['sum_c'] / cluster['count']
            
            world_x = (final_c * self.resolution) + self.origin.position.x
            world_y = (final_r * self.resolution) + self.origin.position.y
            
            # Only update graph if node is new
            is_new = True
            for nx, ny in self.nodes:
                if np.hypot(world_x - nx, world_y - ny) < (self.cell_threshold * self.resolution):
                    is_new = False
                    break
            
            if is_new:
                new_index = len(self.nodes)
                self.nodes.append((world_x, world_y))

                self.graph[new_index] = [current_node_index]
                self.graph[current_node_index].append(new_index)

        self.publish_graph()
        self.publish_edges()
        self.publish_target()
    
    def publish_target(self):
        if self.nodes:
            # TODO
            # Picks the first node for now
            target = self.nodes[0]
            msg = Point()
            msg.x = float(target[0])
            msg.y = float(target[1])
            msg.z = 0.0
            self.target_publisher.publish(msg)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    # Return True if grid[r][c] is far enough from a wall to move to
    def is_safe_distance(self, r: int, c: int):
        for y in range(r - self.robot_radius, r + self.robot_radius + 1):
            for x in range(c - self.robot_radius, c + self.robot_radius + 1):
                if (0 <= y < self.grid.shape[0] and 0 <= x < self.grid.shape[1]):
                    if (self.grid[y][x] == 100):
                        return False
        return True
    
    # Returns index of the node the robot is occupying, or None
    def get_current_node_index(self):
        if not self.nodes or self.resolution <= 0:
            return None

        # Convert robot world position to grid position
        robot_r = (self.robot_y - self.origin.position.y) / self.resolution
        robot_c = (self.robot_x - self.origin.position.x) / self.resolution

        best_index = None
        min_dist = float('inf')

        for i, (nx, ny) in enumerate(self.nodes):
            node_r = (ny - self.origin.position.y) / self.resolution
            node_c = (nx - self.origin.position.x) / self.resolution
            
            dist = np.hypot(robot_r - node_r, robot_c - node_c)
            if dist < min_dist:
                min_dist = dist
                best_index = i

        return best_index if min_dist <= self.arrival_threshold else None
    
    def publish_graph(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        for nx, ny in self.nodes:
            p = Pose()
            p.position.x, p.position.y = float(nx), float(ny)
            msg.poses.append(p)
        self.graph_publisher.publish(msg)

    def publish_edges(self):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.type = Marker.LINE_LIST
        msg.action = Marker.ADD
        msg.scale.x = 0.03  # Line width
        msg.color.g, msg.color.b, msg.color.a = 1.0, 1.0, 1.0

        for start_idx, neighbors in self.graph.items():
            p1 = self.nodes[start_idx]
            for n_idx in neighbors:
                p2 = self.nodes[n_idx]
                # Add pair of points for a single line segment
                msg.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=0.0))
                msg.points.append(Point(x=float(p2[0]), y=float(p2[1]), z=0.0))
                
        self.edge_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()