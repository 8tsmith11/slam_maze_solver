import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from collections import deque
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

OBSTACLE_THRESHOLD = 50
LOOKAHEAD_TILES = 20
MAX_COVARIANCE = 0.5

class PremappedNavNode(Node):
    def __init__(self):
        super().__init__('premapped_nav_node')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        self.goal = (goal_x, goal_y)

        self.grid: list[list[bool]] | None = None
        self.map_info: MapMetaData | None = None
        self.path: list[tuple[int, int]] | None = None
        self.path_progress: int = 0
        self.current_target: tuple[float, float] | None = None

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, qos)
        self.target_publisher = self.create_publisher(PointStamped, 'goal_point', 10)
        self.create_timer(0.1, self.publish_target)
        self.get_logger().info('PremappedNavNode started, waiting for map and amcl_pose...')

    def map_callback(self, msg: OccupancyGrid):
        # Only buld the map once
        if self.grid:
            return

        self.map_info = msg.info

        w = msg.info.width
        h = msg.info.height
        

        # The map is sent as a 1D array with width and height info
        # Convert to 2D array
        i = 0
        self.grid = []
        for _ in range(h):
            self.grid.append([])
            for _ in range(w):
                self.grid[-1].append(0 <= msg.data[i] < OBSTACLE_THRESHOLD)
                i += 1

        self.get_logger().info(f'Map built: {w}x{h}, resolution={msg.info.resolution:.4f}')


    def pose_callback(self, msg: PoseWithCovarianceStamped):
        # Wait until map is initialized and localization completes
        if self.grid is None or msg.pose.covariance[0] > MAX_COVARIANCE:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not self.path:
            self.get_logger().info(f'Planning path from ({x:.2f}, {y:.2f}) to {self.goal}')
            self.path = self.bfs(x, y)
            self.get_logger().info(f'Path found with {len(self.path)} waypoints')

        self.current_target = self.get_target(x, y)

    def publish_target(self):
        if self.current_target is None:
            return
        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'map'
        target_msg.point.x = self.current_target[0]
        target_msg.point.y = self.current_target[1]
        self.target_publisher.publish(target_msg)

    def bfs(self, wx, wy) -> list[tuple[int, int]]:
        start = self.world_to_grid(wx, wy)
        goal = self.world_to_grid(*self.goal)
        q: deque[tuple[int, int]] = deque([start])
        prev = {start: None}

        while q:
            x, y = q.popleft()
            # If goal is in queue, it means prev stores a path to it, so bfs is done
            if (x, y) == goal:
                break

            # Check all diagonal neighbors
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    if dx != 0 or dy != 0:
                        tx, ty = x + dx, y + dy
                        tile = (tx, ty)

                        # We already have a path to this tile
                        # Unweighted BFS guarantees it is shorter or equal
                        if tile in prev:
                            continue
                        # Tile is out of bounds, continue
                        if tx < 0 or tx >= len(self.grid[0]) or ty < 0 or ty >= len(self.grid):
                            continue
                        # Tile is a wall, continue
                        if not self.grid[ty][tx]:
                            continue

                        prev[tile] = (x, y)
                        q.append(tile)

        if goal not in prev:
            self.get_logger().error(f'BFS could not reach goal {goal} from {start}')
            return []

        path = []
        current = goal
        while current:
            path.append(current)
            current = prev.get(current)
        path.reverse()
        return path

    # Set potential field nav target to LOOKAHEAD_TILES ahead of closest tile in path to current position
    def get_target(self, wx, wy) -> tuple[float, float]:
        if self.path is None:
            return

        gx, gy = self.world_to_grid(wx, wy)
        distances = [math.hypot(gx - x, gy - y) for (x, y) in self.path]
        closest_index = distances.index(min(distances))
        self.path_progress = max(self.path_progress, closest_index)
        target_index = min(self.path_progress + LOOKAHEAD_TILES, len(self.path) - 1)
        tx, ty = self.path[target_index]
        return self.grid_to_world(tx, ty)

    def world_to_grid(self, wx, wy) -> tuple[int, int]:
        x0 = self.map_info.origin.position.x
        y0 = self.map_info.origin.position.y

        gx = int((wx - x0) / self.map_info.resolution)
        gy = int((wy - y0) / self.map_info.resolution)

        return (gx, gy)

    def grid_to_world(self, gx, gy) -> tuple[float, float]:
        x0 = self.map_info.origin.position.x
        y0 = self.map_info.origin.position.y

        wx = x0 + (gx + 0.5) * self.map_info.resolution
        wy = y0 + (gy + 0.5) * self.map_info.resolution

        return (wx, wy)

def main(args=None):
    rclpy.init(args=args)
    node = PremappedNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
