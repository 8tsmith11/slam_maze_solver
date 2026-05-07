import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped

class TargetPublisherNode(Node):
    def __init__(self):
        super().__init__('target_publisher_node')

        self.grid: list[list[int]] | None = None
        self.map_info: MapMetaData | None = None
        self.pose: tuple[float, float] | None = None

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)
        self.create_subscription(PoseWithCovarianceStamped, 'pose', self.pose_callback, qos)
        self.target_publisher = self.create_publisher(PointStamped, 'next_target', 10)

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        w = msg.info.width
        h = msg.info.height

        self.grid = []
        i = 0
        for _ in range(h):
            row: list[int] = []
            for _ in range(w):
                row.append(msg.data[i])
                i += 1
            self.grid.append(row)

        self.publish_target()

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.publish_target()

    def publish_target(self):
        if self.grid is None or self.pose is None:
            return

        frontier = self.nearest_frontier()
        if frontier is None:
            self.get_logger().info('No frontiers found')
            return

        wx, wy = self.grid_to_world(*frontier)
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = wx
        msg.point.y = wy
        self.target_publisher.publish(msg)

    def nearest_frontier(self) -> tuple[int, int] | None:
        if self.pose is None:
            return None

        rx, ry = self.world_to_grid(*self.pose)
        directions = [(1,0), (-1,0), (0,1), (0,-1)]
        h = len(self.grid)
        w = len(self.grid[0])

        best = None
        best_dist = math.inf

        for gy in range(h):
            for gx in range(w):
                # frontier is a free cell with at least one unknown neighbor
                if self.grid[gy][gx] != 0:
                    continue
                for dx, dy in directions:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < w and 0 <= ny < h and self.grid[ny][nx] == -1:
                        dist = math.hypot(rx - gx, ry - gy)
                        if dist < best_dist:
                            best_dist = dist
                            best = (gx, gy)
                        break
        return best

    def world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        x0 = self.map_info.origin.position.x
        y0 = self.map_info.origin.position.y
        gx = int((wx - x0) / self.map_info.resolution)
        gy = int((wy - y0) / self.map_info.resolution)
        return (gx, gy)

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        x0 = self.map_info.origin.position.x
        y0 = self.map_info.origin.position.y
        wx = x0 + (gx + 0.5) * self.map_info.resolution
        wy = y0 + (gy + 0.5) * self.map_info.resolution
        return (wx, wy)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()