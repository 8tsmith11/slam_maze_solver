import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import math
class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        self.declare_parameter('goal_dist', 0.2)
        self.goal_dist = self.get_parameter('goal_dist').value
        self.current_goal: PointStamped | None = None
        self.x = 0.0
        self.y = 0.0
        self.target_sub = self.create_subscription(PointStamped, 'next_target', self.target_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_pub = self.create_publisher(PointStamped, 'active_target', 10)
        self.timer = self.create_timer(0.1, self.publish_goal)
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
    def target_callback(self, msg: PointStamped):
        if self.current_goal is None:
            self.current_goal = msg
            return
        if self.flag():
            self.current_goal = msg
    def flag(self) -> bool:
        if self.current_goal is None:
            return False
        dx = self.current_goal.point.x - self.robot_x
        dy = self.current_goal.point.y - self.robot_y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        return distance < self.goal_dist
    def publish_goal(self):
        if self.current_goal is not None:
            self.goal_pub.publish(self.current_goal)
def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()