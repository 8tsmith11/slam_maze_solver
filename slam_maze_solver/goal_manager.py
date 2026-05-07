import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
import math
class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        self.declare_parameter('goal_dist', 0.2)
        self.goal_dist = self.get_parameter('goal_dist').value
        self.current_goal: PointStamped | None = None
        self.x = 0.0
        self.y = 0.0
        self.state = False
        self.spinning = False
        self.spin_start_time = None
        self.SPIN_DURATION = math.pi / 0.5
        self.SPIN_SPEED = 0.5
        self.target_sub = self.create_subscription(PointStamped, 'next_target', self.target_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)  
        self.goal_pub = self.create_publisher(PointStamped, 'active_target', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_goal)
        self.state_timer = self.create_timer(0.1, self.state_callback) 
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
    def scan_callback(self, msg: LaserScan):
        d_sum = 0
        count = 0
        for i, d in enumerate(msg.ranges):
            if math.isinf(d) or math.isnan(d) or d <= 0.0:
                continue
            if d < msg.range_min or d > msg.range_max:
                continue
            d_sum += d
            count += 1
        if count == 0:
            return
        d_avg = d_sum / count
        if d_avg > 1.5:
            self.state = True
    def state_callback(self):
        if not self.state:
            return
        if not self.spinning:
            self.spinning = True
            self.spin_start_time = self.get_clock().now()
        elapsed = (self.get_clock().now() - self.spin_start_time).nanoseconds / 1e9
        if elapsed < self.SPIN_DURATION:
            twist = Twist()
            twist.angular.z = self.SPIN_SPEED
            self.cmd_pub.publish(twist)
        else:
            self.cmd_pub.publish(Twist())
            self.spinning = False
            self.state = False
            self.spin_start_time = None
    def target_callback(self, msg: PointStamped):
        if self.state:
            return
        if self.current_goal is None:
            self.current_goal = msg
            return
        if self.flag():
            self.current_goal = msg
    def flag(self) -> bool:
        if self.current_goal is None:
            return False
        dx = self.current_goal.point.x - self.x
        dy = self.current_goal.point.y - self.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        return distance < self.goal_dist
    def publish_goal(self):
        if self.current_goal is not None and not self.spinning:
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