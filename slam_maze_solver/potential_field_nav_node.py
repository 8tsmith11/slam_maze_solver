import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, PointStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class PotentialFieldNavigationNode(Node):
    def __init__(self):
        super().__init__('potential_field_nav_node')

        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.forward_speed = 0.3 # constant forward velocity
        timer_period = 1.0 / 20.0 # 20 Hz loop rate in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Obstacle Avoidance
        self.repulsive_angular_z = 0.0
        self.k_avoid = 0.15
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data
        )

        # Target Seeking
        # Robot pose source from simulator.
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile_sensor_data
        )

        # Goal point on map
        self.goal_point_sub = self.create_subscription(
            PointStamped, 'goal_point', self.goal_point_callback, 10
        )
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.have_goal = False
        self.goal_tolerance = 0.5
        self.k_heading = 2.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.have_odom = False
        self.max_angular_speed = 1.2

        self.get_logger().info(
            'PotentialFieldNavigationNode started. Ready to receive goal.'
        )

    def scan_callback(self, msg: LaserScan):
        repulsive_sum = 0.0

        for i, d in enumerate(msg.ranges):
            if math.isinf(d) or math.isnan(d) or d <= 0.0:
                continue
            if d < msg.range_min or d > msg.range_max:
                continue

            theta = msg.angle_min + i * msg.angle_increment
            repulsive_sum += (-1.0 / (d * 2)) * math.sin(theta)
        
        self.repulsive_angular_z = repulsive_sum * self.k_avoid

    def goal_point_callback(self, msg: PointStamped):
        """Store latest goal point (map coordinates)."""
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.have_goal = True
        self.get_logger().info(
            f'New goal set: x={self.goal_x:.2f}, y={self.goal_y:.2f} ({msg.header.frame_id})'
        )

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.have_odom = True

    def compute_attractive_heading(self) -> float:
        """Compute attractive yaw-rate command toward the goal.

        This is the inverse intent of repulsive stumbling:
        - Repulsive logic turns away from obstacle-bearing angles.
        - Attractive logic turns toward the goal-bearing angle.
        """
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y

        distance = math.hypot(dx, dy)
        if distance < self.goal_tolerance:
            return 0.0

        goal_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(goal_heading - self.robot_yaw)

        omega = self.k_heading * heading_error
        return omega

    def timer_callback(self):
        """Publish the TwistStamped command at a consistent 20 Hz."""
        twist_stamped = TwistStamped()

        # Stamp the message with the current time and a sensible frame
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'

        if not self.have_odom or not self.have_goal:
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
            self.cmd_pub.publish(twist_stamped)
            return
        
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance:
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
            self.target_angular_z = 0.0
            self.cmd_pub.publish(twist_stamped)
            return

        # Set the forward velocity to a constant value
        twist_stamped.twist.linear.x = self.forward_speed

        # Control only the z-axis orientation velocity
        target_angular_velocity = self.repulsive_angular_z + self.compute_attractive_heading()
        target_angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, target_angular_velocity))
        twist_stamped.twist.angular.z = target_angular_velocity

        self.cmd_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()