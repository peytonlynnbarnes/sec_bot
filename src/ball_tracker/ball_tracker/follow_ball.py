import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time

class FollowBall(Node):
    def __init__(self):
        super().__init__('follow_ball')

        self.subscription = self.create_subscription(
            Point,
            '/ball_positions',
            self.listener_callback,
            10
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.declare_parameter("angular_chase_multiplier", 1.0)
        self.declare_parameter("max_linear_speed", 0.3)
        self.declare_parameter("min_linear_speed", 0.1)
        self.declare_parameter("stop_size_thresh", 0.4)
        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("x_smoothing", 0.9)
        self.declare_parameter("z_smoothing", 0.9)

        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.stop_size_thresh = self.get_parameter('stop_size_thresh').value
        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').value
        self.x_smoothing = self.get_parameter('x_smoothing').value
        self.z_smoothing = self.get_parameter('z_smoothing').value

        # Ball tracking
        self.target_x = 0.0
        self.target_size = 0.0
        self.lastrcvtime = time.time() - 1000

        self.timer = self.create_timer(0.1, self.control_loop)

    def listener_callback(self, msg):
        # Exponential smoothing of input
        self.target_x = self.x_smoothing * self.target_x + (1 - self.x_smoothing) * msg.x
        self.target_size = self.z_smoothing * self.target_size + (1 - self.z_smoothing) * msg.z
        self.lastrcvtime = time.time()
        self.get_logger().info(f"Ball position received: x={msg.x:.3f}, z={msg.z:.3f}")

    def control_loop(self):
        msg = Twist()
        time_since_last = time.time() - self.lastrcvtime

        if time_since_last > self.rcv_timeout_secs:
            msg.angular.z = 0.5
            msg.linear.x = 0.0
            self.get_logger().info("Ball lost — rotating to search.")
        else:
            # Smooth rotation toward the ball
            angular = -self.angular_chase_multiplier * self.target_x
            msg.angular.z = max(min(angular, 0.5), -0.5)

            # Scale forward speed based on how far the ball is
            if self.target_size < self.stop_size_thresh:
                speed = self.max_linear_speed * (1.0 - self.target_size)
                msg.linear.x = max(speed, self.min_linear_speed)
                self.get_logger().info(
                    f"Chasing: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}"
                )
            else:
                msg.linear.x = 0.0
                self.get_logger().info("Ball is close. Stopping.")

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FollowBall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
