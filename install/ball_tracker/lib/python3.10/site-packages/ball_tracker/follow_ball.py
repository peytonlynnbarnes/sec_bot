import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time

class FollowBall(Node):
    def __init__(self):
        super().__init__('follow_ball')
        
        self.subscription = self.create_subscription(
            Point,
            '/ball_positions',  # Listening to detected ball position
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 1.2)
        self.declare_parameter("base_forward_speed", 0.15)  # Base speed
        self.declare_parameter("max_forward_speed", 0.3)  # Maximum speed
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("max_size_thresh", 0.4)  # Adjusted stopping condition
        self.declare_parameter("stop_size_thresh", 0.5)  # Stop when ball is large enough
        self.declare_parameter("filter_value", 0.9)
        self.declare_parameter("angular_timeout", 2.0)
        self.declare_parameter("facing_threshold", 0.03)
        
        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').value
        self.base_forward_speed = self.get_parameter('base_forward_speed').value
        self.max_forward_speed = self.get_parameter('max_forward_speed').value
        self.search_angular_speed = self.get_parameter('search_angular_speed').value
        self.max_size_thresh = self.get_parameter('max_size_thresh').value
        self.stop_size_thresh = self.get_parameter('stop_size_thresh').value
        self.filter_value = self.get_parameter('filter_value').value
        self.angular_timeout = self.get_parameter('angular_timeout').value
        self.facing_threshold = self.get_parameter('facing_threshold').value
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.target_x = 0.0
        self.target_size = 0.0
        self.lastrcvtime = time.time() - 10000
        self.last_turn_time = None  # Track when turning started

    def timer_callback(self):
        msg = Twist()
        time_since_last_ball = time.time() - self.lastrcvtime
        
        if time_since_last_ball < self.rcv_timeout_secs:
            if self.target_size < self.stop_size_thresh:
                # Adjust speed dynamically: closer ball = slower approach
                msg.linear.x = max(self.base_forward_speed, self.max_forward_speed * (1 - self.target_size))
            else:
                msg.linear.x = 0.0  # Stop if ball is very close
                self.get_logger().info("Ball is very close, stopping movement.")
            
            # Improved turning logic with timeout
            if abs(self.target_x) > self.facing_threshold:
                if self.last_turn_time is None:
                    self.last_turn_time = time.time()  # Start turn timer
                elif time.time() - self.last_turn_time > self.angular_timeout:
                    msg.angular.z = 0.0  # Stop turning if timeout reached
                else:
                    msg.angular.z = -self.angular_chase_multiplier * self.target_x
                    msg.angular.z = max(min(msg.angular.z, 1.0), -1.0)  # Clamp rotation speed
            else:
                msg.angular.z = 0.0  # Stop turning when facing the ball
                self.last_turn_time = None  # Reset turn timer
                self.get_logger().info("Robot is facing the ball correctly.")
        else:
            msg.angular.z = self.search_angular_speed  # Search mode when ball is lost
            self.last_turn_time = None  # Reset turn timer when searching
        
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Published velocities: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def listener_callback(self, msg):
        f = self.filter_value
        self.target_x = self.target_x * f + msg.x * (1 - f)
        self.target_size = self.target_size * f + msg.z * (1 - f)  # Assuming z is size
        self.lastrcvtime = time.time()
        
        self.get_logger().info(f'Received ball position: x={msg.x}, z={msg.z}')


def main(args=None):
    rclpy.init(args=args)
    follow_ball = FollowBall()
    rclpy.spin(follow_ball)
    follow_ball.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()