import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point, Twist
from cv_bridge              import CvBridge, CvBridgeError
import ball_tracker.process_image as proc
from geometry_msgs.msg import Twist
import rclpy
import subprocess

class DetectBall(Node):

    def __init__(self):
        super().__init__('detect_ball')

        self.get_logger().info('Looking for the ball...')
        self.image_sub = self.create_subscription(Image,"/camera/image_raw",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
        self.ball_pub  = self.create_publisher(Point,"/detected_ball",1)
        self.cmd_sub = self.create_subscription(Twist, "/diff_drive_controller/cmd_vel_unstamped", self.minimal_callback1, 1)
        self.timer = self.create_timer(1, self.end_timer_callback)
        self.vel_counter = 0
        self.current_time = 0

        self.declare_parameter('tuning_mode', False)

        self.declare_parameter("x_min",0)
        self.declare_parameter("x_max",100)
        self.declare_parameter("y_min",32)
        self.declare_parameter("y_max",100)
        self.declare_parameter("h_min",25)
        self.declare_parameter("h_max",37)
        self.declare_parameter("s_min",42)
        self.declare_parameter("s_max",255)
        self.declare_parameter("v_min",0)
        self.declare_parameter("v_max",255)
        self.declare_parameter("sz_min",0)
        self.declare_parameter("sz_max",20)
        
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max').get_parameter_value().integer_value
        }

        self.bridge = CvBridge()

        if(self.tuning_mode):
            proc.create_tuning_window(self.tuning_params)

      # Additional functionality


        # Create subscriber for the '/diff_drive_controller/cmd_vel_teleop' topic
        

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            if (self.tuning_mode):
                self.tuning_params = proc.get_tuning_params()

            keypoints_norm, out_image, tuning_image = proc.find_circles(cv_image, self.tuning_params)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            self.image_tuning_pub.publish(img_to_pub)

            point_out = Point()

            # Keep the biggest point
            # They are already converted to normalised coordinates
            for i, kp in enumerate(keypoints_norm):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size
                
                # UNCOMMENT THIS FOR DEBUGGING
#                self.get_logger().info(f"Pt {i}: ({x},{y},{s})")

                if (s > point_out.z):                    
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if (point_out.z > 0):
                self.ball_pub.publish(point_out) 
        except CvBridgeError as e:
            print(e)  

#This adds to vel_counter if the robot is moving especially slowly toward the ball.
    def minimal_callback1(self, msg):
        self.vel_counter += 1 if (0 <= msg.angular.z <= 0.05) and (0 <= msg.linear.x <= 0.1) else 0


#This prevents this node from infinitely approaching the ball by checking two conditions every one second. 
#It's seeing if vel_counter(referenced above) is high enough to say that the robot has reached the ball.
    def end_timer_callback(self):
        self.current_time+=1

        #Increase this if you want the robot to get closer
        if self.vel_counter >= 30:
            self.get_logger().info("The robot seems to be infront of it's target. Ending detect_ball..")
            subprocess.call("pkill -2 -f ball_tracker_launch.py", shell=True)
            self.destroy_node()
            rclpy.shutdown()

        #Increase this if the robot will take lots of time to approach the ball
        elif self.current_time > 25:
            self.get_logger().info("Too much time has passed without success, breaking")
            subprocess.call("pkill -2 -f ball_tracker_launch.py", shell=True)
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):

    rclpy.init(args=args)

    detect_ball = DetectBall()
    while rclpy.ok():
        rclpy.spin_once(detect_ball)
        proc.wait_on_gui()

    detect_ball.destroy_node()
    rclpy.shutdown()