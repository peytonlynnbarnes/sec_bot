import cv2
import numpy as np
import queue
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

# Queues to store frames from multiple cameras
frame_queue_1 = queue.Queue(maxsize=3)
frame_queue_2 = queue.Queue(maxsize=3)

class MultiCameraBallTracker(Node):
    def __init__(self):
        super().__init__('multi_camera_ball_tracker')

        # Publisher for detected ball position
        self.ball_pub = self.create_publisher(Point, '/ball_positions', 10)

        # Subscribing to two camera feeds
        self.create_subscription(Image, '/camera1/image_raw', self.image_callback_1, 10)
        self.create_subscription(Image, '/camera2/image_raw', self.image_callback_2, 10)

        self.current_target = None  # Track the currently followed ball
        self.camera_separation = 0.3  # Distance between cameras in meters

    def image_callback_1(self, msg):
        """ Callback for Camera 1 """
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            if frame_queue_1.qsize() < 2:
                frame_queue_1.put(frame)
        except Exception as e:
            self.get_logger().error(f"Camera 1 Error: {e}")

    def image_callback_2(self, msg):
        """ Callback for Camera 2 """
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            if frame_queue_2.qsize() < 2:
                frame_queue_2.put(frame)
        except Exception as e:
            self.get_logger().error(f"Camera 2 Error: {e}")

    def process_frame(self, frame):
        """ Detect balls in a frame and return position + size """
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blurred_hsv = cv2.GaussianBlur(hsv_frame, (5, 5), 0)

        # Define HSV range for purple ball
        lower_bound = np.array([110, 50, 50])  # Lower HSV for purple
        upper_bound = np.array([170, 255, 255])  # Upper HSV for purple
        mask = cv2.inRange(blurred_hsv, lower_bound, upper_bound)

        # Find contours (detected balls)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_balls = []

        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if radius > 5:  # Ignore small noise
                detected_balls.append((x, y, radius))

        if detected_balls:
            # Choose the largest detected ball (closest one)
            detected_balls.sort(key=lambda b: -b[2])
            return detected_balls[0]  # (x, y, radius)
        return None

    def track_balls(self):
        """ Process both camera feeds and merge detections with positional correction """
        balls = []

        # Process Camera 1
        if not frame_queue_1.empty():
            ball_1 = self.process_frame(frame_queue_1.get())
            if ball_1:
                balls.append((ball_1[0], ball_1[2], -self.camera_separation / 2))

        # Process Camera 2
        if not frame_queue_2.empty():
            ball_2 = self.process_frame(frame_queue_2.get())
            if ball_2:
                balls.append((ball_2[0], ball_2[2], self.camera_separation / 2))

        if balls:
            avg_x = sum(b[0] + b[2] for b in balls) / len(balls)  # Adjust for camera separation
            avg_size = sum(b[1] for b in balls) / len(balls)

            msg = Point()
            msg.x = (avg_x - 320) / 640  # Normalize X position
            msg.z = avg_size / 640  # Approximate distance with size
            self.ball_pub.publish(msg)

            self.get_logger().info(f"[DEBUG] Merged Ball Detection: x={msg.x}, z={msg.z}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraBallTracker()

    while rclpy.ok():
        rclpy.spin_once(node)
        node.track_balls()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
