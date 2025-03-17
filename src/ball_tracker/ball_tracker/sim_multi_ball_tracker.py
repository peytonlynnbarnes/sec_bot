import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import queue

# Create a CvBridge object for converting ROS images to OpenCV images
bridge = CvBridge()
frame_queue = queue.Queue(maxsize=3)
cv2.setUseOptimized(True)

class BallTrackerNode(Node):
    def __init__(self):
        super().__init__('ball_tracker_node')
        self.ball_pub = self.create_publisher(Point, '/ball_positions', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        print("[INFO] Ball Tracker Node Initialized.")

    def image_callback(self, msg):
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            if frame_queue.qsize() < 2:
                frame_queue.put(frame)  # Store as NumPy array instead of UMat
            print("[DEBUG] Received an image frame!")
        except Exception as e:
            print(f"[ERROR] Error converting image: {e}")

    def track_balls(self):
        if frame_queue.empty():
            print("[DEBUG] Frame queue is empty, skipping detection.")
            return
        
        frame = frame_queue.get()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blurred_hsv = cv2.GaussianBlur(hsv_frame, (5, 5), 0)
        print("[DEBUG] Processing frame for ball detection.")

        # Define HSV range for the purple ball
        lower_bound = np.array([130, 50, 50])
        upper_bound = np.array([160, 255, 255])
        mask = cv2.inRange(blurred_hsv, lower_bound, upper_bound)

        cv2.imshow("Debug Mask", mask)
        cv2.waitKey(1)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)

            if radius > 5:  # Ignore small noise
                msg = Point()
                msg.x = (x - frame.shape[1] / 2) / frame.shape[1]  # Normalize X position
                msg.z = radius / frame.shape[1]  # Use size as distance approximation
                self.ball_pub.publish(msg)
                print(f"[DEBUG] Published ball position: x={msg.x}, z={msg.z}")
                
                # Draw detected ball on the frame
                cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)  # Green contour
                cv2.circle(frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)  # Blue circle
            else:
                print("[DEBUG] Detected ball is too small, ignoring.")
        else:
            print("[DEBUG] No ball detected in frame.")

        cv2.imshow("Contour Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BallTrackerNode()

    while rclpy.ok():
        rclpy.spin_once(node)
        node.track_balls()  # Ensure this function exists

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
