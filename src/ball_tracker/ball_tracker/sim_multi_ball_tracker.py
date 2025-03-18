import cv2  # OpenCV for image processing
import numpy as np  # Numerical operations
import queue  # Efficient frame storage
import rclpy  # ROS 2 API for communication
from rclpy.node import Node  # ROS 2 Node class
from geometry_msgs.msg import Point  # Message type for ball positions
from sensor_msgs.msg import Image  # ROS 2 Image message
from cv_bridge import CvBridge  # Bridge between ROS and OpenCV

bridge = CvBridge()
frame_queue = queue.Queue(maxsize=3)
cv2.setUseOptimized(True)


class BallTrackerNode(Node):
    def __init__(self):
        super().__init__("ball_tracker_node")
        self.ball_pub = self.create_publisher(Point, "/ball_positions", 10)
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.current_target = None  # Track which ball we're following

    def image_callback(self, msg):
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            if frame_queue.qsize() < 2:
                frame_queue.put(frame)
        except Exception as e:
            print(f"[ERROR] Error converting image: {e}")

    def track_balls(self):
        if frame_queue.empty():
            return

        frame = frame_queue.get()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blurred_hsv = cv2.GaussianBlur(hsv_frame, (5, 5), 0)

        lower_bound = np.array([110, 50, 50])
        upper_bound = np.array([170, 255, 255])
        mask = cv2.inRange(blurred_hsv, lower_bound, upper_bound)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_balls = []

        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if radius > 5:  # Ignore small noise
                detected_balls.append((x, y, radius))
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

        if detected_balls:
            # Sort balls by distance (largest radius = closest)
            detected_balls.sort(key=lambda b: -b[2])

            if self.current_target:
                # Stick to the closest ball unless it's lost
                closest_ball = min(
                    detected_balls, key=lambda b: abs(b[0] - self.current_target[0])
                )
            else:
                closest_ball = detected_balls[0]

            self.current_target = closest_ball  # Update target
            msg = Point()
            msg.x = (closest_ball[0] - frame.shape[1] / 2) / frame.shape[
                1
            ]  # Normalize X position
            msg.z = (
                closest_ball[2] / frame.shape[1]
            )  # Use size as distance approximation
            self.ball_pub.publish(msg)
            print(f"[DEBUG] Tracking ball at x={msg.x}, z={msg.z}")
        else:
            self.current_target = None  # Reset target if no balls detected

        cv2.imshow("Ball Tracking", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BallTrackerNode()

    while rclpy.ok():
        rclpy.spin_once(node)
        node.track_balls()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
