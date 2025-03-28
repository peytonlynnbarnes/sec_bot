import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
from message_filters import ApproximateTimeSynchronizer, Subscriber

cv2.setUseOptimized(True)

class ColorCalibrator:
    def __init__(self):
        self.lower_purple = np.array([100, 30, 30])
        self.upper_purple = np.array([170, 255, 255])
        self.adaptive_lower = self.lower_purple.copy()
        self.adaptive_upper = self.upper_purple.copy()
        self.calibration_frames = 50
        self.calibration_count = 0
        self.color_samples = []

    def update_color_range(self, hsv_frame):
        mask = cv2.inRange(hsv_frame, self.lower_purple, self.upper_purple)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 100:
                contour_mask = np.zeros_like(mask)
                cv2.drawContours(contour_mask, [contour], -1, 255, -1)
                color_samples = hsv_frame[contour_mask == 255]
                self.color_samples.extend(color_samples)
        self.calibration_count += 1
        if self.calibration_count >= self.calibration_frames and self.color_samples:
            samples = np.array(self.color_samples)
            lower_percentile = np.percentile(samples, 10, axis=0)
            upper_percentile = np.percentile(samples, 90, axis=0)
            self.adaptive_lower = np.maximum(lower_percentile - 20, [0, 0, 0])
            self.adaptive_upper = np.minimum(upper_percentile + 20, [180, 255, 255])
            self.color_samples = []
            self.calibration_count = 0
            print(f"Updated color range: Lower {self.adaptive_lower}, Upper {self.adaptive_upper}")
        return self.adaptive_lower, self.adaptive_upper

class BallTrack:
    def __init__(self, track_id, initial_pos, initial_radius):
        self.track_id = track_id
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.transitionMatrix = np.array([
            [1, 0, 0.1, 0],
            [0, 1, 0, 0.1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], np.float32)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.processNoiseCov = 1e-3 * np.eye(4, dtype=np.float32)
        self.kf.measurementNoiseCov = 5e-2 * np.eye(2, dtype=np.float32)
        self.kf.statePost = np.array([[initial_pos[0]], [initial_pos[1]], [0], [0]], dtype=np.float32)
        self.prediction = initial_pos
        self.history = deque(maxlen=15)
        self.last_seen = cv2.getTickCount()
        self.radius = initial_radius

    def update(self, measurement, radius):
        self.kf.correct(np.array(measurement, dtype=np.float32))
        self.prediction = self.kf.predict()
        self.history.append((int(self.prediction[0][0]), int(self.prediction[1][0])))
        self.last_seen = cv2.getTickCount()
        self.radius = radius

class MultiBallTrackerNode(Node):
    def __init__(self):
        super().__init__('multi_ball_tracker_node')
        
        # declare parameters for camera calibration (from stereo.yaml)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera.fx', 320.0),
                ('camera.fy', 320.0),
                ('camera.cx', 320.0),
                ('camera.cy', 240.0),
                ('stereo.b', 0.12),  # baseline in meters
            ]
        )

        # subscribers for left and right images using message_filters
        self.left_sub = Subscriber(self, Image, '/camera_left/image_raw')
        self.right_sub = Subscriber(self, Image, '/camera_right/image_raw')
        self.ts = ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.stereo_image_callback)

        # publisher for ball position as PointStamped
        self.ball_pub = self.create_publisher(PointStamped, '/ball_positions', 10)
        self.bridge = CvBridge()
        self.tracks = {}
        self.next_id = 0
        self.MAX_DISTANCE = 60
        self.TRACK_TIMEOUT = 1.5 * cv2.getTickCount()
        self.color_calibrator = ColorCalibrator()
        self.get_logger().info("Multi Ball Tracker Node Initialized with Stereo Vision.")

    def find_ball(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_purple, upper_purple = self.color_calibrator.update_color_range(hsv)
        blurred_hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
        color_mask = cv2.inRange(blurred_hsv, lower_purple, upper_purple)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh_gray = cv2.threshold(blurred_gray, 60, 255, cv2.THRESH_BINARY)
        combined_mask = cv2.bitwise_and(color_mask, thresh_gray)
        mask_clean = cv2.erode(combined_mask, None, iterations=1)
        mask_clean = cv2.dilate(mask_clean, None, iterations=2)
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_contour = None
        max_area = 300  # Minimum area threshold
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                largest_contour = contour

        if largest_contour is not None:
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                ((_, _), radius) = cv2.minEnclosingCircle(largest_contour)
                return (cx, cy), radius
        return None, None

    def stereo_image_callback(self, left_msg, right_msg):
        try:
            left_frame = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            right_frame = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting stereo images: {e}")
            return

        # detect ball in both images
        left_result, left_radius = self.find_ball(left_frame)
        right_result, right_radius = self.find_ball(right_frame)

        if left_result and right_radius:
            u_left, v_left = left_result
            u_right, v_right = right_result

            # match balls based on vertical proximity (assuming horizontal baseline)
            if abs(v_left - v_right) < 10:  # Tolerance for vertical alignment
                disparity = u_left - u_right
                if disparity > 0:  # Ensure positive disparity (left image has higher u)
                    fx = self.get_parameter('camera.fx').value
                    baseline = self.get_parameter('stereo.b').value
                    depth = (fx * baseline) / disparity
                    cx = self.get_parameter('camera.cx').value
                    cy = self.get_parameter('camera.cy').value
                    fy = self.get_parameter('camera.fy').value

                    # calculate 3D position in left camera frame
                    X = (u_left - cx) * depth / fx
                    Y = (v_left - cy) * depth / fy
                    Z = depth

                    # publish as PointStamped
                    point_msg = PointStamped()
                    point_msg.header = left_msg.header  # Use left image timestamp and frame_id
                    point_msg.header.frame_id = "left_camera_link"  # Adjust frame_id as needed
                    point_msg.point.x = X
                    point_msg.point.y = Y
                    point_msg.point.z = Z
                    self.ball_pub.publish(point_msg)
                    self.get_logger().info(f"Published ball position: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}")

                    # update tracking with left image position
                    if not self.tracks:
                        self.tracks[self.next_id] = BallTrack(self.next_id, (u_left, v_left), left_radius)
                        self.next_id += 1
                    else:
                        best_match = None
                        min_dist = float('inf')
                        for track_id, track in self.tracks.items():
                            pred = (track.prediction[0][0], track.prediction[1][0])
                            distance = np.linalg.norm(np.array((u_left, v_left)) - np.array(pred))
                            if distance < self.MAX_DISTANCE and distance < min_dist:
                                min_dist = distance
                                best_match = track_id
                        if best_match is not None:
                            self.tracks[best_match].update((u_left, v_left), left_radius)
                        else:
                            self.tracks[self.next_id] = BallTrack(self.next_id, (u_left, v_left), left_radius)
                            self.next_id += 1

        # clean up stale tracks
        current_tick = cv2.getTickCount()
        stale_tracks = [tid for tid, t in self.tracks.items() if (current_tick - t.last_seen) > self.TRACK_TIMEOUT]
        for tid in stale_tracks:
            del self.tracks[tid]

        # debugging visualization on left image
        debug_frame = left_frame.copy()
        for track_id, track in self.tracks.items():
            x = int(track.prediction[0][0])
            y = int(track.prediction[1][0])
            cv2.circle(debug_frame, (x, y), 7, (0, 0, 255), -1)
            cv2.putText(debug_frame, f"ID:{track_id}", (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.imshow('Multi Ball Tracker Debug', debug_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MultiBallTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()