import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

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
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                              [0, 1, 0, 0]], np.float32)
        self.kf.processNoiseCov = 1e-3 * np.eye(4, dtype=np.float32)
        self.kf.measurementNoiseCov = 5e-2 * np.eye(2, dtype=np.float32)
        self.kf.statePost = np.array([[initial_pos[0]], [initial_pos[1]], [0], [0]], dtype=np.float32)
        self.prediction = np.array([[initial_pos[0]], [initial_pos[1]]], dtype=np.float32)
        self.history = deque(maxlen=15)
        self.last_seen = cv2.getTickCount()
        self.radius = initial_radius

    def update(self, measurement, radius):
        self.kf.correct(np.array(measurement, dtype=np.float32))
        pred = self.kf.predict()
        self.prediction = pred
        self.history.append((int(pred[0]), int(pred[1])))
        self.last_seen = cv2.getTickCount()
        self.radius = radius

class MultiBallTrackerNode(Node):
    def __init__(self):
        super().__init__('multi_ball_tracker_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.ball_pub = self.create_publisher(Point, '/ball_positions', 10)
        self.bridge = CvBridge()
        self.tracks = {}
        self.next_id = 0
        self.MAX_DISTANCE = 60
        self.TRACK_TIMEOUT = 1.5 * cv2.getTickFrequency()
        self.color_calibrator = ColorCalibrator()
        self.get_logger().info("Multi Ball Tracker Node Initialized.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def process_frame(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_purple, upper_purple = self.color_calibrator.update_color_range(hsv)
        blurred_hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
        color_mask = cv2.inRange(blurred_hsv, lower_purple, upper_purple)
        mask_clean = cv2.erode(color_mask, None, iterations=1)
        mask_clean = cv2.dilate(mask_clean, None, iterations=2)
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        current_detections = []
        for contour in contours:
            if cv2.contourArea(contour) > 300:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    ((x_circle, y_circle), radius) = cv2.minEnclosingCircle(contour)
                    current_detections.append(((cx, cy), radius))
        updated_tracks = set()
        for detection, radius in current_detections:
            best_match = None
            min_dist = float('inf')
            for track_id, track in self.tracks.items():
                pred = (float(track.prediction[0]), float(track.prediction[1]))
                distance = np.linalg.norm(np.array(detection) - np.array(pred))
                if distance < self.MAX_DISTANCE and distance < min_dist:
                    min_dist = distance
                    best_match = track_id
            if best_match is not None:
                self.tracks[best_match].update(detection, radius)
                updated_tracks.add(best_match)
            else:
                self.tracks[self.next_id] = BallTrack(self.next_id, detection, radius)
                updated_tracks.add(self.next_id)
                self.next_id += 1
        current_tick = cv2.getTickCount()
        stale_tracks = [tid for tid, t in self.tracks.items() if (current_tick - t.last_seen) / cv2.getTickFrequency() > 1.5]
        for tid in stale_tracks:
            del self.tracks[tid]
        debug_frame = frame.copy()
        for track_id, track in self.tracks.items():
            x = int(track.prediction[0])
            y = int(track.prediction[1])
            cv2.circle(debug_frame, (x, y), 7, (0, 0, 255), -1)
            cv2.putText(debug_frame, f"ID:{track_id}", (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.imshow('Multi Ball Tracker Debug', debug_frame)
        cv2.waitKey(1)
        frame_height, frame_width = frame.shape[:2]
        image_center = (frame_width / 2.0, frame_height / 2.0)
        selected_track = None
        min_center_dist = float('inf')
        for track in self.tracks.values():
            pred = (float(track.prediction[0]), float(track.prediction[1]))
            dist = np.linalg.norm(np.array(pred) - np.array(image_center))
            if dist < min_center_dist:
                min_center_dist = dist
                selected_track = track
        if selected_track is not None:
            ball_center = (float(selected_track.prediction[0]), float(selected_track.prediction[1]))
            normalized_radius = float(selected_track.radius / frame_width)
            normalized_offset = float((ball_center[0] - frame_width / 2.0) / frame_width)
            point_msg = Point()
            point_msg.x = normalized_radius
            point_msg.y = normalized_offset
            point_msg.z = 0.0
            self.ball_pub.publish(point_msg)
            self.get_logger().info(f"Published ball: disparity: {point_msg.x:.2f}, offset: {point_msg.y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiBallTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
