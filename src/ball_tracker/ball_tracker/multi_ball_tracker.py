import cv2  # opencv for image processing
import numpy as np  # numpy for numerical operations
import threading  # threading for parallel execution
import queue  # queue to store frames efficiently
import rclpy  # ROS 2 python api for communication
from rclpy.node import Node  # ROS 2 node class (every ROS program needs a node)
from geometry_msgs.msg import Point  # ROS 2 message type for publishing ball positions

# enable opencv optimizations to improve performance
cv2.setUseOptimized(True)  # optimized code execution (better cpu usage)
print(f"[System] OPENCV OPTIMIZATIONS ENABLED!!!!!!!! {cv2.useOptimized()}")  # print optimization status

# open video capture from the default camera (index 0)
cap = cv2.VideoCapture(0)

# set the resolution of the video stream to 320x240 (lower resolution = faster processing)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 30)  # set fps to 30 (matches standard webcam refresh rate)

# create a queue to store video frames with a maximum size of 3
frame_queue = queue.Queue(maxsize=3)

# threaded function to continuously capture frames from the camera
def capture_frames():
    while True:  # infinite loop (runs as long as the program runs)
        ret, frame = cap.read()  # read a frame from the camera
        if ret and frame_queue.qsize() < 2:  # only store the frame if the queue is not full
            frame_queue.put(cv2.UMat(frame))  # store the frame in opencv's umat (optimized memory management)

# start the capture_frames function as a separate thread (daemon=True means it stops when the main program stops)
threading.Thread(target=capture_frames, daemon=True).start()

# define the color range for detecting purple balls (hsv format)
lower_purple = np.array([130, 50, 50])  # lower bound of purple
upper_purple = np.array([160, 255, 255])  # upper bound of purple

# class to track individual balls using a kalman filter
class BallTrack:
    def __init__(self, track_id, initial_pos):
        self.track_id = track_id  # unique id for each tracked ball

        # initialize kalman filter (4d state: x, y, dx, dy and 2d measurement: x, y)
        self.kf = cv2.KalmanFilter(4, 2)

        # define the kalman filter transition matrix (predicts the next position based on velocity)
        self.kf.transitionMatrix = np.array([
            [1, 0, 0.1, 0], 
            [0, 1, 0, 0.1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]], np.float32)

        # measurement matrix (describes how we observe the object)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)

        # process noise covariance (tells the kalman filter how much trust to place in the model vs the measurement)
        self.kf.processNoiseCov = 1e-3 * np.eye(4, dtype=np.float32)

        # measurement noise covariance (accounts for noise in sensor readings)
        self.kf.measurementNoiseCov = 5e-2 * np.eye(2, dtype=np.float32)

        # set the initial state estimate (ball's starting position)
        self.kf.statePost = np.array([[initial_pos[0]], [initial_pos[1]], [0], [0]], dtype=np.float32)

        self.prediction = initial_pos  # store last predicted position
        self.last_seen = cv2.getTickCount()  # time when the ball was last detected

    def update(self, measurement):
        self.kf.correct(np.array(measurement, dtype=np.float32))  # correct state based on new measurement
        self.prediction = self.kf.predict()  # predict next state
        self.last_seen = cv2.getTickCount()  # update last seen timestamp

# ROS 2 node for tracking balls and publishing their positions
class BallTrackerNode(Node):
    def __init__(self):
        super().__init__('ball_tracker_node')  # initialize ROS 2 node

        self.ball_pub = self.create_publisher(Point, '/ball_positions', 10)  # ROS 2 publisher to send ball positions

        self.tracks = {}  # dictionary to store all tracked balls
        self.next_id = 0  # id counter for assigning unique ids to new balls
        self.MAX_DISTANCE = 60  # maximum distance allowed to associate new detections with existing tracks
        self.TRACK_TIMEOUT = 1.5 * cv2.get_tick_frequency()  # timeout for removing stale tracks

        self.timer = self.create_timer(0.03, self.track_balls)  # ROS 2 timer to execute `track_balls()` every 30ms

    def track_balls(self):
        tick_start = cv2.getTickCount()  # get the starting time of this iteration

        # process only the most recent frame (drop older frames)
        while frame_queue.qsize() > 1:
            frame_queue.get()
        if frame_queue.empty():
            return  # skip processing if no frame is available

        frame_umat = frame_queue.get()  # get the most recent frame
        hsv_umat = cv2.cvtColor(frame_umat, cv2.COLOR_BGR2HSV)  # convert to hsv
        blurred_hsv = cv2.GaussianBlur(hsv_umat, (5, 5), 0)  # apply gaussian blur
        color_mask = cv2.inRange(blurred_hsv, lower_purple, upper_purple)  # apply color thresholding

        gray_umat = cv2.cvtColor(frame_umat, cv2.COLOR_BGR2GRAY)  # convert to grayscale
        blurred_gray = cv2.GaussianBlur(gray_umat, (5, 5), 0)  # apply gaussian blur
        _, thresh_gray = cv2.threshold(blurred_gray, 60, 255, cv2.THRESH_BINARY)  # thresholding

        combined_mask = cv2.bitwise_and(color_mask, thresh_gray)  # combine color + threshold mask
        mask_clean = cv2.erode(combined_mask, None, iterations=1)  # remove noise (erode)
        mask_clean = cv2.dilate(mask_clean, None, iterations=2)  # expand valid regions (dilate)

        # find contours of detected balls
        mask_cpu = cv2.UMat.get(mask_clean)
        contours, _ = cv2.findContours(mask_cpu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        current_detections = []  # store detected ball positions
        for contour in contours:
            if cv2.contourArea(contour) > 300:  # ignore small noise
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"]/M["m00"])  # calculate centroid x
                    cy = int(M["m01"]/M["m00"])  # calculate centroid y
                    current_detections.append((cx, cy))  # store detection

        # publish ball positions to ROS 2
        for detection in current_detections:
            x, y = detection
            ball_msg = Point()
            ball_msg.x = x
            ball_msg.y = y
            ball_msg.z = 0
            self.ball_pub.publish(ball_msg)
            self.get_logger().info(f"Published Ball: ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)
    node = BallTrackerNode()
    rclpy.spin(node)  # keep node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
