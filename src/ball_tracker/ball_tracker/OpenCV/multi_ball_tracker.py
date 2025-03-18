import cv2
import numpy as np
import threading
import queue
from collections import deque

# turns on OpenCV optimizations to speed things up
cv2.setUseOptimized(True)
print(f"[System] OPENCV OPTIMIZATIONS ENABLED BABY!!!! {cv2.useOptimized()}")

# starts capturing video from the webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # sets video width to 320 pixels
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # sets video height to 240 pixels
cap.set(cv2.CAP_PROP_FPS, 30)  # tries to keep the frame rate at 30 FPS

# creates a queue to store frames, but keeps it small to avoid lag
frame_queue = queue.Queue(maxsize=3)

def capture_frames():
    # runs in a separate thread to constantly grab frames from the camera
    while True:
        ret, frame = cap.read()
        if ret and frame_queue.qsize() < 2:
            frame_queue.put(cv2.UMat(frame))  # converts the frame to UMat for GPU acceleration

# starts the frame capture thread
threading.Thread(target=capture_frames, daemon=True).start()

# defies the HSV color range for detecting purple objects
lower_purple = np.array([130, 50, 50])
upper_purple = np.array([160, 255, 255])

class BallTrack:
    # BALL TRACKING VIA KALMAN FILTER!!!!
    def __init__(self, track_id, initial_pos):
        self.track_id = track_id  # assins a unique ID to this ball
        self.kf = cv2.KalmanFilter(4, 2)  # 4D state (x, y, vx, vy), 2D measurement (x, y)
        self.kf.transitionMatrix = np.array([
            [1, 0, 0.1, 0],  # x, y, vx, vy
            [0, 1, 0, 0.1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]], np.float32)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.processNoiseCov = 1e-3 * np.eye(4, dtype=np.float32)
        self.kf.measurementNoiseCov = 5e-2 * np.eye(2, dtype=np.float32)
        self.kf.statePost = np.array([[initial_pos[0]], [initial_pos[1]], [0], [0]], dtype=np.float32)
        self.prediction = initial_pos  # stores the predicted position
        self.history = deque(maxlen=15)  # keeps track of past positions to draw a trail
        self.last_seen = cv2.getTickCount()  # saves the time when this ball was last seen

    def update(self, measurement):
        # updates the balls (balls haha) position with new positions from the camera
        self.kf.correct(np.array(measurement, dtype=np.float32))  # Adjusts filter with new data
        self.prediction = self.kf.predict()  # predicts the next position
        self.history.append((int(self.prediction[0]), int(self.prediction[1])))  # Saves history for drawing a trail
        self.last_seen = cv2.getTickCount()  # updates the last seen timestamp

# dictionary to keep track of all detected balls
tracks = {}
next_id = 0  # keeps track of the next available ID for new objects
MAX_DISTANCE = 60  # maximum distance a ball can move between frames before it's considered "new"
TRACK_TIMEOUT = 1.5 * cv2.getTickFrequency()  # if a ball disappears for 1.5 seconds, remove it

while True:
    tick_start = cv2.getTickCount()  # starts FPS calculation
    
    # if we have too many frames in the queue, drop old ones to avoid lag
    while frame_queue.qsize() > 1:
        frame_queue.get()
    if frame_queue.empty():
        continue  # if no frames are available, just wait
    
    # grabs the latest frame
    frame_umat = frame_queue.get()
    
    # converts the image to HSV (easier for color filtering)
    hsv_umat = cv2.cvtColor(frame_umat, cv2.COLOR_BGR2HSV)
    blurred_hsv = cv2.GaussianBlur(hsv_umat, (5, 5), 0)  # Blurs the image to reduce noise
    color_mask = cv2.inRange(blurred_hsv, lower_purple, upper_purple)  # Filters out only purple areas
    
    # converts the image to grayscale for extra filtering
    gray_umat = cv2.cvtColor(frame_umat, cv2.COLOR_BGR2GRAY)
    blurred_gray = cv2.GaussianBlur(gray_umat, (5, 5), 0)
    _, thresh_gray = cv2.threshold(blurred_gray, 60, 255, cv2.THRESH_BINARY)
    
    # combines both masks to get a cleaner result
    combined_mask = cv2.bitwise_and(color_mask, thresh_gray)
    mask_clean = cv2.erode(combined_mask, None, iterations=1)  # removes small noise
    mask_clean = cv2.dilate(mask_clean, None, iterations=2)  # restores object shape=
    
    # converts back to CPU format for contour detection (GPU doesnt support contours!!!)
    mask_cpu = cv2.UMat.get(mask_clean)
    contours, _ = cv2.findContours(mask_cpu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # finds the center of each detected purple object
    current_detections = []
    for contour in contours:
        if cv2.contourArea(contour) > 300:  # ignores small objects
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
                current_detections.append((cx, cy))

    #  self explanatory . . .
    updated_tracks = set()
    
    for detection in current_detections:
        best_match = None
        min_dist = float('inf')  # starts with a large number
        
        # tries to match this detection with an existing tracked object
        for track_id, track in tracks.items():
            distance = np.linalg.norm(np.array(detection) - np.array(track.prediction[:2]))
            if distance < MAX_DISTANCE and distance < min_dist:
                min_dist = distance
                best_match = track_id
                
        if best_match is not None:
            tracks[best_match].update(detection)  # updates an existing tracked object
            updated_tracks.add(best_match)
        else:
            # if no match, create a new track for this object
            tracks[next_id] = BallTrack(next_id, detection)
            updated_tracks.add(next_id)
            next_id += 1

    # removes objects that haven't been seen in a while
    current_tick = cv2.getTickCount()
    stale_tracks = [tid for tid, t in tracks.items() if (current_tick - t.last_seen) > TRACK_TIMEOUT]
    for tid in stale_tracks:
        del tracks[tid]

    # converts back to a regular frame for drawing
    display_frame = cv2.UMat.get(frame_umat)
    for track_id, track in tracks.items():
        # draws the predicted position of the ball
        x, y = map(int, track.prediction[:2])
        cv2.circle(display_frame, (x, y), 7, (0, 0, 255), -1)
        
        # labels the ball with its ID
        cv2.putText(display_frame, f"ID:{track_id}", (x+10, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # displays the fps
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - tick_start)
    cv2.putText(display_frame, f"FPS: {int(fps)}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # this shows our output lol
    cv2.imshow('THE ULTIMATE BALL TRACKER', display_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()