import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

        self.publisher = self.create_publisher(Detection2DArray, '/tag_detections', 10)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.get_logger().info("AprilTag detector node started.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) if len(cv_image.shape) == 3 else cv_image
        tags = self.detector.detect(gray)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for tag in tags:
            detection = Detection2D()
            detection.header = msg.header

            # Bounding box center
            center = tag.center
            detection.bbox.center.x = float(center[0])
            detection.bbox.center.y = float(center[1])

            # Bounding box size (estimate)
            corners = tag.corners
            w = np.linalg.norm(corners[0] - corners[1])
            h = np.linalg.norm(corners[1] - corners[2])
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)

            # Tag ID as hypothesis
            result = ObjectHypothesisWithPose()
            result.id = int(tag.tag_id)
            detection.results.append(result)

            detection_array.detections.append(detection)

        if detection_array.detections:
            self.publisher.publish(detection_array)
            self.get_logger().info(f"Published {len(detection_array.detections)} detections")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    