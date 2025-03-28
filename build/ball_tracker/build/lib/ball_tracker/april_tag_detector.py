import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('april_tag_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(Point, '/box_position', 10)
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

        self.get_logger().info(f"Subscribed to {topic} for AprilTag detection")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            tags = self.detector.detect(gray)

            for tag in tags:
                center = tag.center  # (x, y) pixel coords

                # Normalize x and y to [-1, 1]
                h, w = gray.shape
                norm_x = (center[0] - w / 2) / (w / 2)
                norm_y = (center[1] - h / 2) / (h / 2)

                point = Point()
                point.x = norm_x
                point.y = norm_y
                point.z = 0.0  # Could be used for estimated distance if needed

                self.publisher.publish(point)
                self.get_logger().info(f"Published tag position: x={point.x:.2f}, y={point.y:.2f}")
                break  # Only publish the first detected tag
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
