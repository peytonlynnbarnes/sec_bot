import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Get parameter with proper type handling
        self.declare_parameter('image_topic', '/camera/image_raw')
        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info(f"Subscribed to: {topic}")

    def process_image(self, cv_image):
        """Example processing: Convert to grayscale"""
        return cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            processed = self.process_image(cv_image)
            cv2.imshow("Processed Image", processed)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.waitKey(1)  # Ensure OpenCV windows close properly
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
