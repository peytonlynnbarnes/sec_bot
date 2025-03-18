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

package.xml

<?xml version="1.0"?>
<package format="3">
  <name>ball_tracker</name>
  <version>0.0.0</version>
  <description>SEC Bot vision package</description>

  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>opencv-python</exec_depend>
  <exec_depend>launch</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

setup.py

from setuptools import setup
from glob import glob
import os

package_name = 'ball_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.py')) or []),  # Safe handling
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='yournamename.email@example.com',
    description='SEC Bot vision system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = ball_tracker.image_publisher:main',
            'image_subscriber = ball_tracker.image_subscriber:main',
        ],
    },
)