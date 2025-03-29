import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, PoseArray
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np
import serial

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('april_tag_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(Image, topic, self.image_callback, 10)
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

        # Publishers
        self.box_publisher = self.create_publisher(PoseArray, '/box_positions', 10)
        self.robot_publisher = self.create_publisher(Point, '/robot_position', 10)

        # Serial connection
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Serial connection established on /dev/ttyUSB0")
        except serial.SerialException as e:
            self.serial_port = None
            self.get_logger().error(f"Failed to open serial port: {e}")

        self.get_logger().info(f"Subscribed to {topic} for AprilTag detection")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            tags = self.detector.detect(gray)

            h, w = gray.shape
            box_poses = PoseArray()
            box_poses.header = msg.header

            robot_position = None

            for tag in tags:
                tag_id = tag.tag_id
                center = tag.center  # (x, y)

                # Normalize to [-1, 1]
                norm_x = (center[0] - w / 2) / (w / 2)
                norm_y = (center[1] - h / 2) / (h / 2)

                if tag_id == 0:
                    # Robot tag
                    robot_position = Point(x=norm_x, y=norm_y, z=0.0)
                else:
                    # Box tag
                    pose = Pose()
                    pose.position.x = norm_x
                    pose.position.y = norm_y
                    pose.position.z = 0.0
                    box_poses.poses.append(pose)

            # Publish and send robot position
            if robot_position:
                self.robot_publisher.publish(robot_position)
                self.get_logger().info(f"Published robot tag: x={robot_position.x:.2f}, y={robot_position.y:.2f}")
                self.send_serial(f"ROBOT,{robot_position.x:.2f},{robot_position.y:.2f}")

            # Publish and send box tag positions
            if box_poses.poses:
                self.box_publisher.publish(box_poses)
                self.get_logger().info(f"Published {len(box_poses.poses)} box tag(s)")
                for i, pose in enumerate(box_poses.poses):
                    self.send_serial(f"BOX{i},{pose.position.x:.2f},{pose.position.y:.2f}")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def send_serial(self, data_str):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write((data_str + '\n').encode())
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")
        else:
            self.get_logger().warn("Serial port not available.")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
