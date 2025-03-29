import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import serial

class AprilTagReporter(Node):
    def __init__(self):
        super().__init__('april_tag_serial_reporter')

        # Proper indentation here
        self.serial_port = serial.Serial('/dev/serial/by-id/usb-Teensyduino_USB_Serial_16312630-if00', 115200, timeout=1)

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/tag_detections',
            self.detection_callback,
            10)

        self.get_logger().info("AprilTag reporter (serial) node started.")
        
    def detection_callback(self, msg):
        detected_tags = []

        for detection in msg.detections:
            tag_id = int(detection.results[0].id)
            bbox = detection.bbox
            avg_size = (bbox.size_x + bbox.size_y) / 2.0
            distance = 1.0 / avg_size if avg_size > 0 else 100.0
            detected_tags.append((tag_id, distance))

        if not detected_tags:
            return

        closest_tag_id, closest_distance = min(detected_tags, key=lambda x: x[1])

        left_tag_id = -1
        for tag_id, _ in detected_tags:
            if 0 <= tag_id <= 4:
                left_tag_id = tag_id
                break

        report = f"<{left_tag_id}, {closest_tag_id}, {closest_distance:.2f}>\n"
        self.get_logger().info(f"Serial: {report.strip()}")

        try:
            self.serial_port.write(report.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagReporter()
    rclpy.spin(node)
    node.serial_port.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    