import rclpy
import time
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from threading import Lock
from lifecycle_msgs.msg import Transition
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange



class ImagePublisher(LifecycleNode):
    def __init__(self):
        super().__init__("image_publisher")
        self.bridge = CvBridge()
        self.cap = None
        self.static_image = None
        self.failed_frames = 0
        self.max_retries = 5
        self.reconnect_delay = 1.0
        self.cam_lock = Lock()
        self.cb_group = ReentrantCallbackGroup()

        # lifecycle state tracking
        self.activated = False
        self.configured = False

        # parameter declarations with validation ranges
        publish_range = FloatingPointRange()
        publish_range.from_value = 1.0
        publish_range.to_value = 60.0
        publish_range.step = 1.0
        
        self.declare_parameter(
            "publish_rate",
            30.0,
            ParameterDescriptor(
                type=Parameter.Type.DOUBLE,
                floating_point_range=[publish_range],
            ),
        )
        
        self.declare_parameter(
            "image_path", "", ParameterDescriptor(type=Parameter.Type.STRING)
        )
        
        camera_range = IntegerRange()
        camera_range.from_value = 0
        camera_range.to_value = 16
        camera_range.step = 1
        
        self.declare_parameter(
            "camera_index",
            0,
            ParameterDescriptor(
                type=Parameter.Type.INTEGER,
                integer_range=[camera_range],
            ),
        )
        
        width_range = IntegerRange()
        width_range.from_value = -1
        width_range.to_value = 7680
        width_range.step = 1
        
        self.declare_parameter(
            "resize_width",
            -1,
            ParameterDescriptor(
                type=Parameter.Type.INTEGER,
                integer_range=[width_range],
            ),
        )

        # lifecycle services
        self.restart_service = self.create_service(
            Trigger, "~/restart", self.restart_callback, callback_group=self.cb_group
        )
        
    def process_frame(self, frame):
        """Process frame with optional resizing"""
        if self.resize_width > 0:
            h, w = frame.shape[:2]
            new_h = int(h * (self.resize_width / w))
            return cv2.resize(frame, (self.resize_width, new_h))
        return frame

    def restart_callback(self, request, response):
        """Enhanced restart handler with state safety"""
        try:
            self.get_logger().info("Initiating controlled restart...")

            # deactivate if active
            if self.activated:
                if not self.on_deactivate(State.PRIMARY_STATE_ACTIVE).successful:
                    raise RuntimeError("Deactivation failed during restart")

            # cleanup existing configuration
            if self.configured:
                self.on_cleanup(State.PRIMARY_STATE_INACTIVE)

            # attempt full reconfiguration
            config_result = self.on_configure(State.PRIMARY_STATE_UNCONFIGURED)
            if not config_result.successful:
                raise RuntimeError("Reconfiguration failed")

            # reactivate if needed
            activate_result = self.on_activate(State.PRIMARY_STATE_INACTIVE)
            if not activate_result.successful:
                self.on_cleanup(State.PRIMARY_STATE_INACTIVE)
                raise RuntimeError("Reactivation failed")

            response.success = True
            response.message = "Restart completed successfully"

        except Exception as e:
            self.get_logger().error(f"Controlled restart failed: {str(e)}")
            response.success = False
            response.message = f"Restart failed: {str(e)}"
            self.configured = False
            self.activated = False

        return response

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring resources...")
        try:
            with self.cam_lock:
                self.publish_rate = self.get_parameter("publish_rate").value
                self.image_path = self.get_parameter("image_path").value
                self.camera_index = self.get_parameter("camera_index").value
                self.resize_width = self.get_parameter("resize_width").value

                if self.resize_width < -1:
                    raise ValueError("Invalid resize_width (must be ≥ -1)")

                if self.image_path:
                    if not os.path.exists(self.image_path):
                        raise FileNotFoundError("Image file missing")
                    self.static_image = cv2.imread(self.image_path)
                    if self.static_image is None:
                        raise ValueError("Invalid image format")

                self.configured = True
                return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Configuration error: {str(e)}")
            self.configured = False
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating publisher...")
        try:
            with self.cam_lock:
                if not self.configured:
                    raise RuntimeError("Activation without configuration")

                if not self.image_path:
                    self.cap = cv2.VideoCapture(self.camera_index)
                    if not self.cap.isOpened():
                        raise RuntimeError(f"Camera {self.camera_index} unavailable")

                self.publisher = self.create_publisher(Image, "/camera/image_raw", 10)
                self.timer = self.create_timer(
                    1.0 / self.publish_rate,
                    self.timer_callback,
                    callback_group=self.cb_group,
                )
                self.activated = True
                self.failed_frames = 0
                return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Activation failure: {str(e)}")
            self.on_cleanup(state)
            return TransitionCallbackReturn.FAILURE

    def timer_callback(self):
        if not self.activated:
            return

        try:
            with self.cam_lock:
                if self.static_image is not None:
                    frame = self.process_frame(self.static_image)
                else:
                    ret, frame = self.cap.read()
                    if not ret:
                        self.failed_frames += 1
                        self.get_logger().warning(
                            f"Frame loss ({self.failed_frames}/{self.max_retries})"
                        )

                        if self.failed_frames >= self.max_retries:
                            self.get_logger().error(
                                "Camera failure detected. Reconnecting..."
                            )
                            delay = min(2**self.failed_frames, 10)
                            self.get_logger().info(
                                f"Waiting {delay}s before reconnect..."
                            )
                            time.sleep(delay)

                            self.cap.release()
                            self.cap = cv2.VideoCapture(self.camera_index)

                            if not self.cap.isOpened():
                                self.get_logger().error(
                                    "Reconnect failed. Retrying later..."
                                )
                                self.failed_frames = self.max_retries
                            else:
                                self.get_logger().info("Camera reconnected!")
                                self.failed_frames = 0
                            return
                        return

                    self.failed_frames = 0
                    frame = self.process_frame(frame)

                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_frame"
                self.publisher.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f"Frame processing failure: {str(e)}")
            self.on_deactivate(self.get_current_state())


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ImagePublisher()

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
