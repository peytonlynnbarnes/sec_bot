import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
import math
from math import atan2, sqrt, cos, sin
import numpy as np
import heapq


class AStarPlanner:
    # This class doesn't need changes as it's algorithm-based, not ROS-specific
    def __init__(self, grid, width, height):
        self.grid = np.array(grid, dtype=np.int8).reshape((height, width))
        self.width = width
        self.height = height

    def is_valid(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y, x] <= 50

    def get_neighbors(self, node):
        x, y = node
        neighbors = [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
            (x + 1, y + 1),
            (x - 1, y - 1),
            (x + 1, y - 1),
            (x - 1, y + 1),
        ]
        return [n for n in neighbors if self.is_valid(n[0], n[1])]

    def heuristic(self, a, b):
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)

    def plan(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (0, start))
        came_from = {}
        cost_so_far = {start: 0}
        came_from[start] = None

        while open_heap:
            current = heapq.heappop(open_heap)[1]

            if current == goal:
                break

            for next_node in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.heuristic(current, next_node)
                if next_node not in cost_so_far or new_cost < cost_so_far.get(
                    next_node, float("inf")
                ):
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    heapq.heappush(open_heap, (priority, next_node))
                    came_from[next_node] = current

        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                return None
        path.append(start)
        path.reverse()
        return path


class FollowBall(Node):
    def __init__(self):
        super().__init__("follow_ball")

        self.declare_parameters(
            namespace="",
            parameters=[
<<<<<<< HEAD
                ('base_speed', 0.2),
                ('max_speed', 0.3),
                ('angular_gain', 0.8),
                ('stop_distance', 0.3),
                ('search_speed', 0.5),
                ('fov', 1.0),
                ('map_resolution', 0.05),
                ('ball_scale_factor', 0.05),
                ('stereo_baseline', 0.12),
                ('focal_length', 525.0),
                # Add parameters for frame IDs - these might need to be changed for ORB-SLAM3
                ('map_frame', 'map'),         # NEW: Parameter for map frame
                ('robot_frame', 'base_link')  # NEW: Parameter for robot frame
            ])
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Point, '/ball_positions', self.ball_callback, 10)
        
        # Update the map topic to match what ORB-SLAM3 publishes
        # This might need to be changed depending on the actual topic from ros2_orb_slam3
        self.create_subscription(OccupancyGrid, '/orb_slam3/map', self.map_callback, 10)
        
=======
                ("base_speed", 0.2),
                ("max_speed", 0.3),
                ("angular_gain", 0.8),
                ("stop_distance", 0.3),
                ("search_speed", 0.5),
                ("fov", 1.0),
                ("map_resolution", 0.05),
                ("ball_scale_factor", 0.05),
                ("orbslam_focal_length", 525.0),  # ORB-SLAM3 camera parameter
                ("keypoint_threshold", 100),  # Min keypoints for reliable tracking
                ("slam_lost_timeout", 3.0),  # Time to wait before declaring SLAM lost
            ],
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribe to ORB-SLAM3 topics
        self.create_subscription(
            PoseStamped, "/orbslam3/camera_pose", self.slam_pose_callback, 10
        )
        self.create_subscription(Point, "/ball_positions", self.ball_callback, 10)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)

        # Optional: Subscribe to keypoint count if ORB-SLAM3 publishes it
        self.create_subscription(
            Point, "/orbslam3/tracking_status", self.tracking_status_callback, 10
        )

>>>>>>> 796edf8fc2a7f684a27c4111f5fbd71e0103864f
        self.ball_position = None
        self.current_map = None
        self.robot_pose = None
        self.path = []
        self.last_goal = None
        self.path_update_threshold = 0.7
        self.last_plan_time = self.get_clock().now()
        self.search_mode_start_time = None
        self.search_attempt_counter = 0
<<<<<<< HEAD
        
        # Get frame IDs from parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

    def ball_callback(self, msg):
        baseline = self.get_parameter('stereo_baseline').value
        focal_length = self.get_parameter('focal_length').value
        
        if msg.x != 0:
            depth = (focal_length * baseline) / msg.x
        else:
            depth = 0.0
            
=======
        self.slam_tracking_quality = "GOOD"  # Default tracking status
        self.last_slam_update = self.get_clock().now()
        self.keypoint_count = 0

    def tracking_status_callback(self, msg):
        # Assuming ORB-SLAM3 publishes tracking status as a Point message
        # x: keypoint count, y: status code (1=GOOD, 2=POOR, 0=LOST)
        self.keypoint_count = int(msg.x)
        status_code = int(msg.y)

        if status_code == 1:
            self.slam_tracking_quality = "GOOD"
        elif status_code == 2:
            self.slam_tracking_quality = "POOR"
        else:
            self.slam_tracking_quality = "LOST"

        self.last_slam_update = self.get_clock().now()

    def slam_pose_callback(self, msg):
        # Extract pose from ORB-SLAM3
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Extract orientation from quaternion
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Convert quaternion to yaw (theta)
        theta = 2 * math.atan2(qz, qw)

        self.robot_pose = (x, y, theta)
        self.last_slam_update = self.get_clock().now()

    def ball_callback(self, msg):
        # Process ball position from stereo or RGB-D camera
        # For ORB-SLAM3, the depth might come directly from the camera
        focal_length = self.get_parameter("orbslam_focal_length").value

        # Assuming depth is already available from RGB-D camera
        if hasattr(msg, "z") and msg.z > 0:
            depth = msg.z
        else:
            # Fallback to estimating depth if needed
            # Note: This is a simplified approach and may need adjustment
            depth = focal_length / max(0.001, abs(msg.x))

>>>>>>> 796edf8fc2a7f684a27c4111f5fbd71e0103864f
        self.ball_position = Point(x=msg.y, y=msg.z, z=depth)

        self.search_mode_start_time = None
        self.search_attempt_counter = 0

    def map_callback(self, msg):
        self.current_map = msg

<<<<<<< HEAD
    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            quat = transform.transform.rotation
            theta = 2 * math.atan2(quat.z, quat.w)
            self.robot_pose = (x, y, theta)
=======
    def is_slam_reliable(self):
        # Check if SLAM is providing reliable tracking
        current_time = self.get_clock().now()
        time_since_last_update = (
            current_time - self.last_slam_update
        ).nanoseconds / 1e9

        # Check if we've received recent updates and tracking is decent
        if time_since_last_update < self.get_parameter("slam_lost_timeout").value and (
            self.slam_tracking_quality != "LOST"
            or self.keypoint_count >= self.get_parameter("keypoint_threshold").value
        ):
>>>>>>> 796edf8fc2a7f684a27c4111f5fbd71e0103864f
            return True
        except Exception as e:
            self.get_logger().warn(f"Transform error: {str(e)} - Stopping robot")
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return False

    def is_valid_cell(self, x, y):
        grid_array = np.array(self.current_map.data, dtype=np.int8).reshape(
            (self.current_map.info.height, self.current_map.info.width)
        )
        return (
            0 <= x < self.current_map.info.width
            and 0 <= y < self.current_map.info.height
            and grid_array[y, x] <= 50
        )

    def navigation_loop(self):
        if not self.get_robot_pose():
            return

        if None in (self.robot_pose, self.ball_position, self.current_map):
            return

        fov = self.get_parameter("fov").value
        scale_factor = self.get_parameter("ball_scale_factor").value

        distance = self.ball_position.z * scale_factor
        angle = self.ball_position.x * fov

        stop_dist = self.get_parameter("stop_distance").value
        if distance < stop_dist:
            speed_multiplier = max(0.3, (distance / stop_dist) ** 2)
        else:
            speed_multiplier = 1.0

        x_robot = distance * cos(angle)
        y_robot = distance * sin(angle)

        x_map = (
            self.robot_pose[0]
            + x_robot * cos(self.robot_pose[2])
            - y_robot * sin(self.robot_pose[2])
        )
        y_map = (
            self.robot_pose[1]
            + x_robot * sin(self.robot_pose[2])
            + y_robot * cos(self.robot_pose[2])
        )

        map_info = self.current_map.info
        res = map_info.resolution
        ox = map_info.origin.position.x
        oy = map_info.origin.position.y
        start = (
            int((self.robot_pose[0] - ox) / res),
            int((self.robot_pose[1] - oy) / res),
        )
        goal = (
            min(max(int((x_map - ox) / res), 0), map_info.width - 1),
            min(max(int((y_map - oy) / res), 0), map_info.height - 1),
        )

        if not self.is_valid_cell(goal[0], goal[1]):
            self.get_logger().warn("Ball outside mapped area. Ignoring target.")
            return

        current_time = self.get_clock().now()
        time_since_last_plan = (current_time - self.last_plan_time).nanoseconds / 1e9

        replan_condition = (
            self.last_goal is None
            or sqrt((x_map - self.last_goal[0]) ** 2 + (y_map - self.last_goal[1]) ** 2)
            > self.path_update_threshold
        )

        if replan_condition and time_since_last_plan > 3:
            if self.is_valid_cell(goal[0], goal[1]):
                planner = AStarPlanner(
                    self.current_map.data, map_info.width, map_info.height
                )
                self.path = planner.plan(start, goal)
                self.last_goal = (x_map, y_map)
                self.last_plan_time = current_time

        if not self.path:
            self.search_attempt_counter += 1
            self.get_logger().warn(
                f"Path not found. Escape attempt #{self.search_attempt_counter}"
            )

            if self.search_mode_start_time is None:
                self.search_mode_start_time = self.get_clock().now()

            search_duration = (
                self.get_clock().now() - self.search_mode_start_time
            ).nanoseconds / 1e9

            if search_duration > 10:
                self.get_logger().error("No path found after 10s. Stopping.")
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                return

            twist = Twist()
            twist.linear.x = 0.1 * (1 + self.search_attempt_counter % 3)
            twist.angular.z = 0.5 * (-1 if self.search_attempt_counter % 2 else 1)
            self.cmd_vel_pub.publish(twist)
            return

        next_step = self.path[1] if len(self.path) > 1 else self.path[0]
        target_x = next_step[0] * res + ox
        target_y = next_step[1] * res + oy

        dx = target_x - self.robot_pose[0]
        dy = target_y - self.robot_pose[1]
        target_angle = atan2(dy, dx)
        angle_error = (target_angle - self.robot_pose[2] + math.pi) % (
            2 * math.pi
        ) - math.pi

<<<<<<< HEAD
        twist = Twist()
        twist.linear.x = min(
            self.get_parameter('base_speed').value * speed_multiplier,
            self.get_parameter('max_speed').value * distance
=======
        # Reduce speed when SLAM tracking is not optimal
        tracking_factor = 1.0
        if self.slam_tracking_quality == "POOR":
            tracking_factor = 0.7

        twist = Twist()
        twist.linear.x = min(
            self.get_parameter("base_speed").value * speed_multiplier * tracking_factor,
            self.get_parameter("max_speed").value * distance * tracking_factor,
        )
        twist.angular.z = max(
            -1.5,
            min(1.5, self.get_parameter("angular_gain").value * angle_error * distance),
>>>>>>> 796edf8fc2a7f684a27c4111f5fbd71e0103864f
        )

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowBall()
<<<<<<< HEAD
=======

    # Create a timer for the navigation loop
    timer_period = 0.1  # seconds
    timer = node.create_timer(timer_period, node.navigation_loop)

>>>>>>> 796edf8fc2a7f684a27c4111f5fbd71e0103864f
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

<<<<<<< HEAD
if __name__ == '__main__':
    main()
=======

if __name__ == "__main__":
    main()
>>>>>>> 796edf8fc2a7f684a27c4111f5fbd71e0103864f
