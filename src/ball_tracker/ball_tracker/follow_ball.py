import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PointStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer, TransformException
from tf2_geometry_msgs import do_transform_point
import math
from math import atan2, sqrt, cos, sin
import numpy as np
import heapq

class AStarPlanner:
    def __init__(self, grid, width, height):
        self.grid = np.array(grid, dtype=np.int8).reshape((height, width))
        self.width = width
        self.height = height

    def is_valid(self, x, y):
        return (0 <= x < self.width and 
                0 <= y < self.height and 
                self.grid[y, x] <= 50)

    def get_neighbors(self, node):
        x, y = node
        neighbors = [
            (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
            (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x - 1, y + 1)
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
                if next_node not in cost_so_far or new_cost < cost_so_far.get(next_node, float('inf')):
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
        super().__init__('follow_ball')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('base_speed', 0.2),
                ('max_speed', 0.3),
                ('angular_gain', 0.8),
                ('stop_distance', 0.3),
                ('search_speed', 0.5),
                ('map_resolution', 0.05),
                ('map_frame', 'map'),
                ('robot_frame', 'base_link'),
                ('camera_frame', 'left_camera_link'),
            ]
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PointStamped, '/ball_positions', self.ball_callback, 10)
        self.create_subscription(OccupancyGrid, '/orb_slam3/map', self.map_callback, 10)
        
        self.ball_position = None
        self.ball_frame = None
        self.ball_stamp = None
        self.current_map = None
        self.robot_pose = None
        self.path = []
        self.last_goal = None
        self.path_update_threshold = 0.7
        self.last_plan_time = self.get_clock().now()
        self.search_mode_start_time = None
        self.search_attempt_counter = 0
        
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.create_timer(0.1, self.navigation_loop)

    def ball_callback(self, msg):
        self.ball_position = msg.point
        self.ball_frame = msg.header.frame_id
        self.ball_stamp = msg.header.stamp
        self.search_mode_start_time = None
        self.search_attempt_counter = 0

    def map_callback(self, msg):
        self.current_map = msg

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
            return True
        except TransformException as e:
            self.get_logger().warn(f"Transform error: {str(e)} - Stopping robot")
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return False

    def is_valid_cell(self, x, y):
        grid_array = np.array(self.current_map.data, dtype=np.int8).reshape(
            (self.current_map.info.height, self.current_map.info.width))
        return (0 <= x < self.current_map.info.width and 
                0 <= y < self.current_map.info.height and 
                grid_array[y, x] <= 50)

    def navigation_loop(self):
        if not self.get_robot_pose():
            return
            
        if None in (self.robot_pose, self.ball_position, self.current_map, self.ball_frame, self.ball_stamp):
            return

        try:
            # transform ball position from camera frame to map frame
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.ball_frame, self.ball_stamp,
                timeout=rclpy.duration.Duration(seconds=1.0))
            transformed_point = do_transform_point(self.ball_position, transform)
            x_map = transformed_point.x
            y_map = transformed_point.y
            distance = sqrt((x_map - self.robot_pose[0])**2 + (y_map - self.robot_pose[1])**2)
        except TransformException as e:
            self.get_logger().warn(f"Transform error: {e} - Stopping robot")
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return

        stop_dist = self.get_parameter('stop_distance').value
        if distance < stop_dist:
            speed_multiplier = max(0.3, (distance / stop_dist)**2)
        else:
            speed_multiplier = 1.0

        map_info = self.current_map.info
        res = map_info.resolution
        ox = map_info.origin.position.x
        oy = map_info.origin.position.y
        start = (int((self.robot_pose[0] - ox) / res), int((self.robot_pose[1] - oy) / res))
        goal = (
            min(max(int((x_map - ox) / res), 0), map_info.width - 1),
            min(max(int((y_map - oy) / res), 0), map_info.height - 1)
        )

        if not self.is_valid_cell(goal[0], goal[1]):
            self.get_logger().warn("Ball outside mapped area. Ignoring target.")
            return

        current_time = self.get_clock().now()
        time_since_last_plan = (current_time - self.last_plan_time).nanoseconds / 1e9
        
        replan_condition = (
            self.last_goal is None or 
            sqrt((x_map - self.last_goal[0])**2 + (y_map - self.last_goal[1])**2) > self.path_update_threshold
        )

        if replan_condition and time_since_last_plan > 3:
            if self.is_valid_cell(goal[0], goal[1]):
                planner = AStarPlanner(self.current_map.data, map_info.width, map_info.height)
                self.path = planner.plan(start, goal)
                self.last_goal = (x_map, y_map)
                self.last_plan_time = current_time

        if not self.path:
            self.search_attempt_counter += 1
            self.get_logger().warn(f"Path not found. Escape attempt #{self.search_attempt_counter}")
            
            if self.search_mode_start_time is None:
                self.search_mode_start_time = self.get_clock().now()
            
            search_duration = (self.get_clock().now() - self.search_mode_start_time).nanoseconds / 1e9
            
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
        angle_error = (target_angle - self.robot_pose[2] + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()
        twist.linear.x = min(
            self.get_parameter('base_speed').value * speed_multiplier,
            self.get_parameter('max_speed').value * distance
        )
        twist.angular.z = max(-1.5, min(1.5, 
            self.get_parameter('angular_gain').value * angle_error * distance
        ))

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowBall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()