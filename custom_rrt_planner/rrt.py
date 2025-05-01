import copy
import random
import math
from math import pi, sqrt
import time
import threading

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path, MapMetaData

from tf2_ros import TransformListener, Buffer


class RRTPlanner:
    def __init__(self, map_grid):
        self.tree = []
        self.map_grid = map_grid
        self.height, self.width = map_grid.shape
        self.bounds = [(0, self.width - 1), (0, self.height - 1)]

    def is_colliding(self, q):
        x, y = int(q[0]), int(q[1])
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True

        for dx in range(-2, 2+1):
            for dy in range(-2, 2+1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if self.map_grid[ny, nx] == 100:
                        return True
                else:
                    return True

        return False

    def interpolate(self, qA, qB, t):
        return [(1 - t) * a + t * b for a, b in zip(qA, qB)]

    def distance(self, q1, q2):
        return sqrt(sum((a - b) ** 2 for a, b in zip(q1, q2)))

    def random_sample(self, goal, goal_bias=0.05):
        if random.random() < goal_bias:
            return goal

        while True:
            q_rand = [
                random.uniform(*self.bounds[0]),
                random.uniform(*self.bounds[1])
            ]

            if not self.is_colliding(q_rand):
                return q_rand

    def nearest_node(self, q_rand):
        min_dist = float("inf")
        q_near = None

        for node in self.tree:
            dist = self.distance(node["config"], q_rand)
            if dist < min_dist:
                min_dist = dist
                q_near = node
        
        return q_near

    def is_subpath_collision_free(self, q_near, q_rand, step=0.05):
        t = step

        while t <= 1.0:
            q_interp = self.interpolate(q_near["config"], q_rand, t)
            
            if self.is_colliding(q_interp):
                return False
            
            t += step

        return True

    def search_path(self, q_init, q_goal_node):
        path = []
        current = q_goal_node

        while current is not None:
            path.append(current["config"])
            current = current["parent"]

        return path[::-1]

    def rrt(self, q_init, q_goal, goal_bias, max_iter=1000, goal_threshold=0.1):
        self.tree = [{"config": q_init, "parent": None}]

        for _ in range(max_iter):
            q_rand = self.random_sample(q_goal, goal_bias)
            q_near = self.nearest_node(q_rand)
            q_new = self.interpolate(q_near["config"], q_rand, 0.1)

            if self.is_subpath_collision_free(q_near, q_new):
                new_node = {"config": q_new, "parent": q_near}
                self.tree.append(new_node)

                if self.distance(q_new, q_goal) < goal_threshold:
                    return self.search_path(q_init, new_node)

        return []


class RRTNode(Node):
    def __init__(self):
        super().__init__('custom_rrt_planner')

        self.get_logger().info("Starting RRT Planner Node!")

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/rrt_goal', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos) # so we don't miss publish to /map
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.amcl_pose = None
        self.odom_pose = None
        self.goal_pose = None
        self.map_data = None

        self.timer = self.create_timer(0.05, self.control_loop)
        self.world_path = []
        self.path_index = 0
        self.following_path = False

        self.odom_timer = self.create_timer(0.05, self.odom_update)

    def odom_update(self):
        if self.odom_pose is None or self.amcl_pose is None:
            return

        if not hasattr(self, 'odom_to_map_offset'):
            dx = self.amcl_pose.position.x - self.odom_pose.position.x
            dy = self.amcl_pose.position.y - self.odom_pose.position.y
            self.odom_to_map_offset = (dx, dy)
            self.get_logger().info(f"Stored odom → map offset: dx={dx:.3f}, dy={dy:.3f}")


    """
    Planner methods
    """

    def world_to_map(self, x, y):
        origin = self.map_data.info.origin
        resolution = self.map_data.info.resolution

        mx = int((x - origin.position.x) / resolution)
        my = int((y - origin.position.y) / resolution)

        return mx, my

    def map_to_world(self, mx, my):
        origin = self.map_data.info.origin
        resolution = self.map_data.info.resolution

        x = mx * resolution + origin.position.x
        y = my * resolution + origin.position.y

        return x, y

    def pose_callback(self, msg):
        self.amcl_pose = msg.pose.pose

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose

        # compute offset once both poses available
        if self.amcl_pose is not None and not hasattr(self, 'odom_to_map_offset'):
            dx = self.amcl_pose.position.x - self.odom_pose.position.x
            dy = self.amcl_pose.position.y - self.odom_pose.position.y

            self.odom_to_map_offset = (dx, dy)

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"Received goal: {self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f}")

        if self.amcl_pose is not None and self.map_data is not None:
            self.get_logger().info("Starting RRT in a new thread.")
            threading.Thread(target=self.rrt()).start()
        else:
            self.get_logger().warn("Waiting for current pose and map before running RRT.")

    def map_callback(self, msg):
        self.map_data = msg
        self.map_grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def rrt(self):
        self.get_logger().info(f"Starting RRT with goal: {self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f}")

        start_x, start_y = self.amcl_pose.position.x, self.amcl_pose.position.y
        goal_x, goal_y = self.goal_pose.position.x, self.goal_pose.position.y

        start_mx, start_my = self.world_to_map(start_x, start_y)
        goal_mx, goal_my = self.world_to_map(goal_x, goal_y)

        # create RRT Planner
        planner = RRTPlanner(self.map_grid)


        # define bounds
        bounds = [(0, self.map_data.info.width - 1), (0, self.map_data.info.height - 1)]

        # run RRT
        path = planner.rrt(
            q_init=[start_mx, start_my],
            q_goal=[goal_mx, goal_my],
            goal_bias=0.05,
            max_iter=1000,
        )

        if path:
            self.get_logger().info(f"Path found with {len(path)} waypoints!")
            for point in path:
                world_x, world_y = self.map_to_world(*point)
                self.get_logger().info(f" → ({world_x:.2f}, {world_y:.2f})")

            self.get_logger().info("Following path...")
            world_path = [self.map_to_world(x, y) for x, y in path]
            self.follow_path(world_path)

        else:
            self.get_logger().warn("Failed to find a path.")


    """
    Follow path and helper methods.
    """

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        if not self.following_path:
            return

        if self.path_index >= len(self.world_path):
            self.get_logger().info("Path complete!")
            self.following_path = False
            self.stop_robot()
            return

        target_x, target_y = self.world_path[self.path_index]

        if self.odom_pose is None or not hasattr(self, 'odom_to_map_offset'):
            self.get_logger().warn("No odom or offset yet.")
            return

        xoff, yoff = self.odom_to_map_offset
        current_x = self.odom_pose.position.x + xoff
        current_y = self.odom_pose.position.y + yoff
        yaw = self.get_yaw_from_quaternion(self.odom_pose.orientation)

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        heading_error = self.normalize_angle(angle_to_target - yaw)

        twist = Twist()

        # control parameters
        k_linear = 0.5
        k_angular = 3.0
        max_linear_speed = 0.22
        max_angular_speed = 2.84
        distance_tolerance = 0.05
        angle_tolerance = 0.2  # if heading error > ~10 deg, stop and rotate
        small_angle_threshold = 0.1  # below this, allow moving with heading correction

        if distance > distance_tolerance:
            if abs(heading_error) > angle_tolerance:
                # rotate in place
                twist.angular.z = k_angular * heading_error
                twist.angular.z = max(-max_angular_speed, min(twist.angular.z, max_angular_speed))
                twist.linear.x = 0.0
            else:
                # move and rotate
                twist.linear.x = k_linear * distance
                twist.linear.x = min(twist.linear.x, max_linear_speed)

                twist.angular.z = k_angular * heading_error
                twist.angular.z = max(-max_angular_speed, min(twist.angular.z, max_angular_speed))

        else:
            self.get_logger().info(f"Reached waypoint {self.path_index}!")
            self.path_index += 1
            self.stop_robot()
            return

        self.cmd_pub.publish(twist)

    def follow_path(self, world_path):
        self.world_path = world_path
        self.path_index = 0
        self.following_path = True
        self.get_logger().info("Started following path.")


def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin() # multi-threaded
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
