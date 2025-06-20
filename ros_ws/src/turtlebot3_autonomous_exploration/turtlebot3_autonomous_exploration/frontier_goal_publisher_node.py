import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry, Path
import math

class FrontierGoalPublisherNode(Node):
    def __init__(self):
        super().__init__('frontier_goal_publisher')  # Initialize the ROS2 node with a name
        self.centroids = []  # List of received frontier centroids (x, y)
        self.new_centroids_received = False  # Flag indicating whether new centroids were received
        self.current_pose = None  # Current robot pose from odometry
        self.sent_goal = False  # Flag to prevent sending multiple goals simultaneously
        self.returning_home = False  # Flag to indicate return-to-home behavior
        self.min_distance_threshold = 0.5  # Minimum distance to consider a centroid valid (in meters)
        self.last_goal_position = None  # To store last goal position (x, y)

        self.last_movement_time = self.get_clock().now()  # Last time the robot was detected as moving

        self.starting_pose = None  # Stores the starting position to return to
        self.goal_handle = None  # Handle for the currently active navigation goal

        # Timer to periodically check for robot inactivity or invalid frontiers
        self.stuck_check_timer = self.create_timer(20.0, self.check_stuck_or_invalid_frontiers)

        # Subscription to frontier centroids published as a Path
        self.centroid_sub = self.create_subscription(
            Path,
            '/frontier_centroids',
            self.frontier_callback,
            10
        )

        # Subscription to filtered odometry to update current robot pose
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/filtered',
            self.odom_callback,
            10
        )

        # Action client for sending goals to the Nav2 navigation server
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def odom_callback(self, msg):
        # Update current robot pose
        self.current_pose = msg.pose.pose

        # Store starting pose if not already set
        if self.starting_pose is None:
            self.starting_pose = msg.pose.pose

        # Calculate linear speed from odometry
        linear_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if linear_speed > 0.01:
            self.last_movement_time = self.get_clock().now()  # Update last movement time

        self.try_send_goal()  # Attempt to send a new goal if possible

    def frontier_callback(self, msg):
        # Extract centroids as (x, y) tuples from the received Path message
        self.centroids = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.new_centroids_received = True  # Flag indicating new data was received
        self.try_send_goal()  # Try to send a goal based on updated centroids

    def try_send_goal(self):
        # Do nothing if current pose or centroids are missing
        if self.current_pose is None or not self.centroids:
            return

        # Extract current robot position
        x, y = self.current_pose.position.x, self.current_pose.position.y

        # Filter out centroids that are too close
        valid_centroids = [
            (cx, cy) for cx, cy in self.centroids
            if math.hypot(cx - x, cy - y) > self.min_distance_threshold
        ]

        if not valid_centroids:
            return  # No valid frontiers to send a goal to

        # Select the closest valid frontier
        closest = min(valid_centroids, key=lambda p: math.hypot(p[0] - x, p[1] - y))

        # If returning home and a valid centroid is found, cancel return
        if self.returning_home:
            self.get_logger().info("Cancelling return home: valid centroid found.")
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.reset_after_return()

        if self.sent_goal:
            return  # Do not send a new goal if one is already active
        
        # Check if the new goal is too close to the last one
        if self.last_goal_position:
            last_x, last_y = self.last_goal_position
            distance_to_last_goal = math.hypot(closest[0] - last_x, closest[1] - last_y)
            if distance_to_last_goal < self.min_distance_threshold:
                self.get_logger().warn(f"Rejected goal {closest} - too close to last goal {self.last_goal_position}")
                return

        self.get_logger().info(f"Sending goal to frontier: {closest}")
        self.send_navigation_goal(closest[0], closest[1])
        self.last_goal_position = closest

    def send_navigation_goal(self, x_goal, y_goal):
        # Compute yaw angle to face the goal
        dx = x_goal - self.current_pose.position.x
        dy = y_goal - self.current_pose.position.y
        yaw = math.atan2(dy, dx)
        orientation = self.quaternion_from_yaw(yaw)

        # Build the NavigateToPose goal message
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x_goal
        goal_pose.pose.position.y = y_goal
        goal_pose.pose.orientation = orientation
        goal_msg.pose = goal_pose

        # Wait for the navigation server to become available
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 server not available.")
            return

        # Send the goal asynchronously and register a callback
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.sent_goal = True
        self.new_centroids_received = False
        self.last_goal_time = self.get_clock().now()

    def quaternion_from_yaw(self, yaw):
        # Converts a yaw angle (in radians) into a Quaternion message
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()  # Retrieve the goal handle
            self.goal_handle = goal_handle
        except Exception as e:
            self.get_logger().error(f"Error during goal response: {e}")
            self.reset_after_return()
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2.')
            self.reset_after_return()
            return

        self.get_logger().info('Goal accepted by Nav2.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            result = future.result()  # Access the result of the navigation goal
            self.get_logger().info(f'Goal finished with status: {result.status}')
        except Exception as e:
            self.get_logger().error(f"Error while retrieving goal result: {e}")

        self.reset_after_return()  # Reset internal state regardless of result

    def check_stuck_or_invalid_frontiers(self):
        # Check if the robot is stuck or has no reachable frontiers
        now = self.get_clock().now()
        time_since_move = (now - self.last_movement_time).nanoseconds * 1e-9

        if time_since_move > 20.0 or not self.centroids or self.all_frontiers_too_close():
            self.get_logger().warn("Returning to start due to inactivity or lack of valid frontiers.")
            self.return_to_start()

    def all_frontiers_too_close(self):
        # Returns True if all centroids are too close to the current position
        if self.current_pose:
            x, y = self.current_pose.position.x, self.current_pose.position.y
            return all(
                math.hypot(cx - x, cy - y) <= self.min_distance_threshold
                for cx, cy in self.centroids
            )
        return False

    def return_to_start(self):
        # Sends a navigation goal to return to the starting position
        if self.sent_goal or not self.starting_pose:
            return

        self.get_logger().info("Sending goal to return to starting position.")
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = self.starting_pose
        goal_msg.pose = goal_pose

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 server not available for return-to-home.")
            return

        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.returning_home = True
        self.sent_goal = True

    def reset_after_return(self):
        # Resets internal flags and data after goal completion or cancellation
        self.returning_home = False
        self.sent_goal = False
        self.centroids.clear()
        self.new_centroids_received = False
        self.goal_handle = None
        self.last_goal_position = None

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 communication
    node = FrontierGoalPublisherNode()  # Create the node instance
    rclpy.spin(node)  # Keep the node alive and processing callbacks
    node.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shutdown ROS 2