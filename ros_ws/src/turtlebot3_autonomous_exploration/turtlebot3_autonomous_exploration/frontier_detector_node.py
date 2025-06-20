import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')  # Initialize the ROS2 node with name 'frontier_detector'

        # Subscribe to the occupancy grid map
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # Subscribe to the global costmap for filtering frontiers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10
        )

        # Publisher for the output frontier centroids as a Path message
        self.pub = self.create_publisher(Path, '/frontier_centroids', 10)

        self.costmap = None  # To store the latest costmap
        self.get_logger().info('FrontierDetector node initialized.')

    def costmap_callback(self, msg):
        # Store the received costmap message
        self.costmap = msg

    def map_callback(self, msg):
        # Ensure the costmap is available before processing the map
        if self.costmap is None:
            self.get_logger().warn('Costmap not yet available.')
            return

        # Extract map metadata
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        # Convert the map data to a 2D NumPy array
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Identify free and unknown cells
        free = (data == 0)
        unknown = (data == -1)

        # Detect frontier points (free cells adjacent to unknown cells)
        frontier_points = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if free[y, x]:
                    # Check 3x3 neighborhood for unknown cells
                    neighborhood_unknown = unknown[y-1:y+2, x-1:x+2]
                    if np.any(neighborhood_unknown):
                        # Convert map coordinates to world coordinates
                        wx = origin.x + (x + 0.5) * resolution
                        wy = origin.y + (y + 0.5) * resolution
                        frontier_points.append([wx, wy])

        if not frontier_points:
            self.get_logger().info('No frontier points found.')
            return

        pts = np.array(frontier_points)

        # Perform simple distance-based clustering
        centroids = self.cluster_frontiers(pts, radius=0.3)

        # Filter centroids using global costmap
        costmap_data = np.array(self.costmap.data, dtype=np.int8).reshape(
            (self.costmap.info.height, self.costmap.info.width)
        )
        costmap_resolution = self.costmap.info.resolution
        costmap_origin = self.costmap.info.origin.position

        safe_centroids = []
        for cx, cy in centroids:
            # Convert world coordinates to costmap cell indices
            mx = int((cx - costmap_origin.x) / costmap_resolution)
            my = int((cy - costmap_origin.y) / costmap_resolution)

            # Check if the centroid lies within bounds
            if 0 <= mx < self.costmap.info.width and 0 <= my < self.costmap.info.height:
                cost = costmap_data[my, mx]
                # Keep only centroids in low-cost areas (safe)
                if 0 <= cost < 75:
                    safe_centroids.append([cx, cy])

        # Build and publish Path message with safe centroids
        path = Path()
        path.header = msg.header  # Inherit header from map message

        for cx, cy in safe_centroids[:100]:  # Limit to 100 centroids
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = cx
            pose.pose.position.y = cy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 
            path.poses.append(pose)

        self.pub.publish(path)
        self.get_logger().info(f'Published {len(path.poses)} frontier centroids (filtered with costmap).')

    def cluster_frontiers(self, points, radius=0.1):
        clusters = []
        used = np.zeros(len(points), dtype=bool)

        for i, pt in enumerate(points):
            if used[i]:
                continue
            cluster = [pt]
            used[i] = True
            for j in range(i + 1, len(points)):
                if not used[j] and np.linalg.norm(pt - points[j]) < radius:
                    cluster.append(points[j])
                    used[j] = True
            clusters.append(np.array(cluster))

        # Return the centroid of each cluster
        centroids = [c.mean(axis=0) for c in clusters if len(c) > 0]
        return centroids

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = FrontierDetector()  # Instantiate the node
    rclpy.spin(node)  # Keep the node running, waiting for callbacks
    node.destroy_node()  # Destroy the node when done
    rclpy.shutdown()  # Shutdown the ROS 2 client