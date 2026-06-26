#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq
import math

class ClickedGoalSub(Node):
    pass # Used if you want to expand later, but keeping everything in PathPlanner for simplicity

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # --- Subscribers ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )

        # --- Publishers ---
        self.path_pub = self.create_publisher(
            Path, 'global_path', 10
        )

        # Hardcoded start and goal for testing (in meters)
        # In production, you'll subscribe to /initialpose and /goal_pose
        self.start_x = 0.0
        self.start_y = 0.0
        self.goal_x = 3.0
        self.goal_y = 3.0

        self.get_logger().info("A* Path Planner Node successfully started.")

    def map_callback(self, msg):
        self.get_logger().info("Processing new map for A* planning...", throttle_duration_sec=4.0)

        # 1. Convert world coordinates (meters) to 2D grid coordinates
        start_grid = self.world_to_grid(self.start_x, self.start_y, msg.info)
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y, msg.info)

        # 2. Run the A* algorithm on the matrix grid
        grid_path = self.a_star(start_grid, goal_grid, msg)

        if not grid_path:
            self.get_logger().warn("A* failed to find a valid path!")
            return

        # 3. Convert grid path back to ROS nav_msgs/Path (meters)
        path_msg = Path()
        path_msg.header.frame_id = msg.header.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for cell in grid_path:
            world_x, world_y = self.grid_to_world(cell[0], cell[1], msg.info)

            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Successfully published A* path with {len(grid_path)} waypoints.")

    def world_to_grid(self, wx, wy, geo):
        """Converts world coordinates (meters) to 2D Grid Array Indices."""
        gx = int((wx - geo.origin.position.x) / geo.resolution)
        gy = int((wy - geo.origin.position.y) / geo.resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy, geo):
        """Converts 2D Grid Array Indices back to world coordinates (meters)."""
        wx = geo.origin.position.x + (gx + 0.5) * geo.resolution
        wy = geo.origin.position.y + (gy + 0.5) * geo.resolution
        return (wx, wy)

    def heuristic(self, a, b):
        """Standard Euclidean Distance Heuristic."""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, node, msg):
        """Returns valid 8-connected neighbors (includes diagonals)."""
        width = msg.info.width
        height = msg.info.height
        neighbors = []

        # 8-directional movement offsets
        directions = [
            (0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0), # Cardinal
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414) # Diagonals
        ]

        for dx, dy, cost in directions:
            nx, ny = node[0] + dx, node[1] + dy

            # Check map boundary bounds
            if 0 <= nx < width and 0 <= ny < height:
                # 1D array indexing formula: index = y * width + x
                index = ny * width + nx
                occupancy_value = msg.data[index]

                # Treat unmapped (-1) or heavily blocked cells (> 50) as obstacles
                if 0 <= occupancy_value < 50:
                    neighbors.append(((nx, ny), cost))

        return neighbors

    def a_star(self, start, goal, msg):
        """Core A* Search Algorithm."""
        # Priority Queue elements format: (f_score, current_node)
        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}

        # Cost from start to current node
        g_score = {start: 0.0}

        # Estimated cost from start to goal through current node
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            # Goal reached! Reconstruct the path backwards.
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1] # Reverse it to get start -> goal

            for neighbor, step_cost in self.get_neighbors(current, msg):
                tentative_g_score = g_score[current] + step_cost

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    # If neighbor not already evaluated, push it to open set
                    if not any(item[1] == neighbor for item in open_set):
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None # Return None if no path is mathematically possible

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
