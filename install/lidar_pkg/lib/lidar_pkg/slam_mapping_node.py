#!/usr/bin/env python3
import numpy as np
from sklearn.neighbors import NearestNeighbors

import rclpy
from rclpy.node import Node
from lidar_msgs.msg import LidarSweep
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

# ----------------------------------------------------------------------
# Math Core Functions (Preserved from your original script)
# ----------------------------------------------------------------------
def polar_to_cartesian(angles_deg, distances_mm):
    angles_rad = np.deg2rad(angles_deg)
    return np.column_stack([distances_mm * np.cos(angles_rad), distances_mm * np.sin(angles_rad)])

def best_fit_transform(A, B):
    centroid_A, centroid_B = np.mean(A, axis=0), np.mean(B, axis=0)
    H = np.dot((A - centroid_A).T, B - centroid_B)
    U, _, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)
    t = centroid_B.T - np.dot(R, centroid_A.T)
    T = np.identity(3)
    T[:2, :2], T[:2, 2] = R, t
    return T

def icp(A, B, max_iterations=50, tolerance=1e-5, reject_ratio=0.1):
    src = np.ones((3, A.shape[0]))
    src[:2, :] = A.T
    neigh = NearestNeighbors(n_neighbors=1).fit(B)
    prev_error = None
    for _ in range(max_iterations):
        distances, indices = neigh.kneighbors(src[:2, :].T, return_distance=True)
        distances, indices = distances.ravel(), indices.ravel()
        keep = int(len(distances) * (1 - reject_ratio)) if reject_ratio else len(distances)
        order = np.argsort(distances)[:max(keep, 4)]
        T = best_fit_transform(src[:2, order].T, B[indices[order]])
        src = np.dot(T, src)
        mean_error = np.mean(distances[order])
        if prev_error is not None and np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    return best_fit_transform(A, src[:2, :].T)

def bresenham_line(x0, y0, x1, y1):
    points = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        if x0 == x1 and y0 == y1:
            break
        points.append((x0, y0))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points

# ----------------------------------------------------------------------
# ROS 2 Mapping Node Wrapper
# ----------------------------------------------------------------------
class SlamMappingNode(Node):

    def __init__(self):
        super().__init__('slam_mapping_node')
        
        # Configuration parameters
        self.declare_parameter('grid_resolution_m', 0.01) # 1cm = 0.01 meters
        self._res_m = self.get_parameter('grid_resolution_m').value
        self._res_mm = self._res_m * 1000.0

        # Memory buffers to stack scan maps over time
        self._all_poses = [np.identity(3)]  # Start pose at (0,0,0)
        self._all_local_scans = []
        
        # Subscribers and Publishers
        self._scan_sub = self.create_subscription(LidarSweep, '/lidar_scan_node/scan', self._scan_callback, 10)
        self._map_pub = self.create_publisher(OccupancyGrid, '~/map', 10)
        
        self.get_logger().info("SLAM Mapping Node online. Processing map sweeps dynamically...")

    def _scan_callback(self, msg: LidarSweep):
        if len(msg.distances) == 0:
            return

        # 1. Process current scan frame coordinates
        current_local = polar_to_cartesian(msg.angles, msg.distances)
        
        # 2. Track matching poses dynamically using sequential scan steps
        if len(self._all_local_scans) > 0:
            T_rel = icp(current_local, self._all_local_scans[-1])
            current_pose = self._all_poses[-1] @ T_rel
            self._all_poses.append(current_pose)
        
        self._all_local_scans.append(current_local)
        
        # 3. Generate Global Map Matrices
        self._update_and_publish_map(msg.header.frame_id)

    def _update_and_publish_map(self, frame_id):
        all_rays = []
        all_global_pts = []
        
        # Gather global positions
        for i, local_pts in enumerate(self._all_local_scans):
            T = self._all_poses[i]
            robot_pos = T[:2, 2]
            
            homo_pts = np.ones((local_pts.shape[0], 3))
            homo_pts[:, :2] = local_pts
            global_pts = (T @ homo_pts.T).T[:, :2]
            all_global_pts.append(global_pts)
            
            for pt in global_pts:
                all_rays.append((robot_pos[0], robot_pos[1], pt[0], pt[1]))

        # Calculate space dimensions bounds
        all_pts_flat = np.vstack(all_global_pts)
        all_poses_flat = np.array([p[:2, 2] for p in self._all_poses])
        all_coords = np.vstack([all_pts_flat, all_poses_flat])
        
        min_x, min_y = np.min(all_coords, axis=0) - 500  # 500mm padding
        max_x, max_y = np.max(all_coords, axis=0) + 500
        
        width = int(np.ceil((max_x - min_x) / self._res_mm))
        height = int(np.ceil((max_y - min_y) / self._res_mm))
        
        # Base Matrix: Start filled entirely with Unknown values (-1)
        grid = np.full((height, width), -1, dtype=np.int8)
        
        def to_grid(x, y):
            return int(np.floor((x - min_x) / self._res_mm)), int(np.floor((y - min_y) / self._res_mm))

        # Raycasting Execution Pass
        for rx, ry, ex, ey in all_rays:
            gx0, gy0 = to_grid(rx, ry)
            gx1, gy1 = to_grid(ex, ey)
            
            if not (0 <= gx0 < width and 0 <= gy0 < height and 0 <= gx1 < width and 0 <= gy1 < height):
                continue
                
            free_pixels = bresenham_line(gx0, gy0, gx1, gy1)
            for fx, fy in free_pixels:
                if grid[fy, fx] != 100: 
                    grid[fy, fx] = 0   # Free Space
                    
            grid[gy1, gx1] = 100       # Obstacle Wall Block

        # 4. Construct ROS 2 OccupancyGrid Message Output Packet
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = frame_id # matches map base coordinate standard
        
        map_msg.info.resolution = self._res_m
        map_msg.info.width = width
        map_msg.info.height = height
        
        # Map origin pose offsets (converted from mm back to standard meters)
        origin_pose = Pose()
        origin_pose.position.x = float(min_x / 1000.0)
        origin_pose.position.y = float(min_y / 1000.0)
        origin_pose.position.z = 0.0
        map_msg.info.origin = origin_pose
        
        # Flatten the grid matrix row-by-row to pack it into standard int8 array data structure
        map_msg.data = grid.ravel().tolist()
        
        self._map_pub.publish(map_msg)
        self.get_logger().info(f"Published updated map grid layout: {width}x{height} pixels.")


def main(args=None):
    rclpy.init(args=args)
    node = SlamMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
