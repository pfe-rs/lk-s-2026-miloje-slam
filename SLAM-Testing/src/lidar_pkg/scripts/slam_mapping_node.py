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
        # NOTE: 0.01m (1cm) cells make the grid huge for any real room and
        # make every downstream consumer (esp. path_planner's A*) much
        # slower than it needs to be. 0.05m (5cm) is a much more reasonable
        # default for a small mobile robot; override via parameter if you
        # need finer resolution.
        self.declare_parameter('grid_resolution_m', 0.05)
        self._res_m = self.get_parameter('grid_resolution_m').value
        self._res_mm = self._res_m * 1000.0

        # --- Persistent map state ---
        # Instead of keeping every historical scan/pose and re-raycasting
        # the ENTIRE history on every new scan (which made this node get
        # slower and heavier the longer a session ran), we now keep only:
        #   - the current persistent occupancy grid (grown on demand)
        #   - the previous local scan (needed for frame-to-frame ICP)
        #   - the current accumulated global pose
        # and integrate only the NEWEST scan into the grid each time.
        self._grid = None                     # np.int8 array, created on first scan
        self._origin_mm = np.array([0.0, 0.0])  # world mm coords of grid[0,0]'s corner
        self._current_pose = np.identity(3)
        self._last_local_scan = None

        # Subscribers and Publishers
        self._scan_sub = self.create_subscription(LidarSweep, '/lidar_scan_node/scan', self._scan_callback, 10)

        # FIX: was '~/map' (-> /slam_mapping_node/map), which path_planner_node
        # never subscribes to. path_planner_node listens on plain 'map', so we
        # publish there directly.
        self._map_pub = self.create_publisher(OccupancyGrid, 'map', 10)

        self.get_logger().info("SLAM Mapping Node online. Processing map sweeps dynamically...")

    def _scan_callback(self, msg: LidarSweep):
        if len(msg.distances) == 0:
            return

        try:
            current_local = polar_to_cartesian(np.array(msg.angles), np.array(msg.distances))

            if self._last_local_scan is not None:
                try:
                    T_rel = icp(current_local, self._last_local_scan)
                    self._current_pose = self._current_pose @ T_rel
                except Exception as e:
                    # FIX: original code had no error handling around icp() here
                    # (vector_deducer_node did, this one didn't). A degenerate
                    # scan (too few points, no geometric structure) could crash
                    # the whole node. We now log and skip integrating this scan
                    # into the map rather than taking the node down.
                    self.get_logger().error(
                        f"ICP failed on this scan, skipping map integration: {e}"
                    )
                    self._last_local_scan = current_local
                    return

            self._last_local_scan = current_local
            self._integrate_scan(current_local, self._current_pose, msg.header.frame_id)

        except Exception as e:
            self.get_logger().error(f"Unexpected error while processing scan: {e}")

    # ------------------------------------------------------------------
    # Incremental map integration
    # ------------------------------------------------------------------

    def _integrate_scan(self, local_pts, pose, frame_id):
        homo_pts = np.ones((local_pts.shape[0], 3))
        homo_pts[:, :2] = local_pts
        global_pts = (pose @ homo_pts.T).T[:, :2]
        robot_pos = pose[:2, 2]

        pad_mm = 500.0
        pts_and_robot = np.vstack([global_pts, robot_pos.reshape(1, 2)])
        min_needed = pts_and_robot.min(axis=0) - pad_mm
        max_needed = pts_and_robot.max(axis=0) + pad_mm

        self._ensure_grid_covers(min_needed, max_needed)

        height, width = self._grid.shape

        def to_grid(pt):
            gx = int(np.floor((pt[0] - self._origin_mm[0]) / self._res_mm))
            gy = int(np.floor((pt[1] - self._origin_mm[1]) / self._res_mm))
            return gx, gy

        gx0, gy0 = to_grid(robot_pos)

        for pt in global_pts:
            gx1, gy1 = to_grid(pt)

            if not (0 <= gx0 < width and 0 <= gy0 < height and 0 <= gx1 < width and 0 <= gy1 < height):
                continue

            for fx, fy in bresenham_line(gx0, gy0, gx1, gy1):
                if self._grid[fy, fx] != 100:
                    self._grid[fy, fx] = 0  # Free space

            self._grid[gy1, gx1] = 100  # Obstacle

        self._publish_map(frame_id)

    def _ensure_grid_covers(self, min_needed, max_needed):
        """Grows the persistent grid (if necessary) to cover the requested
        world-space bounding box, preserving existing data."""

        if self._grid is None:
            width = max(int(np.ceil((max_needed[0] - min_needed[0]) / self._res_mm)), 1)
            height = max(int(np.ceil((max_needed[1] - min_needed[1]) / self._res_mm)), 1)
            self._grid = np.full((height, width), -1, dtype=np.int8)
            self._origin_mm = np.array(min_needed, dtype=np.float64)
            return

        height, width = self._grid.shape
        cur_min = self._origin_mm
        cur_max = self._origin_mm + np.array([width, height]) * self._res_mm

        new_min = np.minimum(cur_min, min_needed)
        new_max = np.maximum(cur_max, max_needed)

        if np.allclose(new_min, cur_min) and np.allclose(new_max, cur_max):
            return  # existing grid already covers what we need

        new_width = max(int(np.ceil((new_max[0] - new_min[0]) / self._res_mm)), 1)
        new_height = max(int(np.ceil((new_max[1] - new_min[1]) / self._res_mm)), 1)
        new_grid = np.full((new_height, new_width), -1, dtype=np.int8)

        offset_x = int(round((cur_min[0] - new_min[0]) / self._res_mm))
        offset_y = int(round((cur_min[1] - new_min[1]) / self._res_mm))
        new_grid[offset_y:offset_y + height, offset_x:offset_x + width] = self._grid

        self._grid = new_grid
        self._origin_mm = new_min

    def _publish_map(self, frame_id):
        height, width = self._grid.shape

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = frame_id

        map_msg.info.resolution = self._res_m
        map_msg.info.width = width
        map_msg.info.height = height

        origin_pose = Pose()
        origin_pose.position.x = float(self._origin_mm[0] / 1000.0)
        origin_pose.position.y = float(self._origin_mm[1] / 1000.0)
        origin_pose.position.z = 0.0
        map_msg.info.origin = origin_pose

        map_msg.data = self._grid.ravel().tolist()

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