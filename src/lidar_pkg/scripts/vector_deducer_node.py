#!/usr/bin/env python3
import numpy as np
from sklearn.neighbors import NearestNeighbors

import rclpy
from rclpy.node import Node
from lidar_msgs.msg import LidarSweep
from geometry_msgs.msg import Pose2D

# ----------------------------------------------------------------------
# ICP Core Math Functions (Kept identically from your source script)
# ----------------------------------------------------------------------
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

# ----------------------------------------------------------------------
# ROS 2 Wrapper Node
# ----------------------------------------------------------------------
class VectorDeducerNode(Node):

    def __init__(self):
        super().__init__('vector_deducer_node')
        
        self.declare_parameter('reject_ratio', 0.1)
        self._reject_ratio = self.get_parameter('reject_ratio').value

        # Memory buffer to hold onto the previous scan frame
        self._prev_scan_xy = None

        # Subscriber: Listens to the LiDAR reader node output
        self._scan_sub = self.create_subscription(
            LidarSweep,
            '/lidar_scan_node/scan',
            self._scan_callback,
            10
        )

        # Publisher: Emits calculated [dx, dy, dtheta] vector modifications
        self._vector_pub = self.create_publisher(Pose2D, '~/relative_vector', 10)
        
        self.get_logger().info("Vector Deducer Node operational. Waiting for streams...")

    def _scan_callback(self, msg: LidarSweep):
        # 1. Convert incoming message arrays into structural numpy arrays
        angles = np.array(msg.angles)
        distances = np.array(msg.distances)

        if len(distances) == 0:
            self.get_logger().warn("Empty scan frame bypassed.")
            return

        # 2. Convert raw polar coordinates straight to 2D Cartesian coordinates (X, Y)
        angles_rad = np.deg2rad(angles)
        x = distances * np.cos(angles_rad)
        y = distances * np.sin(angles_rad)
        current_scan_xy = np.column_stack([x, y])

        # 3. If there is no baseline frame historical data yet, save this one and wait
        if self._prev_scan_xy is None:
            self._prev_scan_xy = current_scan_xy
            self.get_logger().info("Cached initial scan baseline reference frame.")
            return

        # 4. Execute ICP mapping current scan frame (A) onto previous reference frame (B)
        try:
            T = icp(current_scan_xy, self._prev_scan_xy, reject_ratio=self._reject_ratio)
            
            # Extract displacement transformations
            dx = T[0, 2]
            dy = T[1, 2]
            dtheta = np.arctan2(T[1, 0], T[0, 0])

            # 5. Populate and Emit Vector Message Structure
            output_msg = Pose2D()
            output_msg.x = float(dx)
            output_msg.y = float(dy)
            output_msg.theta = float(dtheta)
            self._vector_pub.publish(output_msg)

            self.get_logger().info(
                f"ICP Match -> dx: {dx:.1f}mm | dy: {dy:.1f}mm | dtheta: {np.rad2deg(dtheta):.2f}°"
            )

        except Exception as e:
            self.get_logger().error(f"ICP Processing fault: {e}")

        # 6. Cycle memory frames forward
        self._prev_scan_xy = current_scan_xy


def main(args=None):
    rclpy.init(args=args)
    node = VectorDeducerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
