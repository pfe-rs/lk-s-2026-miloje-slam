#!/usr/bin/env python3
import numpy as np
from sklearn.neighbors import NearestNeighbors

import rclpy
from rclpy.node import Node
from lidar_msgs.msg import LidarSweep
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

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

        # FIX: this node computes frame-to-frame motion via ICP but never
        # used to accumulate it into an absolute pose, and nothing consumed
        # its output. path_planner_node subscribes to 'odom' expecting
        # nav_msgs/Odometry -- without this, start_x/start_y in path_planner
        # never move from (0, 0), so A* keeps planning from a stale
        # position as the robot explores. This accumulated pose (mm,
        # internally, same convention as slam_mapping_node) is the running
        # global pose estimate.
        self._global_pose = np.identity(3)

        self._scan_sub = self.create_subscription(
            LidarSweep,
            '/lidar_scan_node/scan',
            self._scan_callback,
            10
        )

        self._vector_pub = self.create_publisher(Pose2D, '~/relative_vector', 10)

        # FIX: publishes the accumulated absolute pose as nav_msgs/Odometry
        # on the plain 'odom' topic, which is exactly what
        # path_planner_node.pose_callback expects.
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.get_logger().info("Vector Deducer Node operational. Waiting for streams...")

    def _scan_callback(self, msg: LidarSweep):
        angles = np.array(msg.angles)
        distances = np.array(msg.distances)

        if len(distances) == 0:
            self.get_logger().warn("Empty scan frame bypassed.")
            return

        angles_rad = np.deg2rad(angles)
        x = distances * np.cos(angles_rad)
        y = distances * np.sin(angles_rad)
        current_scan_xy = np.column_stack([x, y])

        if self._prev_scan_xy is None:
            self._prev_scan_xy = current_scan_xy
            self.get_logger().info("Cached initial scan baseline reference frame.")
            # Publish the starting pose immediately so path_planner_node has
            # a valid 'odom' message to work with even before the second
            # scan (and thus the first ICP match) happens.
            self._publish_odom(msg.header.stamp)
            return

        try:
            T = icp(current_scan_xy, self._prev_scan_xy, reject_ratio=self._reject_ratio)

            dx = T[0, 2]
            dy = T[1, 2]
            dtheta = np.arctan2(T[1, 0], T[0, 0])

            output_msg = Pose2D()
            output_msg.x = float(dx)
            output_msg.y = float(dy)
            output_msg.theta = float(dtheta)
            self._vector_pub.publish(output_msg)

            self._global_pose = self._global_pose @ T
            self._publish_odom(msg.header.stamp)

            self.get_logger().info(
                f"ICP Match -> dx: {dx:.1f}mm | dy: {dy:.1f}mm | dtheta: {np.rad2deg(dtheta):.2f}°"
            )

        except Exception as e:
            self.get_logger().error(f"ICP Processing fault: {e}")

        self._prev_scan_xy = current_scan_xy

    def _publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Internal pose math is in millimeters (matches slam_mapping_node's
        # convention); nav_msgs/Odometry and path_planner_node expect meters.
        odom.pose.pose.position.x = float(self._global_pose[0, 2] / 1000.0)
        odom.pose.pose.position.y = float(self._global_pose[1, 2] / 1000.0)
        odom.pose.pose.position.z = 0.0

        yaw = float(np.arctan2(self._global_pose[1, 0], self._global_pose[0, 0]))
        odom.pose.pose.orientation.z = float(np.sin(yaw / 2.0))
        odom.pose.pose.orientation.w = float(np.cos(yaw / 2.0))

        self._odom_pub.publish(odom)


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