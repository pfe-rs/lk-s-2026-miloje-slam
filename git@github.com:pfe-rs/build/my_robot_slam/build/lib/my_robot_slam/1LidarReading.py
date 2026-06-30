#!/usr/bin/env python3
"""
ROS 2 node wrapping an RPLidar A1.

Behavior:
    - Connects to the lidar once at startup and lets the motor spin up.
    - Stays idle (not collecting) until another node calls the
      '~/scan_request' service (std_srvs/Trigger).
    - On trigger, collects exactly one full 360-degree rotation
      (using the same new_scan boundary-flag logic as the original
      script), converts it to a sensor_msgs/LaserScan, and publishes
      it on '~/scan' for downstream consumers (e.g. a vector_deducer
      node).
    - Returns to idle afterward, waiting for the next trigger.

The actual blocking lidar I/O runs in a background thread so the
ROS executor is never blocked while waiting for hardware.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from rplidar import RPLidar


class LidarScanNode(Node):

    def __init__(self):
        super().__init__('lidar_scan_node')

        # ---- Parameters ----
        self.declare_parameter('port_name', '/dev/ttyUSB1')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('motor_stabilize_sec', 2.0)
        self.declare_parameter('range_min_m', 0.15)   # RPLidar A1 spec floor
        self.declare_parameter('range_max_m', 12.0)   # RPLidar A1 spec ceiling

        self._port_name = self.get_parameter('port_name').value
        self._frame_id = self.get_parameter('frame_id').value
        self._stabilize_sec = self.get_parameter('motor_stabilize_sec').value
        self._range_min_m = self.get_parameter('range_min_m').value
        self._range_max_m = self.get_parameter('range_max_m').value

        # ---- Lidar connection (once, at startup) ----
        self._lidar = None
        self._lidar_lock = threading.Lock()  # guards access to self._lidar
        self._connect_lidar()

        # ---- Publisher for completed scans ----
        self._scan_pub = self.create_publisher(LaserScan, '~/scan', 10)

        # ---- Service: other nodes call this to request one scan ----
        # ReentrantCallbackGroup + MultiThreadedExecutor so the service
        # call can be in-flight while we still process other callbacks
        # if needed later (lidar work itself happens in its own thread).
        self._scan_cb_group = ReentrantCallbackGroup()
        self._scan_service = self.create_service(
            Trigger,
            '~/scan_request',
            self._handle_scan_request,
            callback_group=self._scan_cb_group,
        )

        # Prevents two overlapping scan requests from both touching
        # the lidar hardware at once.
        self._scan_in_progress = threading.Lock()

        self.get_logger().info(
            f"Lidar ready on {self._port_name}. "
            f"Waiting for calls to 'scan_request' service."
        )

    # ------------------------------------------------------------------
    # Setup / teardown
    # ------------------------------------------------------------------

    def _connect_lidar(self):
        self._lidar = RPLidar(self._port_name)
        self.get_logger().info(f'RPLidar Information: {self._lidar.get_info()}')
        self.get_logger().info(f'Health status: {self._lidar.get_health()}')
        self.get_logger().info('Waiting for motor to stabilize...')
        time.sleep(self._stabilize_sec)

    def destroy_node(self):
        if self._lidar is not None:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
                self.get_logger().info('Lidar disconnected safely.')
            except Exception as e:
                self.get_logger().warn(f'Error while disconnecting lidar: {e}')
        super().destroy_node()

    # ------------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------------

    def _handle_scan_request(self, request, response):
        """Called by the triggering node. Blocks until one full
        rotation has been collected and published."""

        if not self._scan_in_progress.acquire(blocking=False):
            response.success = False
            response.message = 'A scan is already in progress.'
            return response

        try:
            points = self._collect_one_rotation()
        except Exception as e:
            self.get_logger().error(f'Scan failed: {e}')
            response.success = False
            response.message = f'Scan failed: {e}'
            return response
        finally:
            self._scan_in_progress.release()

        if not points:
            response.success = False
            response.message = 'No valid points captured during scan.'
            return response

        self._publish_scan(points)
        response.success = True
        response.message = f'Scan complete: {len(points)} points published.'
        return response

    # ------------------------------------------------------------------
    # Core scan logic (ported from the original script)
    # ------------------------------------------------------------------

    def _collect_one_rotation(self):
        """Collects exactly one full 360-degree rotation of
        (angle_deg, distance_mm) points, sorted by angle."""

        raw_points = []
        start_collecting = False

        self.get_logger().info('Searching for the start-of-scan flag...')

        with self._lidar_lock:
            for new_scan, _, angle, distance in self._lidar.iter_measures():
                if new_scan:
                    if not start_collecting:
                        start_collecting = True
                        self.get_logger().info(
                            'Start flag found! Collecting one full rotation...'
                        )
                        if distance > 0:
                            raw_points.append((angle, distance))
                        continue
                    else:
                        self.get_logger().info(
                            'End flag reached. Rotation complete.'
                        )
                        break

                if start_collecting and distance > 0:
                    raw_points.append((angle, distance))

        raw_points.sort(key=lambda p: p[0])
        return raw_points

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_scan(self, points):
        """Converts (angle_deg, distance_mm) points into a
        sensor_msgs/LaserScan and publishes it.

        LaserScan is the ROS 2 standard for 2D lidar data and is what
        tools like rviz2, slam_toolbox, and nav2 expect. It requires
        a fixed angle_increment between samples, in radians, with
        ranges in meters (REP 103), so raw angle/distance pairs are
        resampled onto an evenly-spaced array here. The original
        mm/degree semantics are preserved in the conversion -- only
        the on-the-wire units change, not the underlying data.
        """

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        # 1-degree resolution; adjust if you need finer granularity
        num_samples = 360
        msg.angle_increment = (msg.angle_max - msg.angle_min) / num_samples

        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = float(self._range_min_m)
        msg.range_max = float(self._range_max_m)

        # Bucket raw points into the nearest 1-degree bin. If multiple
        # points land in the same bin, keep the closest (helps reject
        # noisy outliers without dropping real geometry).
        ranges = [float('inf')] * num_samples
        for angle_deg, distance_mm in points:
            bin_idx = int(round(angle_deg)) % num_samples
            distance_m = distance_mm / 1000.0
            if distance_m < ranges[bin_idx]:
                ranges[bin_idx] = distance_m

        msg.ranges = ranges
        msg.intensities = []  # not tracked by this lidar/script

        self._scan_pub.publish(msg)
        self.get_logger().info(f'Published LaserScan with {len(points)} raw points.')


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
