import os
import json
import threading
from datetime import datetime
import rclpy
from rclpy.node import Node

# Import the necessary message types based on your existing nodes
from nav_msgs.msg import OccupancyGrid, Odometry
# Replace 'custom_interfaces' with the actual package name where LidarSweep is defined
from custom_interfaces.msg import LidarSweep 


class RosDataLoggerNode(Node):

    def __init__(self):
        super().__init__('ros_data_logger_node')

        # ---- Parametri ----
        self.declare_parameter('log_dir', '~/ros2_logs')
        log_base_path = os.path.expanduser(self.get_parameter('log_dir').value)
        
        # Folder po svakoj sesiji
        session_time = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = os.path.join(log_base_path, f"session_{session_time}")
        os.makedirs(self._log_path, exist_ok=True)

        # Threading lock for safe file writing across multiple subscription callbacks
        self._file_lock = threading.Lock()

        # ---- Subscriptions ----
        
        # 1. Subscribe to Lidar Scans (Note the absolute topic path matching your mapping node)
        self._scan_sub = self.create_subscription(
            LidarSweep,
            '/lidar_scan_node/scan',
            self._scan_callback,
            10
        )

        # 2. Subscribe to Map updates
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10
        )

        # 3. Subscribe to Odometry updates
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

        self.get_logger().info(f"Logger initialized. Saving logs locally to: {self._log_path}")

    def _get_timestamp(self):
        """Helper to generate clean timestamps for logged entries."""
        return datetime.now().isoformat()

    def _write_log(self, filename, data_dict):
        """Thread-safe helper to append JSON data to local files."""
        filepath = os.path.join(self._log_path, filename)
        
        with self._file_lock:
            with open(filepath, 'a') as f:
                # Store as JSON lines (one valid JSON object per line)
                f.write(json.dumps(data_dict) + '\n')

    def _scan_callback(self, msg: LidarSweep):
        # Extract fields from your custom LidarSweep message. 
        # Adjust these keys to match your message's actual attribute names.
        log_data = {
            "timestamp": self._get_timestamp(),
            "header": {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id
            },
            # Example fields (replace with your actual LidarSweep attributes):
            # "ranges": list(msg.ranges), 
            # "angles": list(msg.angles)
        }
        self._write_log("lidar_scans.jsonl", log_data)

    def _map_callback(self, msg: OccupancyGrid):
        log_data = {
            "timestamp": self._get_timestamp(),
            "info": {
                "rezolucija": msg.info.resolution,
                "sirina": msg.info.width,
                "visina": msg.info.height,
                "origin": {
                    "pozicija": {"x": msg.info.origin.position.x, "y": msg.info.origin.position.y},
                    "orijentacija": {"z": msg.info.origin.orientation.z, "w": msg.info.origin.orientation.w}
                }
            },
            # Flattening data to a python list for storage
            "data": list(msg.data) 
        }
        self._write_log("maps.jsonl", log_data)

    def _odom_callback(self, msg: Odometry):
        log_data = {
            "timestamp": self._get_timestamp(),
            "pozicija": {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z
            },
            "orijentacija": {
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z,
                "w": msg.pose.pose.orientation.w
            },
            "okret": {
                "linearni": {"x": msg.twist.twist.linear.x, "y": msg.twist.twist.linear.y},
                "ugaoni": {"z": msg.twist.twist.angular.z}
            }
        }
        self._write_log("odometrija.jsonl", log_data)


def main(args=None):
    rclpy.init(args=args)
    node = RosDataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Loger ugasen...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()