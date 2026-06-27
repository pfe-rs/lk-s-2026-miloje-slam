import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan # LASER SCAN JE ROS PAKET MORA SE KAO TAKAV I KORISTITI
import math

class MockLidar(Node):
    def __init__(self):
        super().__init__('Test cvor za lidar')
        
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10) # 10 - velicina reda, koliko poruka moze u bafer dok se ceka

        # Pablish na 5Hz
        self.timer = self.create_timer(0.2, self.publish_scan)
        self.get_logger().info("Lidar pokrenut, objavljuje na /scan")
    
    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Specifikacije RPLidar A1 senzora
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = math.radians(1.0) # 1 stepen u radijanima
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 12.0

        # Fejk podaci, kruzna soba, r = 2m
        num_readings = 360
        msg.ranges = []
        for i in range(num_readings):
            msg.ranges.append(2.0)
        self.publisher_.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = MockLidar()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()