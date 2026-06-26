#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class KomunikacijaArduinoNode(Node):
    def __init__(self):
        super().__init__("komunikacija_arduino")
        self.get_logger().info("Čvor za komunikaciju sa Arduinom je pokrenut.")

        # Sluša oba motora
        self.levi_sub = self.create_subscription(
            Int32MultiArray,
            'podaci_levog_motora',
            self.levi_callback,
            10)
            
        self.desni_sub = self.create_subscription(
            Int32MultiArray,
            'podaci_desnog_motora',
            self.desni_callback,
            10)

    def levi_callback(self, msg):
        self.get_logger().info(f"Most primio LEVI motor: {msg.data} -> Spreman za UART/Arduino.")
        # Ovde dolazi tvoj kôd za slanje preko serijskog porta (npr. import serial)

    def desni_callback(self, msg):
        self.get_logger().info(f"Most primio DESNI motor: {msg.data} -> Spreman za UART/Arduino.")

def main(args=None):
    rclpy.init(args=args)
    node = KomunikacijaArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()