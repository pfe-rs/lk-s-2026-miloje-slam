#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class FakeEncoder(Node):
    def __init__(self):
        super().__init__('fake_encoder')
        self.pub = self.create_publisher(Int32MultiArray, '/arduino_encoders', 10)

        self.left = 0
        self.right = 0

        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

        self.get_logger().info("Fake encoder pokrenut")

    def tick(self):
        msg = Int32MultiArray()

        # simulacija: oba točka idu napred
        self.left += 10
        self.right += 20

        msg.data = [self.left, self.right]

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeEncoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()