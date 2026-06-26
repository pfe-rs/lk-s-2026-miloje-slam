#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class LeviMotorNode(Node):
    def __init__(self):
        super().__init__("levi_motor")
        self.get_logger().info("Levi motor je pokrenut i spreman.")

        self.pin = 4
        self.komande_sub = self.create_subscription(
            Int32MultiArray,
            'ulazne_komande',
            self.komanda_callback,
            10)

        self.podaci_motora_pub = self.create_publisher(Int32MultiArray, 'podaci_levog_motora', 10)

    def komanda_callback(self, msg):
        if len(msg.data) >= 3:
            naredba = msg.data[0]
            brzina = msg.data[1]
            duzina = msg.data[2]
            
            self.get_logger().info(f"Primljeno -> Naredba: {naredba}, Brzina: {brzina}, Dužina: {duzina} na pinu: {self.pin}")
            izlazna_poruka = Int32MultiArray()
            izlazna_poruka.data = [naredba, brzina, duzina]
            
            self.podaci_motora_pub.publish(izlazna_poruka)
            self.get_logger().info("Podaci uspešno prosleđeni sledećem čvoru!")
        else:
            self.get_logger().warn("Primljena poruka nema dovoljno podataka (očekuju se 3 vrednosti)!")

def main(args=None):
    rclpy.init(args=args)
    node = LeviMotorNode()
    node.get_logger().info("Levi motor je uspesno pokrenut!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()