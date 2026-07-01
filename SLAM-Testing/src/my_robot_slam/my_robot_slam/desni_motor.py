#!/usr/bin/env python3
#ne koristi se vise
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class DesniMotorNode(Node):
    def __init__(self):
        super().__init__("desni_motor")
        self.get_logger().info("Desni motor je pokrenut i spreman.")

        self.pin = 4  # Pin za desni motor
        self.komande_sub = self.create_subscription(
            Int32MultiArray,
            'ulazne_komande_desni',
            self.komanda_callback,
            10)

        self.podaci_motora_pub = self.create_publisher(Int32MultiArray, 'podaci_desnog_motora', 10)

    def komanda_callback(self, msg):
        if len(msg.data) >= 2:
            naredba = 70 #ovo sam dodao kao ASCII vrednost za 'F', sto je fja koju prihvata arduino
            brzina = msg.data[0]
            duzina = msg.data[1]
            
            self.get_logger().info(f"Primljeno -> Naredba: {naredba}, Brzina: {brzina}, Dužina: {duzina} na pinu: {self.pin}")
            izlazna_poruka = Int32MultiArray()
            izlazna_poruka.data = [self.pin, naredba, brzina, duzina]
            
            self.podaci_motora_pub.publish(izlazna_poruka)
            self.get_logger().info("Podaci desnog motora uspešno prosleđeni!")
        else:
            self.get_logger().warn("Poruka za desni motor nema dovoljno podataka!")

def main(args=None):
    rclpy.init(args=args)
    node = DesniMotorNode()
    node.get_logger().info("Desni motor je uspesno pokrenut!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()