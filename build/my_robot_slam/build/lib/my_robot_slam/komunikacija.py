#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import time

class KomunikacijaArduinoNode(Node):
    def __init__(self):
        super().__init__("komunikacija_arduino")
        self.get_logger().info("Čvor za komunikaciju sa Arduinom je pokrenut.")

        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)         # Sluša oba motora
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
        if len(msg.data) >= 3:
            naredba = msg.data[0]
            brzina = msg.data[1]
            duzina = msg.data[2]
            
            # Pozivamo funkciju za slanje
            odgovor = self.write_read(naredba, brzina, duzina)
            self.get_logger().info(f"Arduino odgovorio: {odgovor}")

    def desni_callback(self, msg):
        self.get_logger().info(f"Most primio DESNI motor: {msg.data} -> Spreman za UART/Arduino.")
        if len(msg.data) >= 3:
            naredba = msg.data[0]
            brzina = msg.data[1]
            duzina = msg.data[2]
            
            # Pozivamo funkciju za slanje
            odgovor = self.write_read(naredba, brzina, duzina)
            self.get_logger().info(f"Arduino odgovorio: {odgovor}")

    def write_read(self, naredba, brzina, duzina): 
        poruka = f"{naredba} {brzina} {duzina}\n"
        self.arduino.write(bytes(poruka, 'utf-8')) 
        time.sleep(0.05) 
        data = self.arduino.readline() 
        return data


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