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

        # Otvaranje serijske veze (timeout je stavljen na vrlo kratko da ne blokira tajmer)
        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=0.01)         
        
        # Subscriber za komande motora
        self.motor_sub = self.create_subscription(
            Int32MultiArray,
            '/podaci_motion_planner',
            self.callback,
            10)
        
        # Publisher za enkodere
        self.pub_encoder = self.create_publisher(Int32MultiArray, '/podaci_enkodera', 10)

    def callback(self, msg):
        naredba=7
        koraciL=msg.data[0]
        brzinaL=msg.data[1]
        koraciD=msg.data[2]
        brzinaD=msg.data[3]

        odgovor = self.write_read(naredba, koraciL, brzinaL, koraciD, brzinaD)
        self.get_logger().info(f"Arduino odgovorio na komandu: {odgovor}")

        izlazna_poruka = Int32MultiArray()
        izlazna_poruka.data = [koraciL, koraciD]
        self.pub_encoder.publish(izlazna_poruka)

    def write_read(self, naredba, koraciL, brzinaL, koraciD, brzinaD):
        poruka = f"{naredba} {koraciL} {brzinaL} {koraciD} {brzinaD}\n"
        self.arduino.write(poruka.encode('utf-8'))
        time.sleep(0.01) # Kratka pauza da Arduino obradi i odgovori
        odgovor = self.arduino.readline().decode('utf-8').strip()
        return odgovor

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