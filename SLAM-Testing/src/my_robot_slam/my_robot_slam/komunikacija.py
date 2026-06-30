#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class KomunikacijaArduinoNode(Node):
    def __init__(self):
        super().__init__("komunikacija_arduino")
        self.get_logger().info("Čvor za komunikaciju sa Arduinom je pokrenut.")

        # Otvaranje serijske veze preko ttyS0 porta
        self.arduino = serial.Serial(port='/dev/ttyS0', baudrate=9600, timeout=0.01)
        
        # Poništavamo eventualno smeće u baferu pre početka rada
        self.arduino.reset_input_buffer()
        self.arduino.reset_output_buffer()

        # Subscriber za komande motora
        self.motor_sub = self.create_subscription(
            Int32MultiArray,
            '/podaci_motion_planner',
            self.callback,
            10)
        
        # Publisher za enkodere
        self.pub_encoder = self.create_publisher(Int32MultiArray, '/podaci_enkodera', 10)

    def callback(self, msg):
        # Provera da li niz sadrži dovoljno elemenata da spreči IndexError
        if len(msg.data) < 4:
            self.get_logger().error("Primljeni podaci na topiku nemaju dovoljno elemenata!")
            return

        naredba = 7
        koraciL = msg.data[0]
        brzinaL = msg.data[1]
        koraciD = msg.data[2]
        brzinaD = msg.data[3]

        # POPRAVLJENO: Dodato self. ispred poziva funkcije
        self.write_to_arduino(naredba, koraciL, brzinaL, koraciD, brzinaD)

        # Slanje povratne informacije na topik za enkodere
        izlazna_poruka = Int32MultiArray()
        izlazna_poruka.data = [koraciL, koraciD]
        self.pub_encoder.publish(izlazna_poruka)

    def write_to_arduino(self, naredba, koraciL, brzinaL, koraciD, brzinaD):
        poruka = f"{naredba} {koraciL} {brzinaL} {koraciD} {brzinaD}\n"
        self.arduino.write(poruka.encode('utf-8'))
        
        # Ako Arduino ipak šalje potvrdu, pročitaj je i baci je da ne puni bafer, 
        # ili otkomentariši print ispod ako želiš da vidiš šta kaže
        if self.arduino.in_waiting > 0:
            _ = self.arduino.readline() 
            # odgovor = _.decode('utf-8').strip()
            # self.get_logger().info(f"Arduino: {odgovor}")

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
