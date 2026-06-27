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
        self.motor_sub = self.create_subscription(
            Int32MultiArray,
            'podaci_motora',
            self.callback,
            10)

    def callback(self, msg):
        pin = msg.data[0]
        command = msg.data[1]
        speed = msg.data[2]
        steps = msg.data[3]

        odgovor = self.write_read(pin, command, speed, steps)
        self.get_logger().info(f"Arduino odgovorio: {odgovor}")

    def write_read(self, pin, command, speed, steps):
        poruka = f"{pin} {command} {speed} {steps}\n"
        self.arduino.write(poruka.encode('utf-8'))
        time.sleep(0.01)
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