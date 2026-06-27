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
            'podaci_motora',
            self.callback,
            10)
        
        # Publisher za enkodere
        self.pub_encoder = self.create_publisher(Int32MultiArray, '/podaci_enkodera', 10)

        # TAJMER: Na svakih 0.05 sekundi (20Hz) proverava serijski port i kupi enkodere
        self.tajmer_enkodera = self.create_timer(0.05, self.citaj_i_publikuj_enkodere)

    def callback(self, msg):
        pin = msg.data[0]
        command = msg.data[1]
        speed = msg.data[2]
        steps = msg.data[3]

        odgovor = self.write_read(pin, command, speed, steps)
        self.get_logger().info(f"Arduino odgovorio na komandu: {odgovor}")

    def write_read(self, pin, command, speed, steps):
        poruka = f"{pin} {command} {speed} {steps}\n"
        self.arduino.write(poruka.encode('utf-8'))
        time.sleep(0.01) # Kratka pauza da Arduino obradi i odgovori
        odgovor = self.arduino.readline().decode('utf-8').strip()
        return odgovor

    def citaj_i_publikuj_enkodere(self):
        # Proveravamo da li ima ičega u serijskom baferu
        if self.arduino.in_waiting > 0:
            try:
                # Čitamo liniju sa Arduina (očekuje se format npr. "1250,1310")
                linija = self.arduino.readline().decode('utf-8').strip()
                
                # Preskačemo prazne linije
                if not linija:
                    return
                
                # Ako Arduino vrati eho komande motora (koji sadrži razmake, a ne zareze), preskačemo ga
                if ',' not in linija:
                    return

                # Delimo string po zarezu na levi i desni enkoder
                delovi = linija.split(',')
                if len(delovi) >= 2:
                    levi_enkoder = int(delovi[0])
                    desni_enkoder = int(delovi[1])

                    # Pakujemo u Int32MultiArray poruku
                    msg = Int32MultiArray()
                    msg.data = [levi_enkoder, desni_enkoder]
                    
                    # Publikujemo na /podaci_enkodera
                    self.pub_encoder.publish(msg)
                    
            except (ValueError, IndexError):
                # Ignorišemo ako se desi delimično pročitana linija ili loš format bajtova
                pass
            except Exception as e:
                self.get_logger().error(f"Greška pri čitanju enkodera: {e}")


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