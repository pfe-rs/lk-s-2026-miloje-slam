#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
import serial
import time

class KomunikacijaArduinoNode(Node):
    def __init__(self):
        super().__init__("komunikacija_arduino")
        self.get_logger().info("Čvor za komunikaciju sa Arduinom je pokrenut.")

        # Otvaranje serijske veze preko ttyS0 porta
        self.arduino = serial.Serial(port='/dev/ttyS0', baudrate=9600, timeout=0.01)
        self.arduino.reset_input_buffer()
        self.arduino.reset_output_buffer()

        # Koristimo ReentrantCallbackGroup kako bi više callback-a moglo da radi paralelno
        self.cb_group = ReentrantCallbackGroup()

        # Subscriberi sa dodeljenom callback grupom
        self.motor_sub = self.create_subscription(
            Int32MultiArray, '/podaci_kretanje', self.callback, 10, callback_group=self.cb_group)
        self.skretanje_sub = self.create_subscription(
            Int32MultiArray, '/podaci_skretanje', self.callback, 10, callback_group=self.cb_group)

        # Kreiranje klijenta za LiDAR servis
        # Pošto lidar čvor koristi '~/scan_request', pun naziv servisa je obično '/lidar_scan_node/scan_request'
        self.lidar_client = self.create_client(Trigger, '/lidar_scan_node/scan_request', callback_group=self.cb_group)
        
        # Čekamo da servis postane dostupan na mreži
        while not self.lidar_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Čeka se LiDAR servis da postane dostupan...')

    def callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().error("Primljeni podaci na topiku nemaju dovoljno elemenata!")
            return

        naredba = 7
        koraciL = msg.data[0]
        brzinaL = msg.data[1]
        koraciD = msg.data[2]
        brzinaD = msg.data[3]
        
        # Izbegavanje deljenja sa nulom u slučaju pogrešnih podataka
        if brzinaL == 0:
            brzinaL = 1
            
        vreme = abs(koraciL / brzinaL)

        # Slanje komande i čekanje (Sada bezbedno jer koristimo MultiThreadedExecutor)
        self.write_to_arduino(naredba, koraciL, brzinaL, koraciD, brzinaD, vreme)

        # ---- POZIV LIDAR SERVISA NAKON ŠTO JE ROBOT STAO ----
        self.get_logger().info("Robot se zaustavio. Šaljem zahtev za LiDAR skeniranje...")
        self.pozovi_lidar_servis()

    def write_to_arduino(self, naredba, koraciL, brzinaL, koraciD, brzinaD, vreme):
        poruka = f"{naredba} {koraciL} {brzinaL} {koraciD} {brzinaD} \n"
        self.arduino.write(poruka.encode('utf-8'))
        
        if self.arduino.in_waiting > 0:
            _ = self.arduino.readline() 

        # POPRAVLJENO: Zamenjena zapeta sa tačkom (1.15)
        time.sleep(vreme * 1.15)  
        self.get_logger().info("Izvršavanje kretanja završeno (tajmer istekao).")

    def pozovi_lidar_servis(self):
        req = Trigger.Request()
        # Pozivamo servis sinhrono unutar ove niti (ne blokira druge niti)
        future = self.lidar_client.call_async(req)
        
        # Pošto smo u MultiThreadedExecutor-u, možemo bezbedno sačekati odgovor
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            
        res = future.result()
        if res is not None:
            self.get_logger().info(f"LiDAR Odgovor: Uspeh={res.success}, Poruka={res.message}")
        else:
            self.get_logger().error("Nije uspelu pozivanje LiDAR servisa.")

def main(args=None):
    rclpy.init(args=args)
    node = KomunikacijaArduinoNode()
    
    # POPRAVLJENO: Korišćenje MultiThreadedExecutor-a da sprečimo zamrzavanje čvora tokom time.sleep()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()