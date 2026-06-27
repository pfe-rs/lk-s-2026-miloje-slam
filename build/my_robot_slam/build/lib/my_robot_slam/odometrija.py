#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import serial  # Biblioteka za komunikaciju sa Arduinom

class OdometrijaPrekoSeriala(Node):
    def __init__(self):
        super().__init__('odometrija_serial_node')

        # --- KONFIGURACIJA SERIJSKOG PORTA ---
        # Proveri preko Arduino IDE-a da li je tvoj port /dev/ttyACM0 ili /dev/ttyUSB0
        self.port_name = '/dev/ttyACM0'
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Uspešno povezan na Arduino preko porta: {self.port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Greška pri otvaranju porta {self.port_name}: {e}")
            self.get_logger().error("Proveri da li je Arduino povezan i da li imaš dozvole (sudo chmod 666 /dev/ttyACM0)")
            raise e

        # --- KONSTANTE ROBOTA ---
        self.distancePerCount = 0.00025133  # m/tick (za točak d=8cm i 1000 PPR)
        self.wheelDistance = 0.2            # razmak točkova u metrima

        # Stanje odometrije
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Prethodna stanja
        self.encoderRPosPrev = 0
        self.encoderLPosPrev = 0
        self.prvi_prolaz = True
        self.prev_time = self.get_clock().now()

        # ROS 2 Oglašivači
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Tajmer koji čita Serial i osvežava odometriju na 20Hz (svakih 0.05 sekundi)
        self.tajmer = self.create_timer(0.05, self.osvezi_odometriju_sa_seriala)
        self.get_logger().info("Čvor za serijsku odometriju je spreman!")

    def osvezi_odometriju_sa_seriala(self):
        # Ako ima podataka u serijskom baferu, čitamo ih
        if self.ser.in_waiting > 0:
            try:
                # Čitamo liniju sa Arduina (npr. b"1250,1310\r\n")
                linija = self.ser.readline().decode('utf-8').strip()
                
                # Preskačemo prazne linije ako se dese
                if not linija:
                    return

                # Delimo string na mestu zareza
                delovi = linija.split(',')
                if len(delovi) < 2:
                    return

                # Pretvaramo stringove u cele brojeve (enkoder pozicije)
                encoderLPos = int(delovi[0])
                encoderRPos = int(delovi[1])

                # Proračun vremena
                trenutno_vreme = self.get_clock().now()
                dt = (trenutno_vreme - self.prev_time).nanoseconds / 1e9
                self.prev_time = trenutno_vreme
                if dt <= 0.0:
                    dt = 1e-6

                # Prvi prolaz služi samo da zabeležimo početne pozicije
                if self.prvi_prolaz:
                    self.encoderLPosPrev = encoderLPos
                    self.encoderRPosPrev = encoderRPos
                    self.prvi_prolaz = False
                    self.get_logger().info("Početne pozicije enkodera zabeležene.")
                    return

                # Matematika odometrije
                SR = self.distancePerCount * (encoderRPos - self.encoderRPosPrev)
                SL = self.distancePerCount * (encoderLPos - self.encoderLPosPrev)

                self.encoderRPosPrev = encoderRPos
                self.encoderLPosPrev = encoderLPos

                S = (SR + SL) / 2.0
                delta_theta = (SR - SL) / self.wheelDistance

                # Integracija kretanja
                if abs(delta_theta) < 1e-6:
                    self.x += S * math.cos(self.theta)
                    self.y += S * math.sin(self.theta)
                else:
                    R = S / delta_theta
                    self.x += R * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
                    self.y -= R * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

                self.theta += delta_theta
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

                # Brzine
                v = S / dt
                omega = delta_theta / dt

                # Pretvaranje u kvaternion
                q_z = math.sin(self.theta / 2.0)
                q_w = math.cos(self.theta / 2.0)

                # 1. Slanje TF-a
                t = TransformStamped()
                t.header.stamp = trenutno_vreme.to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.z = q_z
                t.transform.rotation.w = q_w
                self.tf_broadcaster.sendTransform(t)

                # 2. Slanje Odometry poruke
                odom = Odometry()
                odom.header.stamp = trenutno_vreme.to_msg()
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.orientation.z = q_z
                odom.pose.pose.orientation.w = q_w
                odom.twist.twist.linear.x = v
                odom.twist.twist.angular.z = omega

                self.pub_odom.publish(odom)

            except (ValueError, IndexError) as e:
                # Ignorišemo delimično pročitane ili loše formatirane linije sa seriala
                pass
            except Exception as e:
                self.get_logger().error(f"Neočekivana greška u obradi: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = OdometrijaPrekoSeriala()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()