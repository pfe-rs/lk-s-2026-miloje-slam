#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class OdometrijaNode(Node):
    def __init__(self):
        super().__init__('odometrija_node')

        # --- KONSTANTE ROBOTA ---
        self.distancePerCount = 0.00025133  # m/tick (za točak d=8cm i 1000 PPR)
        self.wheelDistance = 0.2            # razmak točkova u metrima

        # Stanje odometrije (Pozicija i orijentacija robota)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Prethodna stanja za proračun diferencijala
        self.encoderRPosPrev = 0
        self.encoderLPosPrev = 0
        self.prvi_prolaz = True
        self.prev_time = self.get_clock().now()

        # --- ROS 2 SUBSCRIBER ---
        # Slušamo topik na koji komunikacija šalje sirove vrednosti enkodera [levo, desno]
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            'podaci_enkodera',
            self.odometrija_callback,
            10)

        # --- ROS 2 PUBLISHERI ---
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Čvor za računanje odometrije preko ROS poruka je pokrenut!")

    def odometrija_callback(self, msg):
        try:
            # Provera da li poruka sadrži bar dva podatka (levi i desni enkoder)
            if len(msg.data) < 2:
                self.get_logger().warn("Primljena nevalidna poruka enkodera (manje od 2 elementa).")
                return

            # Pretvaramo primljene podatke iz poruke
            encoderLPos = int(msg.data[0])
            encoderRPos = int(msg.data[1])

            # Proračun vremena (dt) proteklog između dva čitanja
            trenutno_vreme = self.get_clock().now()
            dt = (trenutno_vreme - self.prev_time).nanoseconds / 1e9
            self.prev_time = trenutno_vreme
            if dt <= 0.0:
                dt = 1e-6

            # Prvi prolaz služi samo da zabeležimo početne pozicije enkodera
            if self.prvi_prolaz:
                self.encoderLPosPrev = encoderLPos
                self.encoderRPosPrev = encoderRPos
                self.prvi_prolaz = False
                self.get_logger().info("Početne pozicije enkodera uspešno zabeležene.")
                return

            # Matematika diferencijalne odometrije (pređeni put točkova)
            SR = self.distancePerCount * (encoderRPos - self.encoderRPosPrev)
            SL = self.distancePerCount * (encoderLPos - self.encoderLPosPrev)

            # Čuvamo trenutne pozicije kao prethodne za sledeći krug
            self.encoderRPosPrev = encoderRPos
            self.encoderLPosPrev = encoderLPos

            # Prosečni pređeni put i promena ugla
            S = (SR + SL) / 2.0
            delta_theta = (SR - SL) / self.wheelDistance

            # Integracija kretanja u globalni koordinatni sistem
            if abs(delta_theta) < 1e-6:
                self.x += S * math.cos(self.theta)
                self.y += S * math.sin(self.theta)
            else:
                R = S / delta_theta
                self.x += R * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
                self.y -= R * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

            self.theta += delta_theta
            # Normalizacija ugla na opseg od -pi do pi
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # Linearne i angularne brzine robota
            v = S / dt
            omega = delta_theta / dt

            # Pretvaranje ugla (Yaw) u kvaternion potreban za ROS standarde
            q_z = math.sin(self.theta / 2.0)
            q_w = math.cos(self.theta / 2.0)

            # 1. Slanje TF-a (Transformacije odom -> base_link)
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

            # 2. Slanje pune Odometry poruke na /odom topik
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
            self.get_logger().error(f"Greška u formatu podataka unutar poruke: {e}")
        except Exception as e:
            self.get_logger().error(f"Neočekivana greška u obradi odometrije: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = OdometrijaNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()