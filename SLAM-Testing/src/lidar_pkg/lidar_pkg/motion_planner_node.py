#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger  # Za pokretanje Lidar skena
import math
import time

# --- HARDVERSKE KONFIGURACIJE (Usklađeno sa novim čvorom) ---
BROJ_STEPOVA_PO_REVOLUCIJI = 400
POLUPRECNIK_TOCKA_MM = 20
MM_PER_STEP = (2 * POLUPRECNIK_TOCKA_MM * math.pi) / BROJ_STEPOVA_PO_REVOLUCIJI

STEPOVI_ZA_90_STEPENI = 275
BRZINA_L = 200
BRZINA_D = 200
MARGINA_SIGURNOSTI = 1.15  # +15% zbog ubrzanja/usporavanja


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_planner_node')
        self.get_logger().info("Miloje Motion Planner čvor je uspešno pokrenut.")
        
        # Publisher ka Arduinu (preko novog ROS2 interfejsa)
        self.kretanje_pub = self.create_publisher(
            Int32MultiArray, '/podaci_motion_planner', 10
        )

        # Pretplata na globalni put (od path_planner_node)
        self.path_sub = self.create_subscription(
            Path, 'global_path', self.path_callback, 10
        )
 
        # Klijent za pokretanje Lidar skena nakon završetka rute
        self.lidar_client = self.create_client(Trigger, '/lidar_scan_node/scan_request')
        
        # Prati robota (Početne koordinate u metrima u ROS-u)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0  # RADIJANI

    def izracunaj_vreme_cekanja(self, stepL, stepD):
        """Računa koliko treba da se sačeka da Arduino završi pokret."""
        koraci_najveci = max(abs(stepL), abs(stepD))
        brzina = max(BRZINA_L, BRZINA_D)
        if brzina <= 0:
            return 0.0
        return (koraci_najveci / brzina) * MARGINA_SIGURNOSTI

    def posalji_komandu(self, stepL, stepD):
        """Pakuje i šalje poruku Arduinu, zatim čeka da se pokret završi."""
        stepL_int = int(round(stepL))
        stepD_int = int(round(stepD))

        poruka = Int32MultiArray()
        poruka.data = [stepL_int, BRZINA_L, stepD_int, BRZINA_D]
        self.kretanje_pub.publish(poruka)

        vreme_cekanja = self.izracunaj_vreme_cekanja(stepL_int, stepD_int)
        if vreme_cekanja > 0:
            self.get_logger().info(
                f"Šaljem komandu (L={stepL_int}, D={stepD_int}), "
                f"čekam {vreme_cekanja:.2f}s da se izvrši."
            )
            time.sleep(vreme_cekanja)
            time.sleep(0.5)  # Dodatna stabilizacija fizičkog zaustavljanja

    def path_callback(self, msg):
        if not msg.poses:
            return

        self.get_logger().info(f"Izvršavam novu putanju ({len(msg.poses)} tačaka)...")

        for pose_stamped in msg.poses:
            target_x = pose_stamped.pose.position.x
            target_y = pose_stamped.pose.position.y

            # 1. VEKTOR TRANSLACIJE (u metrima)
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance_meters = math.sqrt(dx**2 + dy**2)
            
            if distance_meters < 0.05:  # Greška do 5cm se toleriše
                continue

            # 2. RAČUNANJE I NORMALIZACIJA UGLA ZA OKRET
            target_theta = math.atan2(dy, dx)
            delta_theta = target_theta - self.current_theta
            delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))
            delta_degrees = math.degrees(delta_theta)

            # 3. IZVRŠI OKRET (ako je promena veća od 3 stepena)
            if abs(delta_degrees) > 3:
                self.get_logger().info(f"Okretanje: {delta_degrees:.2f}°")
                
                # Skaliranje stepeni u korake na osnovu konstante STEPOVI_ZA_90_STEPENI
                koraci_okreta = (delta_degrees / 90.0) * STEPOVI_ZA_90_STEPENI
                
                # HARDVERSKO PRAVILO za okret ulevo (+ugao): Levi (+), Desni (+)
                # S obzirom da je formula linearna, negativan ugao automatski menja smer u (-) za oba točka
                self.posalji_komandu(koraci_okreta, koraci_okreta)
                self.current_theta = target_theta

            # 4. TRANSLACIJA (Idi pravo)
            # Konvertujemo metre iz ROS-a u milimetre za tvoj hardverski model
            distance_mm = distance_meters * 1000.0
            stepovi_pravo = distance_mm / MM_PER_STEP
            
            self.get_logger().info(f"Vožnja pravo: {distance_meters:.2f}m ({int(stepovi_pravo)} koraka)")
            
            # HARDVERSKO PRAVILO za pravo: Levi (-), Desni (+)
            self.posalji_komandu(-stepovi_pravo, stepovi_pravo)

            # Ažuriranje interne pozicije robota
            self.current_x = target_x
            self.current_y = target_y

        # --- KRAJ RUTE ---
        self.get_logger().info("Putanja uspešno završena. Pokrećem LiDAR skeniranje...")
        self.trigger_lidar_scan()

    def trigger_lidar_scan(self):
        if not self.lidar_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("LiDAR servis nedostupan! Skeniranje otkazano.")
            return

        req = Trigger.Request()
        future = self.lidar_client.call_async(req)
        future.add_done_callback(self.lidar_scan_response_callback)

    def lidar_scan_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("LiDAR skeniranje uspešno završeno. Mapa osvežena.")
            else:
                self.get_logger().warn(f"LiDAR skeniranje prijavilo grešku: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Poziv LiDAR servisa neuspešan: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
