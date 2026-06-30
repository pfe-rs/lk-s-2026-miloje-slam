#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import math

# --- HARDVERSKE KONFIGURACIJE ---
BROJ_STEPOVA_PO_REVOLUCIJI = 400  
POLUPRECNIK_TOCKA_MM = 20         
RAZMAK_TOCKOVA_MM = 110           

# Put po jednom stepu (u mm)
MM_PER_STEP = (2 * POLUPRECNIK_TOCKA_MM * math.pi) / BROJ_STEPOVA_PO_REVOLUCIJI
STEPOVI_ZA_90_STEPENI = 275

BRZINA_L = 200
BRZINA_D = 200

class NametnutoKretanjeNode(Node):
    def __init__(self):
        super().__init__("nametnuto_kretanje")
        self.get_logger().info("Čvor za nametnuto kretanje je pokrenut.")
        
        # Početne koordinate robota
        self.trenutno_x = 440 * 3 
        self.trenutno_y = 440    
        
        # 0 = gleda duž +X, 90 = gleda duž +Y, 180 = gleda duž -X, 270 = gleda duž -Y
        self.trenutna_orijentacija = 0 

        # Subscriber i Publisher
        self.kretanje_sub = self.create_subscription(Int32MultiArray, '/podaci_kretanja', self.callback, 10)
        self.kretanje_pub = self.create_publisher(Int32MultiArray, '/podaci_motion_planner', 10)

    def posalji_komandu(self, stepL, stepD):
        """Pakuje i šalje poruku Arduinu"""
        poruka = Int32MultiArray()
        poruka.data = [int(stepL), BRZINA_L, int(stepD), BRZINA_D]
        self.kretanje_pub.publish(poruka)

    def okreni_robota(self, ciljni_ugao):
        """Okreće robota na željeni pravac (0, 90, 180, 270)"""
        razlika = (ciljni_ugao - self.trenutna_orijentacija) % 360
        
        if razlika == 0:
            return  # Već gleda u dobrom pravcu
            
        if razlika == 90:    # Okret ulevo
            self.posalji_komandu(-STEPOVI_ZA_90_STEPENI, STEPOVI_ZA_90_STEPENI)
        elif razlika == 270:  # Okret udesno (360 - 270 = 90 udesno)
            self.posalji_komandu(STEPOVI_ZA_90_STEPENI, -STEPOVI_ZA_90_STEPENI)
        elif razlika == 180:  # Okret za 180 stepeni
            self.posalji_komandu(STEPOVI_ZA_90_STEPENI * 2, -STEPOVI_ZA_90_STEPENI * 2)
            
        self.trenutna_orijentacija = ciljni_ugao
        self.get_logger().info(f"Robot promenio pravac na: {ciljni_ugao}°")

    def callback(self, msg):
        if len(msg.data) < 2:
            return

        maks_granica = 440 * 4
        ciljno_x = max(0, min(msg.data[0], maks_granica))
        ciljno_y = max(0, min(msg.data[1], maks_granica))

        razlika_x = ciljno_x - self.trenutno_x
        razlika_y = ciljno_y - self.trenutno_y

        # --- SLUČAJ 1: Kretanje duž X ose ---
        if razlika_x != 0:
            # Odredi gde robot treba da gleda
            potreban_ugao = 0 if razlika_x > 0 else 180
            self.okreni_robota(potreban_ugao)
            
            # Izračunaj korake za pravo i pošalji komandu
            stepovi_pravo = abs(razlika_x) / MM_PER_STEP
            self.posalji_komandu(stepovi_pravo, stepovi_pravo)
            
            self.trenutno_x = ciljno_x

        # --- SLUČAJ 2: Kretanje duž Y ose ---
        elif razlika_y != 0:
            # Odredi gde robot treba da gleda
            potreban_ugao = 90 if razlika_y > 0 else 270
            self.okreni_robota(potreban_ugao)
            
            # Izračunaj korake za pravo i pošalji komandu
            stepovi_pravo = abs(razlika_y) / MM_PER_STEP
            self.posalji_komandu(stepovi_pravo, stepovi_pravo)
            
            self.trenutno_y = ciljno_y

def main(args=None):    
    rclpy.init(args=args)
    node = NametnutoKretanjeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()