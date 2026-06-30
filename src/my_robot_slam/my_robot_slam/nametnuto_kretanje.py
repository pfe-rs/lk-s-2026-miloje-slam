#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import math
import time

# --- HARDVERSKE KONFIGURACIJE ---
BROJ_STEPOVA_PO_REVOLUCIJI = 400
POLUPRECNIK_TOCKA_MM = 20
RAZMAK_TOCKOVA_MM = 110

# Put po jednom stepu (u mm)
MM_PER_STEP = (2 * POLUPRECNIK_TOCKA_MM * math.pi) / BROJ_STEPOVA_PO_REVOLUCIJI

STEPOVI_ZA_90_STEPENI = 275
BRZINA_L = 200
BRZINA_D = 200

# Pretpostavka: BRZINA_L/BRZINA_D su koraci u sekundi (steps/s).
# Ako je u pitanju nesto drugo (npr. PWM vrednost), ovu konstantu
# i funkciju izracunaj_vreme_cekanja treba prilagoditi.
MARGINA_SIGURNOSTI = 1.15  # +15% zbog ubrzanja/usporavanja motora


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
        self.kretanje_sub = self.create_subscription(
            Int32MultiArray, '/podaci_kretanja', self.callback, 10
        )
        self.kretanje_pub = self.create_publisher(
            Int32MultiArray, '/podaci_motion_planner', 10
        )

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

    def okreni_robota(self, ciljni_ugao):
        """Okreće robota na željeni pravac prateći hardverske smerove."""
        razlika = (ciljni_ugao - self.trenutna_orijentacija) % 360

        if razlika == 0:
            return  # Već gleda u dobrom pravcu

        if razlika == 90:    # Okret ulevo (kontra-kazaljka)
            # Levi ide unazad (+), desni ide unapred (+)
            self.posalji_komandu(STEPOVI_ZA_90_STEPENI, STEPOVI_ZA_90_STEPENI)
        elif razlika == 270:  # Okret udesno (smer kazaljke)
            # Levi ide unapred (-), desni ide unazad (-)
            self.posalji_komandu(-STEPOVI_ZA_90_STEPENI, -STEPOVI_ZA_90_STEPENI)
        elif razlika == 180:  # Okret za 180 stepeni
            # Dupli okret udesno
            self.posalji_komandu(-STEPOVI_ZA_90_STEPENI * 2, -STEPOVI_ZA_90_STEPENI * 2)

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
            potreban_ugao = 0 if razlika_x > 0 else 180
            self.okreni_robota(potreban_ugao)

            stepovi_pravo = abs(razlika_x) / MM_PER_STEP
            # HARDVERSKO PRAVILO: Levi točak (-), Desni točak (+) za kretanje napred
            self.posalji_komandu(-stepovi_pravo, stepovi_pravo)
            self.trenutno_x = ciljno_x

        # --- SLUČAJ 2: Kretanje duž Y ose ---
        elif razlika_y != 0:
            potreban_ugao = 90 if razlika_y > 0 else 270
            self.okreni_robota(potreban_ugao)

            stepovi_pravo = abs(razlika_y) / MM_PER_STEP
            # HARDVERSKO PRAVILO: Levi točak (-), Desni točak (+) za kretanje napred
            self.posalji_komandu(-stepovi_pravo, stepovi_pravo)
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